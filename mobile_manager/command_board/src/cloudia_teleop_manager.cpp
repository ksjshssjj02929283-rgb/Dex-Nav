#include "cloudia_teleop_manager.hpp"

#include <curl/curl.h>

#include <chrono>
#include <cstdlib>
#include <stdexcept>

namespace command_board {
namespace {

std::string Env(const char* name, const std::string& fallback = "") {
    const char* value = std::getenv(name);
    return value ? value : fallback;
}

void Positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
}

CloudiaTeleopState::Config ReadPolicy(rclcpp::Node* node) {
    CloudiaTeleopState::Config config;
    config.input_timeout = node->declare_parameter("cloudia.input_timeout_sec", config.input_timeout);
    config.safety_timeout = node->declare_parameter("cloudia.safety_timeout_sec", config.safety_timeout);
    config.remote_timeout = node->declare_parameter("cloudia.state_timeout_sec", config.remote_timeout);
    config.takeover_timeout = node->declare_parameter("cloudia.takeover_timeout_sec", config.takeover_timeout);
    Positive(config.input_timeout, "cloudia.input_timeout_sec");
    Positive(config.safety_timeout, "cloudia.safety_timeout_sec");
    Positive(config.remote_timeout, "cloudia.state_timeout_sec");
    Positive(config.takeover_timeout, "cloudia.takeover_timeout_sec");
    return config;
}

size_t ReceiveBody(char* data, size_t size, size_t count, void* target) {
    auto& body = *static_cast<std::string*>(target);
    const size_t bytes = size * count;
    if (body.size() + bytes > 1024 * 1024) return 0;
    try {
        body.append(data, bytes);
        return bytes;
    } catch (...) {
        return 0;
    }
}

double Number(const nlohmann::json& object, const char* key) {
    const auto& value = object.at(key);
    if (!value.is_number()) throw std::runtime_error(std::string("missing numeric ") + key);
    const double number = value.get<double>();
    if (!std::isfinite(number)) throw std::runtime_error("non-finite Cloudia velocity");
    return number;
}

bool TrueFlag(const nlohmann::json& object, const char* key) {
    const auto it = object.find(key);
    return it != object.end() && it->is_boolean() && it->get<bool>();
}

}  // namespace

double CloudiaTeleopManager::Now() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

CloudiaTeleopManager::CloudiaTeleopManager(rclcpp::Node* node)
    : node_(node), state_(ReadPolicy(node)) {
    host_ = node_->declare_parameter("cloudia.host", Env("CLOUDIA_HOST"));
    if (host_.empty()) host_ = Env("CLOUDIA_HOST");
    // Keep credentials out of YAML/logs. HTTP requests never print headers.
    const auto token_env = node_->declare_parameter<std::string>("cloudia.token_env", "CLOUDIA_TOKEN");
    token_ = Env(token_env.c_str());
    task_port_ = node_->declare_parameter("cloudia.task_port", 8002);
    state_port_ = node_->declare_parameter("cloudia.state_port", 8852);
    http_timeout_ = node_->declare_parameter("cloudia.http_timeout_sec", 1.0);
    poll_period_ = node_->declare_parameter("cloudia.poll_period_sec", 0.2);
    stop_repeat_ = node_->declare_parameter("cloudia.stop_repeat_sec", 0.5);
    linear_still_ = node_->declare_parameter("cloudia.linear_still_threshold", 0.02);
    angular_still_ = node_->declare_parameter("cloudia.angular_still_threshold", 0.02);
    Positive(http_timeout_, "cloudia.http_timeout_sec");
    Positive(poll_period_, "cloudia.poll_period_sec");
    Positive(stop_repeat_, "cloudia.stop_repeat_sec");
    Positive(linear_still_, "cloudia.linear_still_threshold");
    Positive(angular_still_, "cloudia.angular_still_threshold");
    if (host_.empty() || host_.find_first_of("/?#@ \t\r\n") != std::string::npos ||
        token_.empty() || token_.find_first_of("\r\n") != std::string::npos ||
        task_port_ < 1 || task_port_ > 65535 || state_port_ < 1 || state_port_ > 65535) {
        throw std::invalid_argument("Cloudia needs a bare host, valid ports, and a nonempty token environment variable");
    }

    const auto raw_topic = node_->declare_parameter<std::string>(
        "cloudia.raw_velocity_topic", "/control/cmd_vel_cloudia_joy_raw");
    const auto output_topic = node_->declare_parameter<std::string>(
        "cloudia.velocity_topic", "/control/cmd_vel_cloudia_joy");
    const auto joy_topic = node_->declare_parameter<std::string>("cloudia.joy_state_topic", "/control/joy_in");
    if (raw_topic == output_topic) throw std::invalid_argument("Cloudia input/output topics must differ");
    static std::once_flag curl_init;
    std::call_once(curl_init, []() {
        if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
            throw std::runtime_error("curl_global_init failed");
        }
    });
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto qos = rclcpp::QoS(1).reliable();
    velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(output_topic, qos);
    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        raw_topic, qos,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.OnVelocity({msg->linear.x, msg->linear.y, msg->angular.z}, Now());
        }, options);
    joy_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        joy_topic, qos,
        [this](std_msgs::msg::Int16::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.OnJoy(msg->data, Now());
        }, options);
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&CloudiaTeleopManager::Tick, this), callback_group_);
    worker_ = std::thread(&CloudiaTeleopManager::Worker, this);
    RCLCPP_INFO(node_->get_logger(),
                "Cloudia takeover: %s -> %s; awaiting safety and remote state",
                raw_topic.c_str(), output_topic.c_str());
}

CloudiaTeleopManager::~CloudiaTeleopManager() {
    if (timer_) timer_->cancel();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        if (state_.phase() != CloudiaTeleopState::Phase::Idle) {
            try { PublishZeroLocked(); } catch (...) {}
        }
    }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void CloudiaTeleopManager::PublishZeroLocked() {
    if (velocity_pub_) velocity_pub_->publish(geometry_msgs::msg::Twist{});
}

void CloudiaTeleopManager::UpdateSafety(bool fault) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_.phase();
    state_.UpdateSafety(fault, Now());
    if (fault) {
        PublishZeroLocked();
        if (before != CloudiaTeleopState::Phase::Fault) urgent_stop_ = true;
    }
    wake_.notify_all();
}

void CloudiaTeleopManager::RequestStop(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.RequestStop(Now(), reason);
    urgent_stop_ = true;
    PublishZeroLocked();
    wake_.notify_all();
}

void CloudiaTeleopManager::Tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = Now();
    const auto before = state_.phase();
    const auto output = state_.Tick(now);
    if (state_.phase() != before) {
        RCLCPP_INFO(node_->get_logger(), "Cloudia control %s -> %s: %s",
                    CloudiaTeleopState::Name(before), CloudiaTeleopState::Name(state_.phase()),
                    state_.reason().c_str());
        if (state_.phase() == CloudiaTeleopState::Phase::Fault ||
            state_.phase() == CloudiaTeleopState::Phase::Error) urgent_stop_ = true;
        wake_.notify_all();
    }
    if (output.publish) {
        geometry_msgs::msg::Twist message;
        message.linear.x = output.velocity.x;
        message.linear.y = output.velocity.y;
        message.angular.z = output.velocity.yaw;
        // Serialize with safety callbacks so an old nonzero output cannot be
        // published after a newer zero safety command.
        velocity_pub_->publish(message);
    }
}

nlohmann::json CloudiaTeleopManager::Http(
    const std::string& method, int port, const std::string& path, bool authenticated) {
    CURL* handle = curl_easy_init();
    if (!handle) throw std::runtime_error("cannot create HTTP handle");
    struct curl_slist* headers = nullptr;
    if (authenticated) headers = curl_slist_append(headers, ("X-Robot-Control-Token: " + token_).c_str());
    std::string body;
    const std::string url = "http://" + host_ + ":" + std::to_string(port) + path;
    curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(http_timeout_ * 1000));
    curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, static_cast<long>(http_timeout_ * 1000));
    curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, ReceiveBody);
    curl_easy_setopt(handle, CURLOPT_WRITEDATA, &body);
    if (method == "POST") {
        curl_easy_setopt(handle, CURLOPT_POST, 1L);
        curl_easy_setopt(handle, CURLOPT_POSTFIELDS, "");
    }
    const CURLcode result = curl_easy_perform(handle);
    long status = 0;
    curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
    curl_slist_free_all(headers);
    curl_easy_cleanup(handle);
    if (result != CURLE_OK) throw std::runtime_error(std::string("Cloudia HTTP: ") + curl_easy_strerror(result));
    if (status != 200) throw std::runtime_error("Cloudia HTTP status " + std::to_string(status));
    auto payload = nlohmann::json::parse(body);
    if (!payload.is_object()) throw std::runtime_error("Cloudia HTTP response is not an object");
    return payload;
}

void CloudiaTeleopManager::StopTask() {
    const auto reply = Http("POST", task_port_, "/api/v1/task/stop", true);
    if (!reply.contains("code") || !reply.at("code").is_number_integer() || reply.at("code") != 0) {
        throw std::runtime_error("Cloudia rejected task stop");
    }
}

CloudiaTeleopState::Remote CloudiaTeleopManager::ReadRemote() {
    const auto status = Http("GET", task_port_, "/api/v1/robot/status", true);
    const auto& robot = status.at("status").at("robotStatus");
    const auto& information_type = robot.contains("informationType")
        ? robot.at("informationType") : robot.at("state");
    if (!information_type.is_number_integer()) throw std::runtime_error("invalid Cloudia robot state");
    const int info = information_type.get<int>();
    const auto& task = status.at("task");
    bool running = false;
    int64_t task_id = 0;
    if (!task.is_null()) {
        if (!task.at("taskId").is_number_integer()) throw std::runtime_error("invalid Cloudia task ID");
        task_id = task.at("taskId").get<int64_t>();
        const auto task_status = task.at("taskStatus").get<std::string>();
        if (task_status != "running" && task_status != "finished" && task_status != "failed") {
            throw std::runtime_error("invalid Cloudia task status");
        }
        running = task_status == "running";
    }
    const auto realtime = Http("GET", state_port_, "/get/state", false);
    if (realtime.value("communicationType", "") != "upstream_state") {
        throw std::runtime_error("invalid Cloudia realtime state");
    }
    const auto& velocity = realtime.at("state").at("velocity");
    const double x = Number(velocity, "vx"), y = Number(velocity, "vy"), yaw = Number(velocity, "omega");
    const auto& information = realtime.at("information");
    int realtime_info = info;
    if (information.contains("informationType")) {
        if (!information.at("informationType").is_number_integer()) {
            throw std::runtime_error("invalid Cloudia realtime robot state");
        }
        realtime_info = information.at("informationType").get<int>();
    }
    const bool emergency = TrueFlag(information, "emergency");
    const bool collision = TrueFlag(information, "collision");
    // The two HTTP snapshots are not atomic: either may observe NAV_STOP first.
    const bool obstacle = TrueFlag(robot, "obstacleStop") || TrueFlag(information, "obstacleStop");
    const auto navigation_state = [](int value) { return value >= 1 && value <= 3; };
    CloudiaTeleopState::Remote result;
    result.valid = true;
    result.running = running;
    result.navigation_blocked = obstacle && navigation_state(info) && navigation_state(realtime_info) &&
                                !emergency && !collision;
    result.ready = info == 1 && realtime_info == 1 && !result.navigation_blocked;
    // Only ordinary navigation stops are exempted. Error, emergency, collision,
    // initialization, charging and unknown states retain the existing protection.
    result.fault = !result.navigation_blocked &&
                   ((info != 1 && info != 2) || (realtime_info != 1 && realtime_info != 2) ||
                    emergency || collision || obstacle);
    result.stopped = std::hypot(x, y) <= linear_still_ && std::abs(yaw) <= angular_still_;
    result.received = Now();
    result.task_id = task_id;
    return result;
}

void CloudiaTeleopManager::Worker() {
    double last_stop = -1.0;
    while (true) {
        bool urgent;
        uint64_t revision;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait_for(lock, std::chrono::duration<double>(poll_period_),
                           [this]() { return shutdown_ || urgent_stop_; });
            if (shutdown_) return;
            urgent = urgent_stop_;
            urgent_stop_ = false;
            revision = state_.revision();
        }
        // Fault stop is immediate. If a later query exposes a running gateway
        // task, keep requesting stop while the safety latch is held.
        if (urgent) {
            try { StopTask(); } catch (const std::exception& error) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                    "Cloudia safety stop not acknowledged: %s", error.what());
            }
            last_stop = Now();
        }
        try {
            auto remote = ReadRemote();
            bool stop = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (revision != state_.revision()) continue;
                if (remote.navigation_blocked != state_.remote().navigation_blocked) {
                    RCLCPP_INFO(node_->get_logger(), "Cloudia ordinary navigation-stop state %s",
                                remote.navigation_blocked ? "active; nonzero teleop inhibited" : "inactive");
                }
                state_.SetRemote(remote);
                stop = state_.NeedsTaskStop() && remote.running && Now() - last_stop >= stop_repeat_;
                if (stop) state_.InvalidateRemote();
            }
            if (stop) {
                last_stop = Now();
                StopTask();
                // A successful POST means accepted, NOT stopped; the next GET
                // must prove that no task is running and velocity is zero.
            }
        } catch (const std::exception& error) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.SetRemote({});
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                "Cloudia state/stop unavailable: %s", error.what());
        }
    }
}

}  // namespace command_board

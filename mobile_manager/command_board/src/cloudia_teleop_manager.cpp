#include "cloudia_teleop_manager.hpp"

#include <curl/curl.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
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

std::string NewId() {
    std::random_device random;
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i) stream << std::setw(8) << random();
    return stream.str();
}

bool ValidId(const std::string& id) {
    return !id.empty() && id.size() <= 128 &&
        std::all_of(id.begin(), id.end(), [](unsigned char c) {
            return std::isalnum(c) || c == '-' || c == '_';
        });
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
    : node_(node), state_(ReadPolicy(node)), coordinator_id_(NewId()) {
    host_ = node_->declare_parameter("cloudia.host", Env("CLOUDIA_HOST"));
    if (host_.empty()) host_ = Env("CLOUDIA_HOST");
    // Keep credentials out of YAML/logs. HTTP requests never print headers.
    const auto token_env = node_->declare_parameter<std::string>("cloudia.token_env", "CLOUDIA_TOKEN");
    token_ = Env(token_env.c_str());
    task_port_ = node_->declare_parameter("cloudia.task_port", 8002);
    state_port_ = node_->declare_parameter("cloudia.state_port", 8852);
    http_timeout_ = node_->declare_parameter("cloudia.http_timeout_sec", 1.0);
    poll_period_ = node_->declare_parameter("cloudia.poll_period_sec", 0.2);
    submission_timeout_ = node_->declare_parameter("cloudia.submission_timeout_sec", 10.0);
    stop_repeat_ = node_->declare_parameter("cloudia.stop_repeat_sec", 0.5);
    linear_still_ = node_->declare_parameter("cloudia.linear_still_threshold", 0.02);
    angular_still_ = node_->declare_parameter("cloudia.angular_still_threshold", 0.02);
    Positive(http_timeout_, "cloudia.http_timeout_sec");
    Positive(poll_period_, "cloudia.poll_period_sec");
    Positive(submission_timeout_, "cloudia.submission_timeout_sec");
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
    const auto acquire_name = node_->declare_parameter<std::string>(
        "cloudia.request_submission_service", "/command_board/cloudia/request_task_submission");
    const auto report_name = node_->declare_parameter<std::string>(
        "cloudia.report_submission_service", "/command_board/cloudia/report_task_submission");
    journal_path_ = node_->declare_parameter(
        "cloudia.submission_journal",
        Env("HOME", "/tmp") + "/.ros/cloudia_task_submissions.json");
    if (journal_path_.empty() || journal_path_.front() != '/') {
        throw std::invalid_argument("cloudia.submission_journal must be an absolute path");
    }

    static std::once_flag curl_init;
    std::call_once(curl_init, []() {
        if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
            throw std::runtime_error("curl_global_init failed");
        }
    });
    try {
        LoadJournal();
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
        acquire_service_ = node_->create_service<Acquire>(
            acquire_name, std::bind(&CloudiaTeleopManager::OnAcquire, this,
                                   std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, callback_group_);
        report_service_ = node_->create_service<Report>(
            report_name, std::bind(&CloudiaTeleopManager::OnReport, this,
                                  std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, callback_group_);
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&CloudiaTeleopManager::Tick, this), callback_group_);
        worker_ = std::thread(&CloudiaTeleopManager::Worker, this);
    } catch (...) {
        if (journal_lock_fd_ >= 0) ::close(journal_lock_fd_);
        journal_lock_fd_ = -1;
        throw;
    }
    RCLCPP_INFO(node_->get_logger(),
                "Cloudia takeover: %s -> %s; pending submissions=%zu; awaiting safety and remote state",
                raw_topic.c_str(), output_topic.c_str(), pending_.size());
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
    if (journal_lock_fd_ >= 0) ::close(journal_lock_fd_);
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

bool CloudiaTeleopManager::SubmissionUncertainLocked(double now) const {
    for (const auto& entry : pending_) {
        if ((entry.second.reported && entry.second.outcome == Report::Request::UNCERTAIN) ||
            now - entry.second.created > submission_timeout_) return true;
    }
    return journal_fault_;
}

void CloudiaTeleopManager::OnAcquire(
    const Acquire::Request::SharedPtr request, Acquire::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response->allowed = false;
    response->coordinator_id = coordinator_id_;
    if (!ValidId(request->request_id)) {
        response->message = "invalid request_id";
        return;
    }
    if (pending_.count(request->request_id) || completed_.contains(request->request_id)) {
        // Never issue a second grant for an old UUID, including a delayed request
        // whose NOT_SENT report arrived first.
        response->message = "request_id already registered; use a new UUID only for a new submission";
        return;
    }
    const double now = Now();
    // Run policy at the submission boundary, not just at the next 20 ms tick.
    state_.Tick(now, !pending_.empty(), SubmissionUncertainLocked(now));
    if (!state_.CanSubmit(now, !pending_.empty()) || journal_fault_ || urgent_stop_) {
        response->message = std::string("submission blocked: ") +
                            CloudiaTeleopState::Name(state_.phase()) + "; " + state_.reason();
        return;
    }
    Submission item;
    item.coordinator_id = coordinator_id_;
    item.created = now;
    pending_.emplace(request->request_id, item);
    state_.InvalidateRemote();
    // Durably register BEFORE returning a grant. A coordinator restart may never
    // silently forget a request that a different process is still submitting.
    if (!PersistLocked()) {
        response->message = "cannot persist submission registration";
        return;
    }
    response->allowed = true;
    response->message = "registered; report HTTP outcome using this request_id";
}

void CloudiaTeleopManager::OnReport(
    const Report::Request::SharedPtr request, Report::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response->recorded = false;
    if (!ValidId(request->request_id) || request->outcome > Report::Request::NOT_SENT ||
        (request->outcome == Report::Request::ACCEPTED && request->task_id <= 0)) {
        response->message = "invalid report";
        return;
    }
    const auto done = completed_.find(request->request_id);
    if (done != completed_.end()) {
        response->recorded = !journal_fault_ && done->at("outcome") == request->outcome &&
                             done->at("task_id") == request->task_id &&
                             (request->outcome == Report::Request::NOT_SENT ||
                              done->at("coordinator_id") == request->coordinator_id);
        response->message = response->recorded ? "already recorded" : "conflicting final report";
        return;
    }
    auto found = pending_.find(request->request_id);
    if (found == pending_.end() && request->outcome != Report::Request::NOT_SENT) {
        // An unregistered HTTP result may represent a task started outside the
        // contract. Stop and require investigation rather than guessing ownership.
        journal_fault_ = true;
        state_.Fail(Now(), "result for unregistered task submission");
        urgent_stop_ = true;
        PublishZeroLocked();
        wake_.notify_all();
        response->message = "unregistered submission; control blocked";
        return;
    }
    if (found != pending_.end() &&
        !(request->outcome == Report::Request::NOT_SENT && request->coordinator_id.empty()) &&
        request->coordinator_id != found->second.coordinator_id) {
        response->message = "coordinator_id mismatch";
        return;
    }
    if (request->outcome == Report::Request::UNCERTAIN) {
        found->second.reported = true;
        found->second.outcome = request->outcome;
        state_.Fail(Now(), "HTTP submission outcome uncertain");
        urgent_stop_ = true;
    } else {
        if (found != pending_.end()) pending_.erase(found);
        if (request->outcome == Report::Request::ACCEPTED) awaiting_task_id_ = request->task_id;
        completed_[request->request_id] = {
            {"coordinator_id", request->coordinator_id},
            {"outcome", request->outcome}, {"task_id", request->task_id}, {"finished", Now()}};
        // Retain recent final reports for idempotent acknowledgement retries.
        while (completed_.size() > 2048) {
            auto oldest = std::min_element(completed_.begin(), completed_.end(),
                [](const nlohmann::json& a, const nlohmann::json& b) {
                    return a.value("finished", 0.0) < b.value("finished", 0.0);
                });
            completed_.erase(oldest);
        }
        state_.InvalidateRemote();
    }
    response->recorded = PersistLocked();
    response->message = response->recorded
        ? (request->outcome == Report::Request::UNCERTAIN
            ? "uncertainty retained; no new task or teleop grant" : "recorded")
        : "cannot persist report";
    wake_.notify_all();
}

void CloudiaTeleopManager::Tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = Now();
    const auto before = state_.phase();
    const auto output = state_.Tick(now, !pending_.empty(), SubmissionUncertainLocked(now));
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
    CloudiaTeleopState::Remote result;
    result.valid = true;
    result.running = running;
    result.ready = info == 1;
    result.fault = (info != 1 && info != 2) || TrueFlag(information, "emergency") ||
                   TrueFlag(information, "collision") || TrueFlag(information, "obstacleStop");
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
        // Fault stop is immediate, even with in-flight task submissions. A later
        // query/report can expose another task; repeat stop until it is settled.
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
                // Do not mistake a previous terminal task for the task just
                // accepted. Wait until the accepted ID appears in global state.
                if (awaiting_task_id_ != 0) {
                    if (remote.task_id == awaiting_task_id_) {
                        awaiting_task_id_ = 0;
                        PersistLocked();
                    } else {
                        remote.valid = false;
                    }
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

void CloudiaTeleopManager::LoadJournal() {
    namespace fs = std::filesystem;
    fs::create_directories(fs::path(journal_path_).parent_path());
    journal_lock_fd_ = ::open((journal_path_ + ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (journal_lock_fd_ < 0 || ::flock(journal_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
        throw std::runtime_error("cannot lock Cloudia submission journal; another coordinator may be running");
    }
    if (!fs::exists(journal_path_)) return;
    std::ifstream input(journal_path_);
    const auto journal = nlohmann::json::parse(input);
    if (journal.at("version") != 1) throw std::runtime_error("unknown Cloudia submission journal version");
    completed_ = journal.at("completed");
    awaiting_task_id_ = journal.value("awaiting_task_id", int64_t{0});
    if (!completed_.is_object() || !journal.at("pending").is_object() || awaiting_task_id_ < 0) {
        throw std::runtime_error("invalid Cloudia submission journal");
    }
    for (auto& entry : completed_.items()) entry.value()["finished"] = 0.0;
    for (const auto& entry : journal.at("pending").items()) {
        if (!ValidId(entry.key()) || completed_.contains(entry.key())) {
            throw std::runtime_error("invalid/conflicting Cloudia pending request ID");
        }
        Submission item;
        item.coordinator_id = entry.value().at("coordinator_id").get<std::string>();
        if (item.coordinator_id.empty()) throw std::runtime_error("pending submission has no coordinator ID");
        item.reported = true;
        item.outcome = Report::Request::UNCERTAIN;
        item.created = Now();
        pending_.emplace(entry.key(), std::move(item));
    }
}

bool CloudiaTeleopManager::PersistLocked() {
    nlohmann::json records = nlohmann::json::object();
    for (const auto& entry : pending_) {
        records[entry.first] = {{"coordinator_id", entry.second.coordinator_id}};
    }
    const std::string bytes = nlohmann::json{
        {"version", 1}, {"pending", records}, {"completed", completed_},
        {"awaiting_task_id", awaiting_task_id_}}.dump();
    const std::string temporary = journal_path_ + ".tmp";
    int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    bool ok = fd >= 0;
    size_t offset = 0;
    while (ok && offset < bytes.size()) {
        const auto written = ::write(fd, bytes.data() + offset, bytes.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) { ok = false; break; }
        offset += static_cast<size_t>(written);
    }
    if (ok) ok = ::fsync(fd) == 0;
    if (fd >= 0 && ::close(fd) != 0) ok = false;
    if (ok) ok = ::rename(temporary.c_str(), journal_path_.c_str()) == 0;
    if (ok) {
        const auto parent = std::filesystem::path(journal_path_).parent_path().string();
        int directory = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
        ok = directory >= 0 && ::fsync(directory) == 0;
        if (directory >= 0) ::close(directory);
    }
    if (!ok) {
        journal_fault_ = true;
        state_.Fail(Now(), "submission journal write failed");
        urgent_stop_ = true;
        PublishZeroLocked();
        wake_.notify_all();
        RCLCPP_ERROR(node_->get_logger(), "Cloudia submission journal failed; motion/submission blocked");
    }
    return ok;
}

}  // namespace command_board

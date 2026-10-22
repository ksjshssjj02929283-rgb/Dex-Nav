#include "cloudia_teleop_manager.hpp"

#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <functional>

using namespace std::chrono_literals;
using Acquire = cloudia_control_interfaces::srv::RequestTaskSubmission;
using Report = cloudia_control_interfaces::srv::ReportTaskSubmission;

namespace {

// Loopback-only fake gateway: no real robot address is used by these tests.
class FakeGateway {
public:
    FakeGateway() {
        socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) throw std::runtime_error("test socket unavailable");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
            ::listen(socket_, 8) != 0) throw std::runtime_error("test bind failed");
        socklen_t length = sizeof(address);
        ::getsockname(socket_, reinterpret_cast<sockaddr*>(&address), &length);
        port = ntohs(address.sin_port);
        thread_ = std::thread([this]() { Run(); });
    }
    ~FakeGateway() {
        done_ = true;
        ::shutdown(socket_, SHUT_RDWR);
        ::close(socket_);
        thread_.join();
    }
    int port;
    std::atomic<bool> running{false}, fault{false};
    std::atomic<double> speed{0.0};
    std::atomic<int> stops{0}, task_id{0};

private:
    void Run() {
        while (!done_) {
            int client = ::accept(socket_, nullptr, nullptr);
            if (client < 0) continue;
            timeval timeout{1, 0};
            ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            std::string request;
            char buffer[2048];
            while (request.find("\r\n\r\n") == std::string::npos && request.size() < 8192) {
                const auto count = ::recv(client, buffer, sizeof(buffer), 0);
                if (count <= 0) break;
                request.append(buffer, static_cast<size_t>(count));
            }
            nlohmann::json body;
            if (request.find("POST /api/v1/task/stop ") == 0) {
                ++stops;
                running = false;
                speed = 0.0;
                body = {{"code", 0}, {"message", "success"}};
            } else if (request.find("GET /api/v1/robot/status ") == 0) {
                nlohmann::json task = nullptr;
                if (task_id > 0) {
                    task = {{"taskId", task_id.load()},
                            {"taskStatus", running ? "running" : "failed"}};
                }
                body = {{"status", {{"robotStatus", {
                    {"informationType", fault ? 4 : (running || speed != 0.0 ? 2 : 1)}}}}},
                        {"task", task}};
            } else {
                body = {{"communicationType", "upstream_state"},
                        {"state", {{"velocity", {{"vx", speed.load()}, {"vy", 0.0}, {"omega", 0.0}}}}},
                        {"information", {{"emergency", fault.load()}, {"collision", false},
                                         {"obstacleStop", false}}}};
            }
            const auto payload = body.dump();
            const std::string response =
                "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: " +
                std::to_string(payload.size()) + "\r\n\r\n" + payload;
            ::send(client, response.data(), response.size(), MSG_NOSIGNAL);
            ::close(client);
        }
    }
    int socket_;
    std::atomic<bool> done_{false};
    std::thread thread_;
};

class CloudiaManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        gateway = std::make_unique<FakeGateway>();
        static int sequence = 0;
        base = "/cloudia_test_" + std::to_string(::getpid()) + "_" + std::to_string(++sequence);
        journal = "/tmp" + base + "/journal.json";
        ::setenv("CLOUDIA_TEST_TOKEN", "loopback-test", 1);
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("cloudia.host", "127.0.0.1"),
            rclcpp::Parameter("cloudia.token_env", "CLOUDIA_TEST_TOKEN"),
            rclcpp::Parameter("cloudia.task_port", gateway->port),
            rclcpp::Parameter("cloudia.state_port", gateway->port),
            rclcpp::Parameter("cloudia.submission_journal", journal),
            rclcpp::Parameter("cloudia.poll_period_sec", 0.03),
            rclcpp::Parameter("cloudia.http_timeout_sec", 0.2),
            rclcpp::Parameter("cloudia.stop_repeat_sec", 0.05),
            rclcpp::Parameter("cloudia.raw_velocity_topic", base + "/raw"),
            rclcpp::Parameter("cloudia.velocity_topic", base + "/out"),
            rclcpp::Parameter("cloudia.joy_state_topic", base + "/joy"),
            rclcpp::Parameter("cloudia.request_submission_service", base + "/acquire"),
            rclcpp::Parameter("cloudia.report_submission_service", base + "/report"),
        });
        owner_options = options;
        owner = std::make_shared<rclcpp::Node>("cloudia_owner", options);
        peer = std::make_shared<rclcpp::Node>("cloudia_peer");
        manager = std::make_unique<command_board::CloudiaTeleopManager>(owner.get());
        acquire = peer->create_client<Acquire>(base + "/acquire");
        report = peer->create_client<Report>(base + "/report");
        raw = peer->create_publisher<geometry_msgs::msg::Twist>(base + "/raw", 1);
        joy = peer->create_publisher<std_msgs::msg::Int16>(base + "/joy", 1);
        output = peer->create_subscription<geometry_msgs::msg::Twist>(
            base + "/out", 1, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                last_velocity = msg->linear.x;
                ++output_count;
            });
        safety_timer = owner->create_wall_timer(20ms, [this]() { manager->UpdateSafety(safety_fault); });
        input_timer = peer->create_wall_timer(20ms, [this]() {
            std_msgs::msg::Int16 state;
            state.data = joy_state;
            joy->publish(state);
            geometry_msgs::msg::Twist velocity;
            velocity.linear.x = input_velocity;
            raw->publish(velocity);
        });
        executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(owner);
        executor->add_node(peer);
        spin = std::thread([this]() { executor->spin(); });
        ASSERT_TRUE(acquire->wait_for_service(2s));
        ASSERT_TRUE(report->wait_for_service(2s));
        std::this_thread::sleep_for(200ms);
    }
    void TearDown() override {
        executor->cancel();
        spin.join();
        executor->remove_node(owner);
        executor->remove_node(peer);
        safety_timer.reset();
        input_timer.reset();
        manager.reset();
        peer.reset();
        owner.reset();
        gateway.reset();
        std::filesystem::remove_all(std::filesystem::path(journal).parent_path());
    }
    bool Wait(const std::function<bool()>& condition, double seconds = 3.0) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            if (condition()) return true;
            std::this_thread::sleep_for(10ms);
        }
        return condition();
    }
    Acquire::Response::SharedPtr Request(const std::string& id) {
        auto request = std::make_shared<Acquire::Request>();
        request->request_id = id;
        auto future = acquire->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) throw std::runtime_error("acquire timeout");
        return future.get();
    }
    bool Result(const std::string& id, const std::string& epoch, uint8_t outcome, int64_t task = 0) {
        auto request = std::make_shared<Report::Request>();
        request->request_id = id;
        request->coordinator_id = epoch;
        request->outcome = outcome;
        request->task_id = task;
        auto future = report->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) throw std::runtime_error("report timeout");
        return future.get()->recorded;
    }
    void RestartManager() {
        executor->cancel();
        spin.join();
        executor->remove_node(owner);
        executor->remove_node(peer);
        safety_timer.reset();
        manager.reset();
        owner.reset();
        owner = std::make_shared<rclcpp::Node>("cloudia_owner", owner_options);
        manager = std::make_unique<command_board::CloudiaTeleopManager>(owner.get());
        safety_timer = owner->create_wall_timer(20ms, [this]() { manager->UpdateSafety(safety_fault); });
        executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(owner);
        executor->add_node(peer);
        spin = std::thread([this]() { executor->spin(); });
        std::this_thread::sleep_for(200ms);
    }
    std::unique_ptr<FakeGateway> gateway;
    std::shared_ptr<rclcpp::Node> owner, peer;
    std::unique_ptr<command_board::CloudiaTeleopManager> manager;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    rclcpp::Client<Acquire>::SharedPtr acquire;
    rclcpp::Client<Report>::SharedPtr report;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr joy;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output;
    rclcpp::TimerBase::SharedPtr input_timer, safety_timer;
    std::thread spin;
    rclcpp::NodeOptions owner_options;
    std::string base, journal;
    std::atomic<int16_t> joy_state{0};
    std::atomic<double> input_velocity{0.2}, last_velocity{0.0};
    std::atomic<int> output_count{0};
    std::atomic<bool> safety_fault{false};
};

TEST_F(CloudiaManagerTest, SubmissionBarrierStopAndForward) {
    auto grant = Request("inflight");
    ASSERT_TRUE(grant->allowed);
    joy_state = 1;
    ASSERT_TRUE(Wait([this]() { return output_count > 4; }));
    EXPECT_EQ(last_velocity, 0.0);
    EXPECT_EQ(gateway->stops, 0);  // wait for the HTTP result before normal takeover stop
    EXPECT_FALSE(Request("second")->allowed);
    gateway->task_id = 42;
    gateway->speed = 0.3;
    gateway->running = true;
    ASSERT_TRUE(Result("inflight", grant->coordinator_id, Report::Request::ACCEPTED, 42));
    ASSERT_TRUE(Result("inflight", grant->coordinator_id, Report::Request::ACCEPTED, 42));
    ASSERT_TRUE(Wait([this]() { return gateway->stops > 0 && last_velocity > 0.1; }));
    input_velocity = 0.05;  // existing joy=1 includes joystick-computed soft stop
    ASSERT_TRUE(Wait([this]() { return last_velocity == 0.05; }));
    input_velocity = 0.0;
    joy_state = 0;
    ASSERT_TRUE(Wait([this]() { return last_velocity == 0.0; }));
    std::this_thread::sleep_for(250ms);
    EXPECT_TRUE(Request("after_release")->allowed);
    EXPECT_TRUE(Result("after_release", "", Report::Request::NOT_SENT));
}

TEST_F(CloudiaManagerTest, FaultStopsBeforeInflightRequestReports) {
    auto grant = Request("pending_at_fault");
    ASSERT_TRUE(grant->allowed);
    gateway->task_id = 7;
    gateway->running = true;
    gateway->speed = 0.3;
    safety_fault = true;
    manager->UpdateSafety(true);
    ASSERT_TRUE(Wait([this]() { return gateway->stops > 0; }));
    EXPECT_EQ(last_velocity, 0.0);
    EXPECT_FALSE(Request("during_fault")->allowed);
    // Late acceptance must be stopped again while the safety latch is held.
    gateway->running = true;
    gateway->speed = 0.3;
    const int previous_stops = gateway->stops;
    ASSERT_TRUE(Result("pending_at_fault", grant->coordinator_id, Report::Request::ACCEPTED, 7));
    ASSERT_TRUE(Wait([this, previous_stops]() { return gateway->stops > previous_stops; }));
    EXPECT_EQ(last_velocity, 0.0);
}

TEST_F(CloudiaManagerTest, UnknownOutcomePersistsAndLateNotSentCannotGrant) {
    auto grant = Request("uncertain");
    ASSERT_TRUE(grant->allowed);
    ASSERT_TRUE(Result("uncertain", grant->coordinator_id, Report::Request::UNCERTAIN));
    std::ifstream stream(journal);
    const auto saved = nlohmann::json::parse(stream);
    EXPECT_TRUE(saved.at("pending").contains("uncertain"));
    EXPECT_FALSE(Request("blocked")->allowed);
    // A NOT_SENT report may arrive before a delayed acquire request.
    EXPECT_TRUE(Result("delayed_request", "", Report::Request::NOT_SENT));
    EXPECT_FALSE(Request("delayed_request")->allowed);
    joy_state = 1;
    ASSERT_TRUE(Wait([this]() { return output_count > 5; }));
    EXPECT_EQ(last_velocity, 0.0);
}

TEST_F(CloudiaManagerTest, RestartRetainsInflightRegistrationAndAcceptsOldEpochReport) {
    auto grant = Request("before_restart");
    ASSERT_TRUE(grant->allowed);
    RestartManager();
    auto blocked = Request("after_restart");
    EXPECT_FALSE(blocked->allowed);
    EXPECT_NE(blocked->coordinator_id, grant->coordinator_id);
    // The producer still knows it never sent HTTP. Only this definitive report,
    // not restart or an idle snapshot, can settle its recovered registration.
    ASSERT_TRUE(Result("before_restart", grant->coordinator_id, Report::Request::NOT_SENT));
    std::this_thread::sleep_for(200ms);
    EXPECT_TRUE(Request("settled")->allowed);
    EXPECT_TRUE(Result("settled", "", Report::Request::NOT_SENT));
}

}  // namespace

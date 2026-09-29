#ifndef AUTO_CHARGING_BT_NODES_HPP_
#define AUTO_CHARGING_BT_NODES_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_interfaces/msg/map_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>

// 前向声明
class AutoChargingNode;
class ReverseDockingBtController;

// 全局节点指针

// ==================== ROS节点类 ====================
class AutoChargingNode : public rclcpp::Node {
public:
    enum class ChargerState { UNKNOWN = 0, CHARGING = 1, DISCONNECTED = 2 };

    // 导航后端和异步导航结果都是外部接口字符串，集中定义后按字符串判断，
    // 避免枚举值与话题/Action 实际状态之间再维护一层隐式映射。
    inline static constexpr const char* kNavigationBackendNav2 = "nav2";
    inline static constexpr const char* kNavigationBackendPathNavigation = "path_navigation";
    inline static constexpr const char* kNavStatusRunning = "RUNNING";
    inline static constexpr const char* kNavStatusSucceeded = "SUCCEEDED";
    inline static constexpr const char* kNavStatusFailed = "FAILED";
    inline static constexpr const char* kNavStatusCancelled = "CANCELLED";

    struct NavigationBackendSnapshot {
        std::string name;
        // 仅当状态字符串实际变化时递增；同值重复发布不会改变 revision。
        std::uint64_t revision = 0;
    };

    AutoChargingNode();

    void cbDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    bool isCharging() const { return is_charging_; }
    double getBatteryPercentage() const { return battery_percentage_; }
    ChargerState confirmEntryChargerState();
    bool navToPose(const geometry_msgs::msg::PoseStamped& goal_pose);
    // 以下接口为 NavigateToCharging 改造为 StatefulActionNode 提供非阻塞支持，
    // 与 navigation_client.py 的 Action 状态约定保持一致：
    //   sendNavGoal 异步发送 Nav2 goal，立即返回；
    //   isNavDone 非阻塞轮询，返回 RUNNING/SUCCEEDED/FAILED/CANCELLED；
    //   cancelNav 发起定向取消，waitForNavCancellation 在任务退出前等待 Nav2 确认。
    bool sendNavGoal(const geometry_msgs::msg::PoseStamped& goal_pose);
    std::string isNavDone();
    void cancelNav();
    bool waitForNavCancellation(double timeout_sec);
    NavigationBackendSnapshot waitForNavigationBackend(double timeout_sec);
    bool navigationBackendChanged(const NavigationBackendSnapshot& expected) const;
    std::string currentNavigationBackendName() const { return navigation_backend_name_; }
    void setTaskInterruptionCheck(std::function<bool()> check);
    void clearTaskInterruptionCheck();
    bool taskInterruptionRequested() const;
    void publishTwist(double linear_x, double angular_z);
    bool backward();
    bool relieveDockingPressure();
    bool undocking(double duration);
    bool startDetection();
    bool stopDetection();

private:
    using NavClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
    using NavGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using NavCancelResponse = NavClient::CancelResponse;

    enum class NavCancelState {
        IDLE,
        WAITING_FOR_GOAL_RESPONSE,
        WAITING_FOR_CANCEL_RESPONSE,
        WAITING_FOR_TERMINAL_RESULT,
        CONFIRMED,
        FAILED
    };

    void requestNavCancellation(const std::shared_ptr<NavGoalHandle>& goal_handle);
    void cbNavigationBackend(const std_msgs::msg::String::SharedPtr msg);

    NavClient::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_backend_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_detection_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_detection_client_;
    bool is_charging_ = false;
    bool apriltag_detection_running_ = false;
    double battery_percentage_ = -1.0;
    std::string navigation_backend_name_;
    std::uint64_t navigation_backend_revision_ = 0;
    std::function<bool()> task_interruption_check_;

    // NavigateToCharging 异步导航相关状态。当前 Action server 在同一线程中通过
    // spin_some 派发 Nav2 回调并执行 tickRoot，因此这些字段按串行访问处理。
    bool nav_goal_sent_ = false;             // 是否已发送 Nav2 goal
    NavCancelState nav_cancel_state_ = NavCancelState::IDLE;
    std::shared_ptr<NavGoalHandle> nav_goal_handle_;
    std::shared_ptr<NavClient::SendGoalOptions> nav_send_options_;
    std::shared_future<NavCancelResponse::SharedPtr> nav_cancel_future_;
    std::uint64_t nav_goal_generation_ = 0;  // 隔离晚到的旧目标回调，避免污染下一次回充。
    std::string nav_result_status_ = kNavStatusRunning;
    bool nav_result_ready_ = false;          // result 回调是否已置位
};

struct ChargingPointAsset {
    geometry_msgs::msg::PoseStamped pose;
    std::string name;
    std::string assets_file;
    std::string map_folder;
    std::string map_name;
    std::uint64_t map_epoch = 0;
};

// 读取当前地图中唯一的 CHARGING 点；调用方只应在 nav2 后端下调用。
ChargingPointAsset readChargingPointFromCurrentMap(
    const std::shared_ptr<AutoChargingNode>& charging_node, double wait_timeout_sec);

// ==================== 行为树节点类 ====================
class NavigateToCharging : public BT::StatefulActionNode {
public:
    NavigateToCharging(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<AutoChargingNode> charging_node_;
};

// class AprilTagAndDock : public BT::StatefulActionNode
// {
// public:
//     AprilTagAndDock(const std::string& name, const BT::NodeConfiguration& config);
//     static BT::PortsList providedPorts();
//     BT::NodeStatus onStart() override;
//     BT::NodeStatus onRunning() override;
//     void onHalted() override;

// private:
//     FILE* apriltag_pipe_ = nullptr;
//     bool process_started_ = false;
// };

class Dock : public BT::StatefulActionNode {
public:
    Dock(const std::string& name, const BT::NodeConfiguration& config);
    ~Dock() override;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<AutoChargingNode> charging_node_;
    std::unique_ptr<ReverseDockingBtController> reverse_docking_;
    rclcpp::Time start_time_;
    double timeout_ = 120.0;
    double qr_detection_timeout_ = 5.0;
};

class CheckCharging : public BT::StatefulActionNode {
public:
    CheckCharging(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Time start_time_;
    double timeout_ = 5.0;
    int consecutive_charging_ = 0;
    std::shared_ptr<AutoChargingNode> charging_node_;
};

class CheckBatteryAboveThreshold : public BT::ConditionNode {
public:
    CheckBatteryAboveThreshold(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class IsCharging : public BT::ConditionNode {
public:
    IsCharging(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class IsChargingNavigationEnabled : public BT::ConditionNode {
public:
    IsChargingNavigationEnabled(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class Undocking : public BT::SyncActionNode {
public:
    Undocking(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class CounterCheck : public BT::SyncActionNode {
public:
    CounterCheck(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    int max_retries_ = 5;
};

class IncrementCounter : public BT::SyncActionNode {
public:
    IncrementCounter(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class AttemptLogger : public BT::SyncActionNode {
public:
    AttemptLogger(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class StartDetection : public BT::SyncActionNode {
public:
    StartDetection(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class StopDetection : public BT::SyncActionNode {
public:
    StopDetection(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

#endif  // CHARGING_BT_NODES_HPP_

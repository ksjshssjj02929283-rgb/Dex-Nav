#ifndef AUTO_CHARGING_BT_NODES_HPP_
#define AUTO_CHARGING_BT_NODES_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
#include <vector>

// 前向声明
class AutoChargingNode;
class ReverseDockingBtController;

// 全局节点指针

// ==================== ROS节点类 ====================
class AutoChargingNode : public rclcpp::Node {
public:
    enum class ChargerState { UNKNOWN = 0, CHARGING = 1, CONNECTED_NOT_CHARGING = 2, DISCONNECTED = 3 };

    AutoChargingNode();

    void cbDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    bool isCharging() const { return is_charging_; }
    bool isConnectedToCharger() const { return is_connected_to_charger_; }
    double getBatteryPercentage() const { return battery_percentage_; }
    ChargerState confirmEntryChargerState();
    const char* chargerStateName(ChargerState state) const;

    bool navToPose(const geometry_msgs::msg::PoseStamped& goal_pose);
    // 以下三个接口为 NavigateToCharging 改造为 StatefulActionNode 提供非阻塞支持，
    // 与 navigation_client.py 的 Action 状态约定保持一致：
    //   sendNavGoal 异步发送 Nav2 goal，立即返回；
    //   isNavDone 非阻塞轮询，返回 RUNNING/SUCCEEDED/FAILED/CANCELLED；
    //   cancelNav 在 onHalted 时取消 Nav2 goal，保证回充取消能立即传到 Nav2。
    bool sendNavGoal(const geometry_msgs::msg::PoseStamped& goal_pose);
    std::string isNavDone();
    void cancelNav();
    void publishTwist(double linear_x, double angular_z);
    int runSubprocess(const std::vector<std::string>& cmd);
    void backward();
    void relieveDockingPressure();
    void undocking(double duration);
    bool startDetection();
    bool stopDetection();

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_detection_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_detection_client_;
    bool is_charging_ = false;
    bool is_connected_to_charger_ = false;
    bool apriltag_detection_running_ = false;
    double battery_percentage_ = -1.0;

    // NavigateToCharging 异步导航相关状态。Action server 主循环是单线程 tickRoot，
    // 这些字段只在主循环线程被 sendNavGoal/isNavDone/cancelNav 访问，无需加锁。
    bool nav_goal_sent_ = false;             // 是否已发送 Nav2 goal
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> nav_goal_handle_;
    std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions> nav_send_options_;
    std::string nav_result_status_ = "RUNNING";  // RUNNING/SUCCEEDED/FAILED/CANCELLED
    bool nav_result_ready_ = false;          // result 回调是否已置位
};

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

class IsConnectedToCharger : public BT::ConditionNode {
public:
    IsConnectedToCharger(const std::string& name, const BT::NodeConfiguration& config);
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

// 工具函数
geometry_msgs::msg::PoseStamped readChargingPose(const std::string& json_path);

#endif  // CHARGING_BT_NODES_HPP_

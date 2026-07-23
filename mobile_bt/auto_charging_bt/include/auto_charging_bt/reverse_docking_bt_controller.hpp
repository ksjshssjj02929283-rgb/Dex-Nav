#ifndef AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_
#define AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "reverse_docking_controller.h"

class AutoChargingNode;

/**
 * @brief 行为树 Dock 节点使用的分阶段倒车对接控制器。
 *
 * 控制器负责“停车采样—锁定目标—倒车靠近”三轮流程。采样得到的 base_link
 * 相对目标会锁定到 fixed_frame，机器人运动期间再转换回 base_link，保证误差随
 * 机器人位置实时变化。
 */
class ReverseDockingBtController {
public:
    explicit ReverseDockingBtController(std::shared_ptr<AutoChargingNode> charging_node);
    ~ReverseDockingBtController();

    /// 加载参数并启动目标订阅、TF 和控制定时器。
    void start();
    /// 停止控制、释放运行期资源并发布零速度。
    void stop();
    /// 最终对接阶段完成后返回 true。
    bool done() const { return controller_done_; }
    /// 返回稳定的失败分类码，便于日志检索和现场统计。
    std::string failureReasonCode() const;
    /// 根据本次对接统计生成面向现场排查的中文失败原因。
    std::string failureReasonSummary() const;
    /// 生成超时日志使用的单行状态摘要。
    std::string diagnosticSummary() const;

private:
    /// 外层对接阶段。三个采样阶段与三个运动阶段严格交替执行。
    enum class Stage {
        INITIAL_SAMPLING,
        FIRST_APPROACH,
        SECOND_SAMPLING,
        SECOND_APPROACH,
        FINAL_SAMPLING,
        FINAL_APPROACH,
        FINISHED,
    };

    /// Dock 超时时的统一失败分类，不改变行为树成功/失败语义。
    enum class FailureReason {
        NO_TARGET,
        FRAME_MISMATCH,
        INVALID_SAMPLES,
        INSUFFICIENT_SAMPLES,
        FINAL_SAMPLE_UNSTABLE,
        TF_LOCK_FAILED,
        TF_TRACKING_FAILED,
        CONTROL_TIMEOUT,
    };

    /// 每个运动阶段独立使用的误差阈值、速度比例和速度上限。
    struct ApproachConfig {
        double linear_tolerance = 0.020;
        double lateral_tolerance = 0.020;
        double final_yaw_tolerance = 0.020;
        double linear_scale = 1.0;
        double angular_scale = 1.0;
        double max_linear_speed = 0.0;
        double max_angular_speed = 0.0;
        bool use_stage_limit = false;
    };

    /// 目标在 base_link 下的绝对误差，用于判断最终采样结果是否明显变差。
    struct TargetError {
        double linear = 0.0;
        double lateral = 0.0;
        double final_yaw = 0.0;
    };

    // 主流程：接收目标、周期控制、处理采样和阶段切换。
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();
    void handleSamplingStage();
    void handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base);
    // 根据当前阶段选择采样时长、距离限制和最低有效样本数。
    bool isSamplingStage() const;
    double currentSamplingDuration() const;
    double currentSamplingMaxDistance() const;
    int currentMinValidSampleCount() const;
    void startSampling(double duration);
    // 输入样本检查。过期、时间戳超前或距离超限的样本不会参与平均。
    bool isSampleFresh(const geometry_msgs::msg::PoseStamped& msg);
    bool isSampleDistanceValid(const geometry_msgs::msg::Pose& pose) const;
    bool isLockedTargetReasonable(const geometry_msgs::msg::Pose& target_pose_base) const;
    // 为底层控制器应用当前阶段参数，并限制最终速度指令。
    void configureControllerForStage();
    std::pair<double, double> limitStageVelocity(double linear_x, double angular_z) const;
    const ApproachConfig& currentApproachConfig() const;
    double clamp(double value, double min_value, double max_value) const;
    // 阶段名称和说明仅用于紧凑日志，不参与状态判断。
    const char* stageName() const;
    const char* stageName(Stage stage) const;
    const char* stageDescription() const;
    const char* stageDescription(Stage stage) const;
    const char* controllerStageDescription() const;
    FailureReason classifyFailureReason() const;
    const char* failureReasonCode(FailureReason reason) const;
    void logStageBanner(Stage stage) const;
    // 将相对目标锁定到固定坐标系，并在控制周期中转换回机器人坐标系。
    std::optional<geometry_msgs::msg::PoseStamped> lockTargetInFixedFrame(
        const geometry_msgs::msg::Pose& target_pose_base);
    std::optional<geometry_msgs::msg::Pose> getLockedTargetInBaseFrame();
    // 对样本进行裁剪平均，减少少量跳变数据对锁定目标的影响。
    geometry_msgs::msg::Pose averageTargetSamples() const;
    double trimmedMean(std::vector<double> values) const;
    double averageSampleYaw() const;
    // 最终采样明显差于第二阶段到位误差时，触发有限次数的重新采样。
    TargetError calculateTargetError(const geometry_msgs::msg::Pose& target_pose_base) const;
    bool isFinalSampleConsistentWithSecondReach(const geometry_msgs::msg::Pose& target_pose_base) const;
    // 速度、TF 和诊断辅助函数。
    void publishZero();
    void publishStaticTargetTf();
    void publishBaseTargetTf(const geometry_msgs::msg::Pose& target_pose_base);
    void logTargetError(const geometry_msgs::msg::Pose& target_pose_base, const std::string& prefix) const;
    double normalizePi(double value) const;
    // 清理运行状态并从参数服务器读取配置。
    void resetState();
    void configureFromParameters();

    // 共享节点与运行期 ROS 资源。
    std::shared_ptr<AutoChargingNode> charging_node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ReverseDockingController controller_;

    // 对接使用的话题和坐标系配置。
    std::string base_frame_;
    std::string target_base_frame_;
    std::string static_target_frame_;
    std::string target_topic_;
    std::string fixed_frame_;

    // 各停车采样阶段的时长、有效范围和最低样本数。
    double initial_sampling_duration_ = 1.0;
    double second_sampling_duration_ = 2.0;
    double final_sampling_duration_ = 1.0;
    double final_resampling_duration_ = 2.0;
    double max_sample_age_ = 1.5;
    double initial_sampling_max_distance_ = 0.0;
    double second_sampling_max_distance_ = 0.20;
    double final_sampling_max_distance_ = 0.08;
    int min_valid_sample_count_ = 5;
    int initial_min_valid_sample_count_ = 5;
    int second_min_valid_sample_count_ = 10;
    int final_min_valid_sample_count_ = 5;
    int final_resample_min_valid_sample_count_ = 10;

    // 裁剪平均和最终重采样策略，防止近距离偶发跳变直接进入最终靠近。
    double sample_trim_ratio_ = 0.20;
    int final_resample_max_attempts_ = 3;
    double final_resample_linear_margin_ = 0.020;
    double final_resample_lateral_margin_ = 0.015;
    double final_resample_yaw_margin_ = 0.035;

    // 三个运动阶段分别使用的控制约束。
    ApproachConfig first_stage_config_;
    ApproachConfig second_stage_config_;
    ApproachConfig final_stage_config_;
    double final_yaw_offset_ = 0.0;

    // 当前任务状态。stage_ 是外层流程状态，controller_done_ 是 Dock 完成标志。
    Stage stage_ = Stage::INITIAL_SAMPLING;
    bool sampling_started_ = false;
    bool controller_done_ = false;
    int final_sampling_retry_count_ = 0;
    // 第二阶段尚未实际运动却已满足阈值时，只允许额外复采样一次，避免反复切换。
    bool second_immediate_recheck_done_ = false;
    // 标记第二阶段是否已经输出过运动速度，用于区分“原地满足阈值”和“运动后到位”。
    bool second_stage_commanded_ = false;

    // 当前采样窗口状态，每次进入新的采样阶段都会重新计时和清零计数。
    rclcpp::Time sample_start_time_;
    std::optional<rclcpp::Time> last_valid_target_time_;
    uint64_t sampling_received_count_ = 0;
    std::vector<geometry_msgs::msg::Pose> target_samples_;

    // 本次 Dock 尝试的累计诊断计数，用于在总超时时给出明确失败分类。
    uint64_t target_message_count_ = 0;
    uint64_t valid_sample_count_ = 0;
    uint64_t frame_rejected_count_ = 0;
    uint64_t stale_sample_count_ = 0;
    uint64_t future_sample_count_ = 0;
    uint64_t distance_rejected_count_ = 0;
    uint64_t lock_tf_failure_count_ = 0;
    uint64_t tracking_tf_failure_count_ = 0;
    uint64_t final_sample_inconsistent_count_ = 0;

    // 锁定目标保存在 fixed_frame；第二阶段到位误差用于校验最终采样质量。
    std::optional<geometry_msgs::msg::PoseStamped> locked_target_fixed_;
    std::optional<TargetError> second_reached_error_;
};

#endif  // AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_

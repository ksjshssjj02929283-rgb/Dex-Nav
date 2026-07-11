#ifndef REVERSE_DOCKING_CONTROLLER_H
#define REVERSE_DOCKING_CONTROLLER_H

#include <chrono>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>

class ReverseDockingController {
public:
    enum class Stage {
        FINISH,
        APPROACHING,
        ADJUST_HEADING,
        ADJUST_DISTANCE,
        ROTATING_TO_GOAL,
    };

    ReverseDockingController();

    void reset();

    void setLinearGain(double gain);
    void setHeadingGain(double gain);
    void setFinalYawGain(double gain);
    void setFinalYawIntegralGain(double gain);
    void setFinalYawDerivativeGain(double gain);
    void setFinalYawIntegralLimit(double limit);
    void setFinalYawOffset(double offset);
    void setMaxLinearSpeed(double speed);
    void setMinLinearSpeed(double speed);
    void setMaxAngularSpeed(double speed);
    void setMinAngularSpeed(double speed);
    void setFinalMaxAngularSpeed(double speed);
    void setFinalMinAngularSpeed(double speed);
    void setMaxLinearAcceleration(double accel);
    void setMaxAngularAcceleration(double accel);
    void setLinearTolerance(double tolerance);
    void setLateralTolerance(double tolerance);
    void setHeadingTolerance(double tolerance);
    void setFinalYawTolerance(double tolerance);
    void setAlignHeadingThreshold(double tolerance);
    void setApproachDistance(double distance);
    void setLargeHeadingThreshold(double tolerance);

    std::pair<double, double> getVelocity(const geometry_msgs::msg::Pose& current_pose,
                                          const geometry_msgs::msg::Pose& goal_pose);
    bool atGoal(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose) const;

    Stage stage() const { return stage_; }
    std::string stageName() const;

private:
    struct PoseError {
        double local_x;
        double local_y;
        double distance;
        double reverse_heading_error;
        double final_yaw_error;
    };

    PoseError computeError(const geometry_msgs::msg::Pose& current_pose,
                           const geometry_msgs::msg::Pose& goal_pose) const;
    std::pair<double, double> limitVelocity(double linear_x, double angular_z, bool final_rotation);
    double computeFinalYawVelocity(double final_yaw_error);
    void resetFinalYawPid();
    double applyMinSpeed(double value, double min_value) const;
    double getPoseYaw(const geometry_msgs::msg::Pose& pose) const;
    double normalizePi(double value) const;

    double linear_gain_;
    double heading_gain_;
    double final_yaw_gain_;
    double final_yaw_integral_gain_;
    double final_yaw_derivative_gain_;
    double final_yaw_integral_limit_;
    double final_yaw_offset_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double min_angular_speed_;
    double final_max_angular_speed_;
    double final_min_angular_speed_;
    double max_linear_accel_;
    double max_angular_accel_;
    double linear_tolerance_;
    double lateral_tolerance_;
    double heading_tolerance_;
    double final_yaw_tolerance_;
    double approach_distance_;
    double large_heading_threshold_;

    Stage stage_;
    bool final_yaw_pid_active_;
    double final_yaw_integral_;
    double prev_final_yaw_error_;
    std::chrono::steady_clock::time_point final_yaw_pid_time_;
    bool accel_first_call_;
    double prev_linear_x_;
    double prev_angular_z_;
    std::chrono::steady_clock::time_point last_time_;
};

#endif  // REVERSE_DOCKING_CONTROLLER_H

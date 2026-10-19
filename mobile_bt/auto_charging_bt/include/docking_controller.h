#ifndef DOCKING_CONTROLLER_H
#define DOCKING_CONTROLLER_H

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

class DockingController {
public:
    DockingController();

    void setConstants(double kP, double kA, double kB);
    void setMaxLinearSpeed(double speed);
    void setMinLinearSpeed(double speed);
    void setMaxAngularSpeed(double speed);
    void setMinAngularSpeed(double speed);
    void setLinearTolerance(double tolerance);
    void setAngularTolerance(double tolerance);

    std::pair<double, double> getVelocity(const geometry_msgs::msg::Pose& current_pose,
                                          const geometry_msgs::msg::Pose& goal_pose,
                                          int direction = -1);
    bool atGoal(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose);

private:
    double kA_, kP_, kB_;
    double max_linear_speed_, min_linear_speed_;
    double max_angular_speed_, min_angular_speed_;
    double linear_tolerance_, angular_tolerance_;

    double getGoalDistance(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose);
    double getPoseYaw(const geometry_msgs::msg::Pose& pose);
    double normalizePi(double alpha);
    double normalizeHalfPi(double alpha);
    int sign(double x);
    tf2::Quaternion getRotationError(const geometry_msgs::msg::Pose& target_pose,
                                     const geometry_msgs::msg::Pose& origin_pose);
};

#endif  // DOCKING_CONTROLLER_H

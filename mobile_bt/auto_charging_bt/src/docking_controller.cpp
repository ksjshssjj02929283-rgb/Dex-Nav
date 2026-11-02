#include "docking_controller.h"

DockingController::DockingController() {
    kA_ = 0.1050 * 1.05;
    kP_ = 0.0800 * 1.05;
    kB_ = -0.0040 * 1.05;
    max_linear_speed_ = 0.50;
    min_linear_speed_ = 0.0;
    max_angular_speed_ = 0.500;
    min_angular_speed_ = 0.0;
    linear_tolerance_ = 0.03;
    angular_tolerance_ = 0.05;
}

void DockingController::setConstants(double kP, double kA, double kB) {
    kP_ = kP;
    kA_ = kA;
    kB_ = kB;
}

void DockingController::setMaxLinearSpeed(double speed) { max_linear_speed_ = speed; }

void DockingController::setMinLinearSpeed(double speed) { min_linear_speed_ = speed; }

void DockingController::setMaxAngularSpeed(double speed) { max_angular_speed_ = speed; }

void DockingController::setMinAngularSpeed(double speed) { min_angular_speed_ = speed; }

void DockingController::setLinearTolerance(double tolerance) { linear_tolerance_ = tolerance; }

void DockingController::setAngularTolerance(double tolerance) { angular_tolerance_ = tolerance; }

double DockingController::getGoalDistance(const geometry_msgs::msg::Pose& current_pose,
                                          const geometry_msgs::msg::Pose& goal_pose) {
    double diffX = current_pose.position.x - goal_pose.position.x;
    double diffY = current_pose.position.y - goal_pose.position.y;
    return std::sqrt(diffX * diffX + diffY * diffY);
}

double DockingController::getPoseYaw(const geometry_msgs::msg::Pose& pose) {
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

tf2::Quaternion DockingController::getRotationError(const geometry_msgs::msg::Pose& target_pose,
                                                    const geometry_msgs::msg::Pose& origin_pose) {
    tf2::Quaternion q_target, q_origin;
    tf2::fromMsg(target_pose.orientation, q_target);
    tf2::fromMsg(origin_pose.orientation, q_origin);
    return q_target * q_origin.inverse();
}

bool DockingController::atGoal(const geometry_msgs::msg::Pose& current_pose,
                               const geometry_msgs::msg::Pose& goal_pose) {
    double d = getGoalDistance(current_pose, goal_pose);
    tf2::Quaternion q_error = getRotationError(goal_pose, current_pose);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_error).getRPY(roll, pitch, yaw);
    double dy = std::abs(current_pose.position.y - goal_pose.position.y);
    double dTh = std::abs(normalizePi(yaw));
    bool res = (d < linear_tolerance_ && dTh < angular_tolerance_);
    return res;
}

std::pair<double, double> DockingController::getVelocity(const geometry_msgs::msg::Pose& current_pose,
                                                         const geometry_msgs::msg::Pose& goal_pose,
                                                         int direction) {
    double goal_heading =
        atan2(current_pose.position.y - goal_pose.position.y, current_pose.position.x - goal_pose.position.x);
    double normalized_goal_heading = normalizePi(goal_heading);

    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::Quaternion q_goal_heading;
    q_goal_heading.setRPY(0, 0, normalized_goal_heading);

    tf2::Quaternion q_target1 = q_goal_heading * q_current;
    tf2::Quaternion q_del_yaw = q_target1 * q_current.inverse();
    double roll, pitch, del_yaw;
    tf2::Matrix3x3(q_del_yaw).getRPY(roll, pitch, del_yaw);

    double a = normalizePi(del_yaw);

    tf2::Quaternion q_error = getRotationError(goal_pose, current_pose);
    tf2::Matrix3x3(q_error).getRPY(roll, pitch, del_yaw);
    double theta = normalizePi(del_yaw);
    double b = normalizePi(-theta - a);

    double d = getGoalDistance(current_pose, goal_pose);

    if (direction != 1) {
        a = normalizeHalfPi(a);
        b = normalizeHalfPi(b);
    }

    double xVel, thetaVel;
    // 使用滞后区间避免模式切换振荡：<0.019m进入纯旋转，>0.021m退出纯旋转
    static bool in_rotation_mode = false;
    const double enter_rotation_threshold = 0.020;  // 进入纯旋转阈值
    const double exit_rotation_threshold = 0.10;    // 退出纯旋转阈值

    if (std::abs(d) < enter_rotation_threshold) {
        in_rotation_mode = true;
    } else if (std::abs(d) > exit_rotation_threshold) {
        in_rotation_mode = false;
    }

    if (in_rotation_mode) {
        // 到位后使用kA_进行纯旋转姿态调整，kA_=0.1103比kB_=-0.0042大26倍
        // 配合onStart中设置的min_angular_speed=0.005，确保有效的角速度输出
        // 注意：theta<0时需要正角速度修正，所以取负号
        xVel = 0.0;
        thetaVel = -kA_ * theta;
    } else {
        xVel = kP_ * d * direction;
        thetaVel = kA_ * a + kB_ * b;
        if (direction == 1) {
            thetaVel = -thetaVel;
        }
    }

    if (std::abs(xVel) > max_linear_speed_) {
        double ratio = max_linear_speed_ / std::abs(xVel);
        xVel *= ratio;
        thetaVel *= ratio;
    }

    if (std::abs(thetaVel) > max_angular_speed_) {
        double ratio = max_angular_speed_ / std::abs(thetaVel);
        xVel *= ratio;
        thetaVel *= ratio;
    }

    if (std::abs(xVel) > 0 && std::abs(xVel) < min_linear_speed_) {
        double ratio = min_linear_speed_ / std::abs(xVel);
        xVel *= ratio;
        thetaVel *= std::max(ratio * 0.75, 1.0);
    } else if (xVel == 0 && std::abs(thetaVel) > 1e-6 && std::abs(thetaVel) < min_angular_speed_) {
        double ratio = min_angular_speed_ / std::abs(thetaVel);
        xVel *= ratio;
        thetaVel *= ratio;
    }
    if (std::abs(thetaVel) < min_angular_speed_) {
        if (std::abs(thetaVel) > 0.002) {
            double ratio = min_angular_speed_ / abs(thetaVel);
            thetaVel *= ratio;
        }
    }

    return {xVel, thetaVel};
}

double DockingController::normalizePi(double alpha) {
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;
    return alpha;
}

double DockingController::normalizeHalfPi(double alpha) {
    alpha = normalizePi(alpha);
    if (alpha > M_PI / 2) return alpha - M_PI;
    if (alpha < -M_PI / 2) return alpha + M_PI;
    return alpha;
}

int DockingController::sign(double x) { return (x >= 0) ? 1 : -1; }

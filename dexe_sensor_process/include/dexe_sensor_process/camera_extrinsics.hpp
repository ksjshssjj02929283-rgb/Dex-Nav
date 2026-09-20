#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <stdexcept>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace dexe_sensor_process {

// 仅用于相对机身固定安装的相机：保存完整四元数旋转和平移，而非仅保存 yaw。
class CameraExtrinsics {
public:
    bool matches(const std::string& source_frame) const {
        return transform_ && transform_->child_frame_id == source_frame;
    }

    void set(const geometry_msgs::msg::TransformStamped& transform) {
        // 相机光轴为 (0, 0, 1)，仅在外参更新时计算它在机身 XY 平面的朝向。
        tf2::Quaternion rotation;
        tf2::fromMsg(transform.transform.rotation, rotation);
        const auto axis = tf2::Matrix3x3(rotation).getColumn(2);
        const double yaw = std::atan2(axis.y(), axis.x());
        transform_ = transform;
        cached_transform_.setOrigin(tf2::Vector3(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z));
        cached_transform_.setRotation(rotation);
        optical_axis_yaw_ = yaw;
    }

    // 与完整外参同步缓存，避免逐帧构造旋转矩阵和计算 atan2。
    // 未初始化时仍抛出 bad_optional_access，保持原有行为。
    double opticalAxisYaw() const {
        return optical_axis_yaw_.value();
    }

    void transformCloud(const sensor_msgs::msg::PointCloud2& input,
                        sensor_msgs::msg::PointCloud2& output) const {
        if (!matches(input.header.frame_id)) {
            throw std::invalid_argument("相机外参缓存与点云坐标系不匹配");
        }
        // doTransform 计算 P_base = R * P_camera + t；静态 TF 时间不能替代采样时间。
        tf2::doTransform(input, output, transform_.value());
        output.header.stamp = input.header.stamp;
    }

    tf2::Vector3 transformPoint(float x, float y, float z) const {
        if (!transform_) {
            throw std::bad_optional_access();
        }
        return cached_transform_ * tf2::Vector3(x, y, z);
    }

private:
    std::optional<geometry_msgs::msg::TransformStamped> transform_;
    tf2::Transform cached_transform_;
    std::optional<double> optical_axis_yaw_;
};

}  // namespace dexe_sensor_process

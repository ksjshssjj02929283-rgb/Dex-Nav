#pragma once

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dexe_sensor_process {
class CameraPipeline;

// Each camera owns an extractor, including all temporal algorithm state.
class ObstacleExtractor {
public:
    virtual ~ObstacleExtractor() = default;
    virtual bool requiresProjectedPoints() const = 0;
    virtual sensor_msgs::msg::PointCloud2::SharedPtr extract(
        CameraPipeline& camera,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
        const sensor_msgs::msg::PointCloud2::SharedPtr& raw) = 0;
};

class HeightFilterExtractor final : public ObstacleExtractor {
public:
    bool requiresProjectedPoints() const override { return true; }
    sensor_msgs::msg::PointCloud2::SharedPtr extract(
        CameraPipeline& camera,
        const sensor_msgs::msg::Image::ConstSharedPtr&,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr&,
        const sensor_msgs::msg::PointCloud2::SharedPtr& input_msg) override;
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered_{new pcl::PointCloud<pcl::PointXYZ>};
};

class UVDisparityExtractor final : public ObstacleExtractor {
public:
    explicit UVDisparityExtractor(rclcpp::Node& node);
    bool requiresProjectedPoints() const override { return false; }
    sensor_msgs::msg::PointCloud2::SharedPtr extract(
        CameraPipeline& camera,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
        const sensor_msgs::msg::PointCloud2::SharedPtr&) override;
private:
    double uv_disparity_min_z_ = 0.15;
    double uv_disparity_max_z_ = 4.0;
    double uv_disparity_dz_ = 0.02;
    int uv_disparity_v_margin_ = 15;
    int uv_disparity_min_u_count_ = 5;
    double uv_disparity_ground_margin_h_ = 0.025;

    // 上一帧成功拟合的地面参数（v = a + b/z）
    double last_fit_a_ = 0.0;
    double last_fit_b_ = 0.0;
    bool has_last_fit_ = false;

    // RANSAC 随机数生成器（持久化成员变量，固定种子，避免每帧随机重新初始化导致拟合结果逐帧抖动）
    std::mt19937 rng_{42};

    // UV-Disparity 跨帧复用平坦缓冲（替代嵌套 vector，改善 cache 局部性，消除每帧堆分配）
    struct GroundPoint  { float z, v; };
    struct UVTempPoint  { uint16_t u, v; uint16_t raw_z; int z_idx; }; // 轻量化，只存储像素坐标与原始值
    std::vector<int>         vdepth_buf_;        // height × N_z 平坦 V-Depth 直方图
    std::vector<int>         udepth_buf_;         // width  × N_z 平坦 U-Depth 直方图
    std::vector<int>         udepth_prefix_buf_;  // (width+1) × (N_z+1) 积分图
    std::vector<GroundPoint> sample_buf_;         // 地面候选点（RANSAC 输入）
    std::vector<UVTempPoint> candidates_buf_;     // 障碍物候选点

    // 查找表（LUT）缓存，避免每像素进行浮点除法、倒数和 clip 运算
    std::vector<float>       lut_expected_v_;
    std::vector<float>       lut_dynamic_margin_;
    std::vector<int>         lut_z_idx_;
    std::vector<float>       lut_u_coef_;         // 预计算 (u - cx) / fx 比例表
    std::vector<float>       lut_v_coef_;         // 预计算 (v - cy) / fy 比例表
    float                    last_lut_a_ = -999.0f;
    float                    last_lut_b_ = -999.0f;

    // 原生无 PCL 依赖极速体素去重哈希集
    std::unordered_set<uint64_t> voxel_grid_set_;
    std::vector<float> buf_obstacles_;
};

std::unique_ptr<ObstacleExtractor> makeObstacleExtractor(
    const std::string& method, rclcpp::Node& node);
}  // namespace dexe_sensor_process

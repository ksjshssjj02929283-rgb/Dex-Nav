#include "dexe_sensor_process/camera_pipeline.hpp"
#include <set>

class RgbdToPointCloud : public rclcpp::Node {
public:
    explicit RgbdToPointCloud(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rgbd_to_pointcloud", options) {
        // 基础参数
        this->declare_parameter<std::string>("camera_name", "rear");
        this->declare_parameter<std::string>("input_topic", "");
        this->declare_parameter<std::string>("output_topic_raw", "");
        this->declare_parameter<std::string>("output_topic_filtered", "");
        this->declare_parameter<std::string>("output_topic_laserscan", "");
        this->declare_parameter<double>("depth_max_range", 3.0);

        // 障碍物提取方式与双视差参数
        this->declare_parameter<std::string>("obstacle_extraction_method", "auto");
        this->declare_parameter<std::string>("hardware_path", "/etc/dexe_robot/hardware.yaml");
        this->declare_parameter<double>("uv_disparity_min_z", 0.15);
        this->declare_parameter<double>("uv_disparity_max_z", 4.0);
        this->declare_parameter<double>("uv_disparity_dz", 0.02);
        this->declare_parameter<int>("uv_disparity_v_margin", 15);
        this->declare_parameter<int>("uv_disparity_min_u_count", 5);
        this->declare_parameter<double>("uv_disparity_ground_margin_h", 0.025);

        // 预处理参数
        this->declare_parameter<bool>("flag_voxel_filter", true);
        this->declare_parameter<bool>("flag_mls_filter", false);
        this->declare_parameter<bool>("flag_sor_filter", false);
        this->declare_parameter<double>("ds_leaf_size", 0.05);
        this->declare_parameter<int>("mls_polynomial_order", 2);
        this->declare_parameter<double>("mls_search_radius", 0.2);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        // 采样步长与控制参数
        this->declare_parameter<bool>("enable_laserscan", true);
        this->declare_parameter<bool>("division_mode", true);
        this->declare_parameter<int>("step_u", 2);
        this->declare_parameter<int>("step_v", 3);

        // 地面过滤在 ground_filter_frame 坐标系下按高度做判断，默认 base_link 的 z 轴为离地高度方向。
        // min_height 以下认为接近地面并剔除；max_height 以上认为过高，不作为近地障碍输入。
        this->declare_parameter<std::string>("ground_filter_frame", "base_link");
        this->declare_parameter<double>("ground_filter_min_height", 0.05);
        this->declare_parameter<double>("ground_filter_max_height", 1.5);


        // Empty cameras preserves the legacy single-camera parameters.
        const auto cameras = declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>{});
        const auto selection = dexe_sensor_process::selectObstacleMethod(
            get_parameter("obstacle_extraction_method").as_string(),
            get_parameter("hardware_path").as_string());
        if (!selection.error.empty()) {
            RCLCPP_WARN(get_logger(), "读取硬件版本失败，回退到高度法: %s", selection.error.c_str());
        }
        RCLCPP_INFO(get_logger(), "障碍物提取方法: %s，硬件版本: %s",
                    selection.method.c_str(), selection.revision.c_str());
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(10.0));
        // Same ROS node, dedicated TF callback thread so timed lookups can receive transforms.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        if (cameras.empty()) {
            cameras_.push_back(std::make_unique<dexe_sensor_process::CameraPipeline>(
                *this, get_parameter("camera_name").as_string(), "", selection.method, tf_buffer_));
        } else {
            std::set<std::string> names;
            for (const auto& name : cameras) {
                if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos ||
                    !names.insert(name).second) {
                    throw std::invalid_argument("cameras must contain unique non-empty camera identifiers");
                }
                const auto prefix = name + ".";
                for (const auto* parameter : {"input_topic", "output_topic_raw", "output_topic_filtered", "output_topic_laserscan"}) {
                    declare_parameter<std::string>(prefix + parameter, "");
                }
                cameras_.push_back(std::make_unique<dexe_sensor_process::CameraPipeline>(
                    *this, name, prefix, selection.method, tf_buffer_));
            }
        }
    }

    size_t cameraCount() const noexcept {
        return cameras_.size();
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Each pipeline owns an independent mutually-exclusive callback group.
    std::vector<std::unique_ptr<dexe_sensor_process::CameraPipeline>> cameras_;
};

#ifndef DEXE_SENSOR_PROCESS_NO_MAIN
int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdToPointCloud>();
    const auto thread_count = node->cameraCount();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), thread_count);
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "相机处理线程数: %zu", thread_count);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#endif

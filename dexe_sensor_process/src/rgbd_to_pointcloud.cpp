#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "rgbd_processing_state.hpp"
#include "hardware_obstacle_method.hpp"
#include "camera_extrinsics.hpp"

class RgbdToPointCloud : public rclcpp::Node {
public:
    RgbdToPointCloud() : Node("rgbd_to_pointcloud") {
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


        camera_name_ = this->get_parameter("camera_name").as_string();
        std::string camera_name = camera_name_;
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic_raw = this->get_parameter("output_topic_raw").as_string();
        std::string output_topic_filtered = this->get_parameter("output_topic_filtered").as_string();
        std::string output_topic_laserscan = this->get_parameter("output_topic_laserscan").as_string();
        depth_max_range_ = this->get_parameter("depth_max_range").as_double();
        if (!std::isfinite(depth_max_range_) || depth_max_range_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "depth_max_range 非法: %.3f m，回退为 3.0 m", depth_max_range_);
            depth_max_range_ = 3.0;
        }

        // 如果未指定话题，使用默认命名规则
        if (input_topic.empty()) {
            input_topic = "/orbbec/" + camera_name;
        }
        if (output_topic_raw.empty()) {
            output_topic_raw = input_topic + "/pointcloud2_raw";
        }
        if (output_topic_filtered.empty()) {
            output_topic_filtered = input_topic + "/pointcloud2_filtered";
        }
        if (output_topic_laserscan.empty()) {
            output_topic_laserscan = input_topic + "/laser_scan";
        }

        // 获取预处理参数
        flag_voxel_filter_ = this->get_parameter("flag_voxel_filter").as_bool();
        flag_mls_filter_ = this->get_parameter("flag_mls_filter").as_bool();
        flag_sor_filter_ = this->get_parameter("flag_sor_filter").as_bool();
        ds_leaf_size_ = this->get_parameter("ds_leaf_size").as_double();
        mls_polynomial_order_ = this->get_parameter("mls_polynomial_order").as_int();
        mls_search_radius_ = this->get_parameter("mls_search_radius").as_double();
        sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
        sor_stddev_mul_thresh_ = this->get_parameter("sor_stddev_mul_thresh").as_double();
        enable_laserscan_ = this->get_parameter("enable_laserscan").as_bool();
        division_mode_ = this->get_parameter("division_mode").as_bool();
        frame_division_state_.setEnabled(division_mode_);
        step_u_ = this->get_parameter("step_u").as_int();
        step_v_ = this->get_parameter("step_v").as_int();
        ground_filter_frame_ = this->get_parameter("ground_filter_frame").as_string();
        ground_filter_min_height_ = this->get_parameter("ground_filter_min_height").as_double();
        ground_filter_max_height_ = this->get_parameter("ground_filter_max_height").as_double();

        // auto 在启动时依据整机硬件选择算法；显式指定算法仍可用于回放和调试。
        const std::string hardware_path = this->get_parameter("hardware_path").as_string();
        const auto selection = dexe_sensor_process::selectObstacleMethod(
            this->get_parameter("obstacle_extraction_method").as_string(), hardware_path);
        obstacle_extraction_method_ = selection.method;
        if (!selection.error.empty()) {
            RCLCPP_WARN(get_logger(), "读取硬件版本失败 (%s)，回退到高度法: %s",
                        hardware_path.c_str(), selection.error.c_str());
        } else if (!selection.revision.empty()) {
            RCLCPP_INFO(get_logger(), "硬件版本 %s，自动选择障碍物提取方法: %s",
                        selection.revision.c_str(), obstacle_extraction_method_.c_str());
        }
        uv_disparity_min_z_ = this->get_parameter("uv_disparity_min_z").as_double();
        uv_disparity_max_z_ = this->get_parameter("uv_disparity_max_z").as_double();
        uv_disparity_dz_ = this->get_parameter("uv_disparity_dz").as_double();
        uv_disparity_v_margin_ = this->get_parameter("uv_disparity_v_margin").as_int();
        uv_disparity_min_u_count_ = this->get_parameter("uv_disparity_min_u_count").as_int();
        uv_disparity_ground_margin_h_ = this->get_parameter("uv_disparity_ground_margin_h").as_double();

        // 创建原生 ROS 订阅（不再使用时间同步器）
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic + "/depth", rclcpp::SensorDataQoS(),
            std::bind(&RgbdToPointCloud::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            input_topic + "/camera_info", rclcpp::QoS(10),
            std::bind(&RgbdToPointCloud::cameraInfoCallback, this, std::placeholders::_1));

        // 发布原始点云
        m_pointcloud_publisher_raw = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_raw, 10);

        // 发布预处理后的点云
        m_pointcloud_publisher_filtered =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_filtered, 10);

        // 发布压缩成 LaserScan 的点云
        m_laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_laserscan, 10);

        // TF buffer（Humble 需要 3 个参数）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(), "RGBD 点云转换节点启动，相机名称: %s", camera_name.c_str());
        RCLCPP_INFO(get_logger(), "输入模式: Depth + CameraInfo 重建");
        RCLCPP_INFO(get_logger(), "订阅深度图话题: %s/depth", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "订阅相机参数话题: %s/camera_info", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "发布原始点云话题: %s", output_topic_raw.c_str());
        RCLCPP_INFO(get_logger(), "发布过滤点云话题: %s", output_topic_filtered.c_str());
        RCLCPP_INFO(get_logger(), "点云最大深度: %.3f m", depth_max_range_);
        RCLCPP_INFO(get_logger(), "输入帧二分频模式: %s", division_mode_ ? "开启（处理一帧、跳过一帧）" : "关闭");
        RCLCPP_INFO(get_logger(), "LaserScan 话题: %s（enable_laserscan=true 时发布）", output_topic_laserscan.c_str());
        RCLCPP_INFO(get_logger(), "预处理参数 - 降采样: %s, MLS: %s, 统计滤波: %s",
                    flag_voxel_filter_ ? "启用" : "禁用", flag_mls_filter_ ? "启用" : "禁用",
                    flag_sor_filter_ ? "启用" : "禁用");
        RCLCPP_INFO(get_logger(), "地面过滤参数 - 判断坐标系: %s, 最低保留高度: %.3f m, 最高保留高度: %.3f m",
                    ground_filter_frame_.c_str(), ground_filter_min_height_, ground_filter_max_height_);
        RCLCPP_INFO(get_logger(),
                    "地面过滤流程: 点云会临时转换到 %s 按 z 高度过滤，最终发布仍保持 CameraInfo 的光学坐标系，便于 "
                    "Nav2 costmap 正确做 marking/clearing",
                    ground_filter_frame_.c_str());

        RCLCPP_INFO(get_logger(), "障碍物提取方法: %s", obstacle_extraction_method_.c_str());
        if (obstacle_extraction_method_ == "uv_disparity") {
            RCLCPP_INFO(get_logger(), "双视差参数 - 深度范围: [%.2f, %.2f] m, 步长: %.3f m, V边距: %d, U最小计数值: %d",
                        uv_disparity_min_z_, uv_disparity_max_z_, uv_disparity_dz_, uv_disparity_v_margin_,
                        uv_disparity_min_u_count_);
        }
    }

    ~RgbdToPointCloud() {
        tf_listener_.reset();
        tf_buffer_.reset();
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
        // 相机内参基本不变，这里只保存最新一帧的 CameraInfo 供深度回调使用
        last_camera_info_ = camera_info_msg;
        RCLCPP_INFO_ONCE(get_logger(), "已收到 CameraInfo，相机参数坐标系: %s, fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f",
                         camera_info_msg->header.frame_id.c_str(), camera_info_msg->k[0], camera_info_msg->k[4],
                         camera_info_msg->k[2], camera_info_msg->k[5]);
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        // 检查是否有任何节点在订阅本节点发布的话题，若没有任何订阅者则跳过处理，节省 CPU 资源
        size_t sub_count_raw = m_pointcloud_publisher_raw ? m_pointcloud_publisher_raw->get_subscription_count() : 0;
        size_t sub_count_filtered = m_pointcloud_publisher_filtered ? m_pointcloud_publisher_filtered->get_subscription_count() : 0;
        size_t sub_count_laserscan = (enable_laserscan_ && m_laserscan_publisher_) ? m_laserscan_publisher_->get_subscription_count() : 0;

        if (sub_count_raw == 0 && sub_count_filtered == 0 && sub_count_laserscan == 0) {
            return;
        }

        // division_mode 开启时严格处理一帧、跳过一帧，不把该行为描述为目标频率控制。
        if (!frame_division_state_.shouldProcess()) {
            return;
        }

        // 使用最近一次收到的 CameraInfo
        auto camera_info_msg = last_camera_info_;
        if (!camera_info_msg) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "尚未收到 CameraInfo，无法将深度图转换为点云");
            return;
        }

        int width = depth_msg->width;
        int height = depth_msg->height;

        // 基本有效性检查
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "收到的图像尺寸非法: %dx%d", width, height);
            return;
        }
        const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
        if (depth_msg->step == 0 || depth_msg->data.size() < expected_size) {
            RCLCPP_WARN(get_logger(), "深度图数据长度不足: %zu < %zu", depth_msg->data.size(), expected_size);
            return;
        }

        // 判断是否需要计算原始点云 msg_raw：
        // 1. 订阅者需要原始点云 (sub_count_raw > 0)
        // 2. 障碍物提取方法不是 "uv_disparity" (比如是 "height_filter"，其滤波函数需要 msg_raw 作为输入)
        bool need_raw = (sub_count_raw > 0) || (obstacle_extraction_method_ != "uv_disparity");

        sensor_msgs::msg::PointCloud2::SharedPtr msg_raw;
        if (need_raw) {
            if (!computePointClouds(depth_msg, camera_info_msg, msg_raw)) {
                RCLCPP_WARN(get_logger(), "深度图转换原始点云失败");
                return;
            }
        }

        // 根据配置的滤波方法计算过滤后的点云
        sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered;
        if (obstacle_extraction_method_ == "uv_disparity") {
            sensor_msgs::msg::PointCloud2::SharedPtr msg_obstacle_raw;
            if (computePointCloudsUVDisparity(depth_msg, camera_info_msg, msg_obstacle_raw)) {
                msg_filtered = msg_obstacle_raw;
            }
        } else {
            if (msg_raw) {
                msg_filtered = filterPointCloud(msg_raw);
            }
        }

        // 如果订阅者需要原始点云且计算成功，则发布
        if (sub_count_raw > 0 && msg_raw) {
            m_pointcloud_publisher_raw->publish(*msg_raw);
        }

        if (!msg_filtered) {
            size_t raw_points = msg_raw ? static_cast<size_t>(msg_raw->width) * msg_raw->height : 0;
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "本帧 filtered 点云未发布：原始点云有 %zu 点，但过滤算法未能成功提取障碍物，"
                                 "或地面过滤所需 TF 暂时不可用。", raw_points);
            return;
        }

        if (enable_laserscan_) {
            auto laser_scan_msg = pointCloudToLaserScanBaseLink(msg_filtered, tf_buffer_, get_logger());
            if (laser_scan_msg) {
                m_laserscan_publisher_->publish(*laser_scan_msg);
            }
        }
        else {
            m_pointcloud_publisher_filtered->publish(*msg_filtered);
        }

        size_t raw_points = msg_raw ? static_cast<size_t>(msg_raw->width) * msg_raw->height : 0;
        const char* raw_frame = msg_raw ? msg_raw->header.frame_id.c_str() : "N/A";
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                             "点云发布状态 - 原始点云: %zu 点, 坐标系=%s; 过滤点云: %zu 点, 坐标系=%s; "
                             "当前提取方法=%s, 地面过滤判断坐标系=%s, 保留高度范围=(%.3f, %.3f] m",
                             raw_points, raw_frame,
                             static_cast<size_t>(msg_filtered->width) * msg_filtered->height,
                             msg_filtered->header.frame_id.c_str(), obstacle_extraction_method_.c_str(),
                             ground_filter_frame_.c_str(), ground_filter_min_height_, ground_filter_max_height_);

        if (msg_raw && msg_raw->width > 0 && msg_filtered->width == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "过滤后点云为空：raw 有 %zu 点，但 filtered 为 0 "
                                 "点。可能原因：过滤算法参数过严，或当前视野内没有满足条件的障碍点",
                                 raw_points);
        }
    }

    // 成功后跨帧复用完整外参；源坐标系改变时重新获取，失败时不使用旧坐标系缓存。
    // 假设相机与 base_link 相对位姿固定，安装标定更新后需重启节点。
    bool cacheCameraExtrinsics(const std::string& camera_frame, tf2_ros::Buffer& tf_buffer) {
        if (camera_extrinsics_.matches(camera_frame)) {
            return true;
        }
        if (camera_frame.empty() || !rclcpp::ok()) {
            return false;
        }
        try {
            const auto transform = tf_buffer.lookupTransform(
                "base_link", camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
            camera_extrinsics_.set(transform);
            RCLCPP_INFO(get_logger(), "缓存相机完整外参 (base_link <- %s)，光轴 yaw: %.3f rad",
                        camera_frame.c_str(), camera_extrinsics_.opticalAxisYaw());
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                "相机外参未就绪 (base_link <- %s)，跳过本帧转换并重试: %s",
                                camera_frame.c_str(), ex.what());
            return false;
        }
    }

    sensor_msgs::msg::LaserScan::SharedPtr pointCloudToLaserScanBaseLink(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
        const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
        const rclcpp::Logger& logger) {
        if (!cloud_msg || cloud_msg->width == 0 || cloud_msg->height == 0) {
            return nullptr;
        }
        const bool already_in_base_link = cloud_msg->header.frame_id == "base_link";
        if (!already_in_base_link && !cacheCameraExtrinsics(cloud_msg->header.frame_id, *tf_buffer)) {
            return nullptr;
        }
        // 已在机身系的点云无需转换，其坐标轴不能再视为相机光轴。
        const double center_yaw = already_in_base_link
            ? (camera_name_ == "rear" ? M_PI : 0.0) : camera_extrinsics_.opticalAxisYaw();
        // ================= 参数（写死） =================
        constexpr double DEG2RAD = M_PI / 180.0;
        constexpr double HFOV = 86.0 * DEG2RAD;

        constexpr double ANGLE_INCREMENT = 1.0 * DEG2RAD;

        constexpr double RANGE_MIN = 0.15;
        constexpr double RANGE_MAX = 4.0;

        constexpr double MIN_HEIGHT = -0.25;
        constexpr double MAX_HEIGHT = 1.7;

        constexpr bool USE_INF = true;
        constexpr double INF_EPSILON = 0.01;

        double angle_min = center_yaw - HFOV / 2.0;
        double angle_max = center_yaw + HFOV / 2.0;

        // ================= 构造 LaserScan =================
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = cloud_msg->header.stamp;
        scan->header.frame_id = "base_link";

        scan->angle_min = angle_min;
        scan->angle_max = angle_max;
        scan->angle_increment = ANGLE_INCREMENT;
        scan->time_increment = 0.0;
        scan->range_min = RANGE_MIN;
        scan->range_max = RANGE_MAX;

        // ⚠️ 官方用 ceil，而不是 +1
        const uint32_t ranges_size = std::ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);

        if (USE_INF) {
            scan->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
        } else {
            scan->ranges.assign(ranges_size, scan->range_max + INF_EPSILON);
        }

        // ================= TF 到 base_link =================
        sensor_msgs::msg::PointCloud2 cloud_bl;
        if (cloud_msg->header.frame_id != scan->header.frame_id) {
            if (!rclcpp::ok()) {
                return nullptr;
            }
            try {
                // 复用完整旋转和平移，避免每帧查询 TF；点云时间戳保持不变。
                camera_extrinsics_.transformCloud(*cloud_msg, cloud_bl);
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(logger, "点云转换到 base_link 失败，无法生成 LaserScan: %s", ex.what());
                return nullptr;
            }
        } else {
            cloud_bl = *cloud_msg;
        }

        // ================= 遍历点云（官方 iterator 方式） =================
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_bl, "x"), iter_y(cloud_bl, "y"),
             iter_z(cloud_bl, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                continue;
            }

            // 高度裁剪
            if (*iter_z < MIN_HEIGHT || *iter_z > MAX_HEIGHT) {
                continue;
            }

            const double range = std::hypot(*iter_x, *iter_y);
            if (range < RANGE_MIN || range > RANGE_MAX) {
                continue;
            }

            const double angle = std::atan2(*iter_y, *iter_x);
            const double angle_diff = std::atan2(std::sin(angle - center_yaw), std::cos(angle - center_yaw));

            if (std::abs(angle_diff) > HFOV / 2.0) {
                continue;
            }

            const int index = static_cast<int>((angle_diff + HFOV / 2.0) / scan->angle_increment);

            if (index >= 0 && index < static_cast<int>(ranges_size)) {
                if (range < scan->ranges[index]) {
                    scan->ranges[index] = range;
                }
            }
        }

        const auto stamp_nanoseconds = static_cast<std::int64_t>(cloud_msg->header.stamp.sec) * 1'000'000'000LL +
                                       static_cast<std::int64_t>(cloud_msg->header.stamp.nanosec);
        scan->scan_time = scan_time_tracker_.update(stamp_nanoseconds);
        return scan;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr makeEmptyXYZPointCloud(const std_msgs::msg::Header& header) {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg->header = header;
        msg->height = 1;
        msg->width = 0;
        msg->is_dense = false;
        msg->is_bigendian = false;
        msg->fields.resize(3);
        msg->fields[0].name = "x";
        msg->fields[0].offset = 0;
        msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[0].count = 1;
        msg->fields[1].name = "y";
        msg->fields[1].offset = 4;
        msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[1].count = 1;
        msg->fields[2].name = "z";
        msg->fields[2].offset = 8;
        msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[2].count = 1;
        msg->point_step = 12;
        msg->row_step = 0;
        return msg;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud2FromXYZBuffer(
        const std::vector<float>& xyz_buffer,
        const std_msgs::msg::Header& header,
        const std::string& fallback_frame = "camera_frame") {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg->header = header;
        if (msg->header.frame_id.empty()) {
            msg->header.frame_id = fallback_frame;
        }
        msg->height = 1;
        msg->width = xyz_buffer.size() / 3;
        msg->is_dense = false;
        msg->is_bigendian = false;
        msg->fields.resize(3);
        msg->fields[0].name = "x";
        msg->fields[0].offset = 0;
        msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[0].count = 1;
        msg->fields[1].name = "y";
        msg->fields[1].offset = 4;
        msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[1].count = 1;
        msg->fields[2].name = "z";
        msg->fields[2].offset = 8;
        msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[2].count = 1;
        msg->point_step = 12;
        msg->row_step = msg->point_step * msg->width;
        msg->data.resize(msg->row_step * msg->height);
        uint8_t* data_ptr = msg->data.data();
        for (size_t i = 0; i < msg->width; ++i) {
            memcpy(data_ptr + i * msg->point_step + 0, &xyz_buffer[i * 3 + 0], sizeof(float));
            memcpy(data_ptr + i * msg->point_step + 4, &xyz_buffer[i * 3 + 1], sizeof(float));
            memcpy(data_ptr + i * msg->point_step + 8, &xyz_buffer[i * 3 + 2], sizeof(float));
        }
        return msg;
    }

    // 对点云进行滤波处理：先体素降采样，后做 TF 变换与高度过滤。
    sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& input_msg) {
        if (!input_msg || input_msg->width == 0) {
            return input_msg;
        }

        // 步骤1：ROS Msg -> PCL PointCloud<PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *cloud_sensor_xyz);

        // 步骤2：体素降采样（在相机原生坐标系下处理，大幅减少点数）
        if (flag_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud_sensor_xyz);
            voxel_filter.setLeafSize(ds_leaf_size_, ds_leaf_size_, ds_leaf_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*cloud_filtered);
            cloud_sensor_xyz = cloud_filtered;
        }

        // 步骤3：TF 变换至地面过滤坐标系（base_link），仅对降采样后的少聚点做变换，极度高效
        sensor_msgs::msg::PointCloud2 sensor_msg_after_voxel;
        pcl::toROSMsg(*cloud_sensor_xyz, sensor_msg_after_voxel);
        sensor_msg_after_voxel.header = input_msg->header;

        sensor_msgs::msg::PointCloud2 filter_frame_msg;
        try {
            if (sensor_msg_after_voxel.header.frame_id != ground_filter_frame_) {
                if (!rclcpp::ok()) {
                    return nullptr;
                }
                if (ground_filter_frame_ == "base_link") {
                    // 高度过滤与 LaserScan 共用完整静态外参，由先执行的阶段初始化。
                    // 即使关闭 LaserScan，也可在此独立初始化；保留点云采样时间戳。
                    if (!cacheCameraExtrinsics(sensor_msg_after_voxel.header.frame_id, *tf_buffer_)) {
                        return nullptr;
                    }
                    camera_extrinsics_.transformCloud(sensor_msg_after_voxel, filter_frame_msg);
                } else {
                    // 自定义目标系可能相对机身运动，不能套用相机到 base_link 的静态外参。
                    tf_buffer_->transform(sensor_msg_after_voxel, filter_frame_msg, ground_filter_frame_,
                                          tf2::durationFromSec(0.05));
                }
            } else {
                filter_frame_msg = sensor_msg_after_voxel;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "点云无法转换到 %s 做地面过滤，本帧不发布 filtered 点云: %s",
                                 ground_filter_frame_.c_str(), ex.what());
            return nullptr;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_frame_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(filter_frame_msg, *cloud_filter_frame_xyz);

        if (cloud_filter_frame_xyz->points.size() != cloud_sensor_xyz->points.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "地面过滤前后点数不一致，跳过本帧 filtered 点云: sensor=%zu, %s=%zu",
                                 cloud_sensor_xyz->points.size(), ground_filter_frame_.c_str(),
                                 cloud_filter_frame_xyz->points.size());
            return nullptr;
        }

        // 步骤4：在 base_link 坐标系下进行高度过滤（剔除地面点与过高点）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_height_filtered->header = cloud_sensor_xyz->header;
        cloud_height_filtered->is_dense = cloud_sensor_xyz->is_dense;
        cloud_height_filtered->points.reserve(cloud_filter_frame_xyz->points.size());

        for (size_t i = 0; i < cloud_filter_frame_xyz->points.size(); ++i) {
            const auto& height_point = cloud_filter_frame_xyz->points[i];
            const auto& sensor_point = cloud_sensor_xyz->points[i];
            if (!std::isfinite(height_point.x) || !std::isfinite(height_point.y) || !std::isfinite(height_point.z) ||
                !std::isfinite(sensor_point.x) || !std::isfinite(sensor_point.y) || !std::isfinite(sensor_point.z)) {
                continue;
            }
            if (height_point.z > static_cast<float>(ground_filter_min_height_) &&
                height_point.z <= static_cast<float>(ground_filter_max_height_)) {
                cloud_height_filtered->points.push_back(sensor_point);
            }
        }
        cloud_height_filtered->width = cloud_height_filtered->points.size();
        cloud_height_filtered->height = 1;

        if (cloud_height_filtered->points.empty()) {
            return makeEmptyXYZPointCloud(input_msg->header);
        }

        // 步骤5：PCL 点云转换回 ROS PointCloud2 消息
        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_height_filtered, *output_msg);

        output_msg->header = input_msg->header;
        return output_msg;
    }

    // 双视差（UV-Disparity）地面剔除与障碍物提取点云
    bool computePointCloudsUVDisparity(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                                       sensor_msgs::msg::PointCloud2::SharedPtr& out_obstacle) {
        const int width = depth_msg->width;
        const int height = depth_msg->height;
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "深度图尺寸非法: %dx%d", width, height);
            return false;
        }

        const auto fx = camera_info_msg->k[0];
        const auto fy = camera_info_msg->k[4];
        const auto cx = camera_info_msg->k[2];
        const auto cy = camera_info_msg->k[5];
        if (fx <= 0.f || fy <= 0.f) {
            RCLCPP_WARN(get_logger(), "相机内参非法 fx=%.3f fy=%.3f", fx, fy);
            return false;
        }

        const bool is_uint16_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                                     depth_msg->encoding == sensor_msgs::image_encodings::MONO16;
        const bool is_float32_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
        if (!is_uint16_depth && !is_float32_depth) {
            RCLCPP_WARN(get_logger(), "双视差不支持深度图编码: %s", depth_msg->encoding.c_str());
            return false;
        }
        if (depth_msg->is_bigendian) {
            RCLCPP_WARN(get_logger(), "双视差暂不支持大端序深度图");
            return false;
        }

        const size_t bytes_per_pixel = is_uint16_depth ? sizeof(uint16_t) : sizeof(float);
        const size_t minimum_step = static_cast<size_t>(width) * bytes_per_pixel;
        const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
        if (depth_msg->step < minimum_step || depth_msg->data.size() < expected_size) {
            RCLCPP_WARN(get_logger(), "双视差深度数据布局非法: step=%u, 最小step=%zu, data=%zu, 期望=%zu",
                        depth_msg->step, minimum_step, depth_msg->data.size(), expected_size);
            return false;
        }

        const size_t pixel_count = static_cast<size_t>(width) * height;

        const float min_z = static_cast<float>(uv_disparity_min_z_);
        const float max_z = static_cast<float>(uv_disparity_max_z_);
        const float dz = static_cast<float>(uv_disparity_dz_);
        if (dz <= 0.f) return false;

        const int N_z = static_cast<int>((max_z - min_z) / dz);
        if (N_z <= 0) return false;

        const float inv_dz  = 1.0f / dz;
        const float cy_f    = static_cast<float>(cy);
        const float fy_f    = static_cast<float>(fy);
        const float cx_f    = static_cast<float>(cx);
        const float fx_f    = static_cast<float>(fx);

        // 如果内参改变，预先建立 X, Y 反投影比例查找表
        if (lut_u_coef_.size() != static_cast<size_t>(width)) {
            lut_u_coef_.resize(width);
            for (int u = 0; u < width; ++u) {
                lut_u_coef_[u] = (static_cast<float>(u) - cx_f) / fx_f;
            }
        }
        if (lut_v_coef_.size() != static_cast<size_t>(height)) {
            lut_v_coef_.resize(height);
            for (int v = 0; v < height; ++v) {
                lut_v_coef_[v] = (static_cast<float>(v) - cy_f) / fy_f;
            }
        }

        // ── ① 扁平 V-Depth 直方图（复用 vdepth_buf_，消除嵌套 vector 每帧堆分配）──
        const size_t vdepth_size = static_cast<size_t>(height) * N_z;
        if (vdepth_buf_.size() < vdepth_size) vdepth_buf_.resize(vdepth_size);
        std::fill(vdepth_buf_.begin(), vdepth_buf_.begin() + vdepth_size, 0);

        // 如果是 uint16 深度，预先生成查找表 (LUT)，避免在百万像素热循环中计算倒数与分支
        if (is_uint16_depth) {
            const size_t lut_size = static_cast<size_t>(max_z * 1000.0f) + 100;
            if (lut_z_idx_.size() < lut_size) lut_z_idx_.resize(lut_size);
            
            // 第一阶段 V-Depth 查找表初始化
            for (size_t mm = 0; mm < lut_size; ++mm) {
                const float z = mm * 0.001f;
                if (z < min_z || z > max_z) {
                    lut_z_idx_[mm] = -1;
                } else {
                    lut_z_idx_[mm] = static_cast<int>((z - min_z) * inv_dz);
                }
            }

            // 统计 V-depth
            for (int v = height / 3; v < height; ++v) {
                const int v_step = (v >= height * 3 / 4) ? 1 : 3;
                if (v % v_step != 0) continue;
                const int u_step = (v >= height * 3 / 4) ? 1 : 2;
                const auto* row = reinterpret_cast<const uint16_t*>(
                    depth_msg->data.data() + static_cast<size_t>(v) * depth_msg->step);
                int* vrow = vdepth_buf_.data() + static_cast<size_t>(v) * N_z;
                for (int u = 0; u < width; u += u_step) {
                    const uint16_t raw = row[u];
                    if (raw == 0 || raw >= lut_size) continue; // 剪裁 raw==0 过滤无用点
                    const int z_idx = lut_z_idx_[raw];
                    if (z_idx >= 0 && static_cast<unsigned>(z_idx) < static_cast<unsigned>(N_z)) {
                        vrow[z_idx]++;
                    }
                }
            }
        } else {
            // float32 回退分支
            for (int v = height / 3; v < height; ++v) {
                const int v_step = (v >= height * 3 / 4) ? 1 : 3;
                if (v % v_step != 0) continue;
                const int u_step = (v >= height * 3 / 4) ? 1 : 2;
                const auto* row = reinterpret_cast<const float*>(
                    depth_msg->data.data() + static_cast<size_t>(v) * depth_msg->step);
                int* vrow = vdepth_buf_.data() + static_cast<size_t>(v) * N_z;
                for (int u = 0; u < width; u += u_step) {
                    const float z = row[u];
                    if (!std::isfinite(z) || z < min_z || z > max_z) continue;
                    const int z_idx = static_cast<int>((z - min_z) * inv_dz);
                    if (static_cast<unsigned>(z_idx) < static_cast<unsigned>(N_z)) vrow[z_idx]++;
                }
            }
        }

        // ── ② 提取地面候选点（复用 sample_buf_）──
        sample_buf_.clear();
        int* vdepth_ptr = vdepth_buf_.data() + static_cast<size_t>(height / 2) * N_z;
        for (int v = height / 2; v < height; ++v) {
            const int v_step = (v >= height * 3 / 4) ? 2 : 3;
            if (v % v_step != 0) {
                vdepth_ptr += N_z;
                continue;
            }
            const int count_thresh = (v >= height * 3 / 4) ? 3 : 2;
            const float fv = static_cast<float>(v);
            for (int z_idx = 0; z_idx < N_z; ++z_idx) {
                if (vdepth_ptr[z_idx] > count_thresh) {
                    sample_buf_.push_back({min_z + z_idx * dz, fv});
                }
            }
            vdepth_ptr += N_z;
        }
        constexpr size_t kMaxSamplePoints = 80;
        if (sample_buf_.size() > kMaxSamplePoints) {
            const size_t stride = sample_buf_.size() / kMaxSamplePoints;
            size_t j = 0;
            for (size_t i = 0; i < sample_buf_.size(); i += stride) sample_buf_[j++] = sample_buf_[i];
            sample_buf_.resize(j);
        }

        // ── ③ 两点 RANSAC 拟合 v = a + b/z，内点比 > 70% 立即早退 ──
        float best_a = cy_f;
        float best_b = fy_f * 0.4f;
        int max_inliers = 0;
        const float inlier_threshold_v = 12.0f;

        if (has_last_fit_) {
            const float la = static_cast<float>(last_fit_a_);
            const float lb = static_cast<float>(last_fit_b_);
            int cnt = 0;
            for (const auto& p : sample_buf_) {
                if (std::abs(p.v - (la + lb / p.z)) < inlier_threshold_v) cnt++;
            }
            if (cnt > 0) { max_inliers = cnt; best_a = la; best_b = lb; }
        }

        if (sample_buf_.size() >= 2) {
            std::uniform_int_distribution<size_t> dist(0, sample_buf_.size() - 1);
            const int early_exit_cnt = static_cast<int>(static_cast<float>(sample_buf_.size()) * 0.70f);
            for (int iter = 0; iter < 20; ++iter) {
                const auto& p1 = sample_buf_[dist(rng_)];
                const auto& p2 = sample_buf_[dist(rng_)];
                const float inv_z1 = 1.0f / p1.z, inv_z2 = 1.0f / p2.z;
                if (std::abs(inv_z1 - inv_z2) < 0.05f) continue;
                const float b_cand = (p1.v - p2.v) / (inv_z1 - inv_z2);
                const float a_cand = p1.v - b_cand * inv_z1;
                if (!std::isfinite(a_cand) || !std::isfinite(b_cand) || b_cand <= 0.0f) continue;
                int inliers = 0;
                for (const auto& p : sample_buf_) {
                    if (std::abs(p.v - (a_cand + b_cand / p.z)) < inlier_threshold_v) inliers++;
                }
                if (inliers > max_inliers) {
                    max_inliers = inliers; best_a = a_cand; best_b = b_cand;
                    if (inliers >= early_exit_cnt) break;
                }
            }
        }

        const bool fit_success = !sample_buf_.empty() && max_inliers > 0 &&
            static_cast<float>(max_inliers) / static_cast<float>(sample_buf_.size()) > 0.30f;

        if (fit_success) {
            if (has_last_fit_) {
                constexpr float alpha = 0.15f;
                best_a = alpha * best_a + (1.0f - alpha) * static_cast<float>(last_fit_a_);
                best_b = alpha * best_b + (1.0f - alpha) * static_cast<float>(last_fit_b_);
            }
            last_fit_a_ = best_a; last_fit_b_ = best_b; has_last_fit_ = true;
        } else {
            if (has_last_fit_) {
                best_a = static_cast<float>(last_fit_a_); best_b = static_cast<float>(last_fit_b_);
            } else {
                best_a = cy_f; best_b = fy_f * 0.4f;
            }
        }

        // ── ④ 障碍物检测 + 扁平 U-Depth 直方图（复用 udepth_buf_）──
        const size_t udepth_size = static_cast<size_t>(width) * N_z;
        if (udepth_buf_.size() < udepth_size) udepth_buf_.resize(udepth_size);
        std::fill(udepth_buf_.begin(), udepth_buf_.begin() + udepth_size, 0);

        candidates_buf_.clear();
        candidates_buf_.reserve(pixel_count / 20);

        int u_min = width;
        int u_max = -1;

        if (is_uint16_depth) {
            const size_t lut_size = static_cast<size_t>(max_z * 1000.0f) + 100;
            if (lut_expected_v_.size() < lut_size) lut_expected_v_.resize(lut_size);
            if (lut_dynamic_margin_.size() < lut_size) lut_dynamic_margin_.resize(lut_size);
            
            // 增量更新检查：如果拟合参数变化小于 0.001，复用上一帧 LUT 避免重填
            if (std::abs(best_a - last_lut_a_) > 1e-3f || std::abs(best_b - last_lut_b_) > 1e-3f) {
                const float base_v_margin = static_cast<float>(uv_disparity_v_margin_);
                const float fy_gm = fy_f * static_cast<float>(uv_disparity_ground_margin_h_);

                for (size_t mm = 0; mm < lut_size; ++mm) {
                    const float z = mm * 0.001f;
                    if (z < min_z || z > max_z) {
                        lut_expected_v_[mm] = -9999.0f;
                        lut_dynamic_margin_[mm] = 0.0f;
                    } else {
                        const float inv_z = 1.0f / z;
                        lut_expected_v_[mm] = best_a + best_b * inv_z;
                        lut_dynamic_margin_[mm] = std::max(5.0f, std::min(base_v_margin, fy_gm * inv_z));
                    }
                }
                last_lut_a_ = best_a;
                last_lut_b_ = best_b;
            }

            for (int v = 0; v < height; ++v) {
                const int v_inc = (v >= height * 3 / 4) ? 2 : 3;
                if (v % v_inc != 0) continue;
                const int u_step = 3; // 扩大步长到 3，大幅减少采样像素点
                const auto* row = reinterpret_cast<const uint16_t*>(
                    depth_msg->data.data() + static_cast<size_t>(v) * depth_msg->step);
                const float fv = static_cast<float>(v);
                for (int u = 0; u < width; u += u_step) {
                    const uint16_t raw = row[u];
                    if (raw == 0 || raw >= lut_size) continue;
                    
                    const float expected_ground_v = lut_expected_v_[raw];
                    const float dyn_margin = lut_dynamic_margin_[raw];
                    if (fv < expected_ground_v - dyn_margin) {
                        const int z_idx = lut_z_idx_[raw];
                        if (z_idx >= 0 && static_cast<unsigned>(z_idx) < static_cast<unsigned>(N_z)) {
                            udepth_buf_[static_cast<size_t>(u) * N_z + z_idx]++;
                            candidates_buf_.push_back({
                                static_cast<uint16_t>(u), 
                                static_cast<uint16_t>(v), 
                                raw, z_idx
                            });
                            if (u < u_min) u_min = u;
                            if (u > u_max) u_max = u;
                        }
                    }
                }
            }
        } else {
            // float32 分支回退
            const float base_v_margin = static_cast<float>(uv_disparity_v_margin_);
            const float fy_gm = fy_f * static_cast<float>(uv_disparity_ground_margin_h_);

            for (int v = 0; v < height; ++v) {
                const int v_inc = (v >= height * 3 / 4) ? 2 : 3;
                if (v % v_inc != 0) continue;
                const int u_step = 3;
                const auto* row = reinterpret_cast<const float*>(
                    depth_msg->data.data() + static_cast<size_t>(v) * depth_msg->step);
                const float fv = static_cast<float>(v);
                for (int u = 0; u < width; u += u_step) {
                    const float z = row[u];
                    if (!std::isfinite(z) || z < min_z || z > max_z) continue;
                    const float inv_z = 1.0f / z;
                    const float expected_ground_v = best_a + best_b * inv_z;
                    const float dyn_margin = std::max(5.0f, std::min(base_v_margin, fy_gm * inv_z));
                    if (fv < expected_ground_v - dyn_margin) {
                        const int z_idx = static_cast<int>((z - min_z) * inv_dz);
                        if (static_cast<unsigned>(z_idx) < static_cast<unsigned>(N_z)) {
                            udepth_buf_[static_cast<size_t>(u) * N_z + z_idx]++;
                            candidates_buf_.push_back({
                                static_cast<uint16_t>(u), 
                                static_cast<uint16_t>(v), 
                                static_cast<uint16_t>(z * 1000.0f), z_idx
                            });
                            if (u < u_min) u_min = u;
                            if (u > u_max) u_max = u;
                        }
                    }
                }
            }
        }

        // ── ⑤ 局部积分图计算（精准区域清零 + 局部计算） ──
        const int prefix_stride = N_z + 1;
        const size_t prefix_size = static_cast<size_t>(width + 1) * prefix_stride;
        if (udepth_prefix_buf_.size() < prefix_size) udepth_prefix_buf_.resize(prefix_size);

        buf_obstacles_.clear();
        buf_obstacles_.reserve(candidates_buf_.size() * 3);

        if (u_max >= u_min) {
            const int start_u = std::max(0, u_min - 4);
            const int end_u   = std::min(width, u_max + 5);

            // 局部区域清零，避免对全图内存整体 memset
            const size_t clear_start = static_cast<size_t>(start_u) * prefix_stride;
            const size_t clear_count = static_cast<size_t>(end_u - start_u + 1) * prefix_stride;
            std::fill(udepth_prefix_buf_.begin() + clear_start, 
                      udepth_prefix_buf_.begin() + std::min(prefix_size, clear_start + clear_count), 0);

            for (int u = start_u; u < end_u; ++u) {
                const int* ucol = udepth_buf_.data() + static_cast<size_t>(u) * N_z;
                const int* prev  = udepth_prefix_buf_.data() + static_cast<size_t>(u) * prefix_stride;
                int*       cur   = udepth_prefix_buf_.data() + static_cast<size_t>(u + 1) * prefix_stride;
                int row_sum = 0;
                for (int z_idx = 0; z_idx < N_z; ++z_idx) {
                    row_sum += ucol[z_idx];
                    cur[z_idx + 1] = prev[z_idx + 1] + row_sum;
                }
            }

            // ── ⑥ 邻域支持过滤 + Fast-path + 原生 3D 网格极速降采样 ──
            const int min_u_count = std::max(3, uv_disparity_min_u_count_);
            const bool enable_voxel = flag_voxel_filter_ && ds_leaf_size_ > 0.001;
            const float inv_leaf = enable_voxel ? static_cast<float>(1.0 / ds_leaf_size_) : 1.0f;
            if (enable_voxel) voxel_grid_set_.clear();

            for (const auto& pt : candidates_buf_) {
                const float z = pt.raw_z * 0.001f;
                const int req_count = (z < 1.0f ? 4 : min_u_count);
                
                // Fast-path: 如果当前格子自身支持数已足够，直接判定通过，免去积分图范围查询
                bool pass = (udepth_buf_[static_cast<size_t>(pt.u) * N_z + pt.z_idx] >= req_count);
                if (!pass) {
                    const int u0 = std::max(0, pt.u - 3);
                    const int u1 = std::min(width, pt.u + 4);
                    const int z0 = std::max(0, pt.z_idx - 1);
                    const int z1 = std::min(N_z, pt.z_idx + 2);
                    const int support =
                        udepth_prefix_buf_[static_cast<size_t>(u1) * prefix_stride + z1] -
                        udepth_prefix_buf_[static_cast<size_t>(u0) * prefix_stride + z1] -
                        udepth_prefix_buf_[static_cast<size_t>(u1) * prefix_stride + z0] +
                        udepth_prefix_buf_[static_cast<size_t>(u0) * prefix_stride + z0];
                    pass = (support >= req_count);
                }
                
                if (pass) {
                    const float x = lut_u_coef_[pt.u] * z;
                    const float y = lut_v_coef_[pt.v] * z;

                    if (enable_voxel) {
                        // 原生 3D 空间网格去重哈希
                        const int32_t gx = static_cast<int32_t>(std::floor(x * inv_leaf));
                        const int32_t gy = static_cast<int32_t>(std::floor(y * inv_leaf));
                        const int32_t gz = static_cast<int32_t>(std::floor(z * inv_leaf));
                        const uint64_t key = (static_cast<uint64_t>(gx & 0xFFFF) << 32) |
                                             (static_cast<uint64_t>(gy & 0xFFFF) << 16) |
                                             (static_cast<uint64_t>(gz & 0xFFFF));
                        if (!voxel_grid_set_.insert(key).second) {
                            continue; // 处于同一 3D 体素内，直接去重
                        }
                    }

                    buf_obstacles_.push_back(x);
                    buf_obstacles_.push_back(y);
                    buf_obstacles_.push_back(z);
                }
            }
        }

        const std::string& frame_id = !camera_info_msg->header.frame_id.empty()
            ? camera_info_msg->header.frame_id
            : (depth_msg->header.frame_id.empty() ? "camera_frame" : depth_msg->header.frame_id);
        out_obstacle = createPointCloud2FromXYZBuffer(buf_obstacles_, depth_msg->header, frame_id);
        return true;
    }



    // 针对双视差提取的障碍物点云进行简单体素滤波
    sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloudSimple(
        const sensor_msgs::msg::PointCloud2::SharedPtr& input_msg) {
        if (!input_msg || input_msg->width == 0) {
            return input_msg;
        }

        pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*input_msg, *cloud_pcl);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_sensor_xyz);

        if (flag_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud_sensor_xyz);
            voxel_filter.setLeafSize(ds_leaf_size_, ds_leaf_size_, ds_leaf_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*cloud_filtered);
            cloud_sensor_xyz = cloud_filtered;
        }

        if (cloud_sensor_xyz->points.empty()) {
            return makeEmptyXYZPointCloud(input_msg->header);
        }

        pcl::PCLPointCloud2 output_pcl;
        pcl::toPCLPointCloud2(*cloud_sensor_xyz, output_pcl);

        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(output_pcl, *output_msg);
        output_msg->header = input_msg->header;
        return output_msg;
    }

    bool computePointClouds(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                            sensor_msgs::msg::PointCloud2::SharedPtr& out_raw) {
        const int width = depth_msg->width;
        const int height = depth_msg->height;
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "深度图尺寸非法: %dx%d", width, height);
            return false;
        }

        const bool is_uint16_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                                     depth_msg->encoding == sensor_msgs::image_encodings::MONO16;
        const bool is_float32_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
        if (!is_uint16_depth && !is_float32_depth) {
            RCLCPP_WARN(get_logger(), "不支持的深度图编码: %s（仅支持 16UC1/mono16 和 32FC1）",
                        depth_msg->encoding.c_str());
            return false;
        }
        if (depth_msg->is_bigendian) {
            RCLCPP_WARN(get_logger(), "暂不支持大端序深度图");
            return false;
        }

        const size_t bytes_per_pixel = is_uint16_depth ? sizeof(uint16_t) : sizeof(float);
        const size_t minimum_step = static_cast<size_t>(width) * bytes_per_pixel;
        if (depth_msg->step < minimum_step) {
            RCLCPP_WARN(get_logger(), "深度图行跨度不足: %u < %zu", depth_msg->step, minimum_step);
            return false;
        }
        const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
        if (depth_msg->data.size() < expected_size) {
            RCLCPP_WARN(get_logger(), "深度数据长度不足: %zu < %zu", depth_msg->data.size(), expected_size);
            return false;
        }

        const auto fx = camera_info_msg->k[0];
        const auto fy = camera_info_msg->k[4];
        const auto cx = camera_info_msg->k[2];
        const auto cy = camera_info_msg->k[5];
        if (fx <= 0.f || fy <= 0.f) {
            RCLCPP_WARN(get_logger(), "相机内参非法 fx=%.3f fy=%.3f", fx, fy);
            return false;
        }

        if (!depth_msg->header.frame_id.empty() && !camera_info_msg->header.frame_id.empty() &&
            depth_msg->header.frame_id != camera_info_msg->header.frame_id) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 50000,
                                 "Depth frame (%s) 与 CameraInfo 光学 frame (%s) 不一致；重建点云使用 CameraInfo frame",
                                 depth_msg->header.frame_id.c_str(), camera_info_msg->header.frame_id.c_str());
        }

        const size_t pixel_count = static_cast<size_t>(width) * height;

        buf_raw_.clear();
        buf_raw_.reserve(pixel_count * 3 / 2);

        const int step_u = std::max(1, step_u_);
        const int step_v = std::max(1, step_v_);

        for (int v = 0; v < height; v += step_v) {
            const auto* row = depth_msg->data.data() + static_cast<size_t>(v) * depth_msg->step;
            for (int u = 0; u < width; u += step_u) {
                const auto* pixel = row + static_cast<size_t>(u) * bytes_per_pixel;
                float z = 0.0f;
                if (is_uint16_depth) {
                    uint16_t depth_mm = 0;
                    std::memcpy(&depth_mm, pixel, sizeof(depth_mm));
                    z = static_cast<float>(depth_mm) / 1000.0f;
                } else {
                    std::memcpy(&z, pixel, sizeof(z));
                }
                if (!std::isfinite(z) || z <= 0.f || z > static_cast<float>(depth_max_range_)) {
                    continue;
                }
                float x = (static_cast<float>(u) - cx) * z / fx;
                float y = (static_cast<float>(v) - cy) * z / fy;

                // raw：不过滤地面
                buf_raw_.push_back(x);
                buf_raw_.push_back(y);
                buf_raw_.push_back(z);
            }
        }

        std::string fallback_frame = !camera_info_msg->header.frame_id.empty()
                                         ? camera_info_msg->header.frame_id
                                         : "camera_frame";
        out_raw = createPointCloud2FromXYZBuffer(buf_raw_, depth_msg->header, fallback_frame);
        return true;
    }

    // 订阅器（原生 ROS 订阅）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 最近一次收到的相机参数
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;

    // 发布器与 TF 转换
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_raw;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_filtered;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laserscan_publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // 预处理参数
    bool flag_voxel_filter_ = true;
    bool flag_mls_filter_ = false;
    bool flag_sor_filter_ = false;
    double ds_leaf_size_ = 0.05;
    int mls_polynomial_order_ = 2;
    double mls_search_radius_ = 0.2;
    int sor_mean_k_ = 50;
    double sor_stddev_mul_thresh_ = 1.0;
    bool enable_laserscan_ = true;
    bool division_mode_ = true;
    dexe_sensor_process::FrameDivisionState frame_division_state_{true};
    dexe_sensor_process::SourceStampIntervalTracker scan_time_tracker_;
    int step_u_ = 2;
    int step_v_ = 3;
    std::string ground_filter_frame_ = "base_link";
    double ground_filter_min_height_ = 0.05;
    double ground_filter_max_height_ = 1.5;
    double depth_max_range_ = 3.0;

    // 双视差障碍物提取参数
    std::string obstacle_extraction_method_ = "height_filter";
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

    // 相机名称与相机到 base_link 的完整静态外参缓存
    std::string camera_name_ = "rear";
    dexe_sensor_process::CameraExtrinsics camera_extrinsics_;

    // 堆内存重用缓冲区（避免高频分配/销毁导致内存碎片化与内存高波动）
    std::vector<float> buf_raw_;
    std::vector<float> buf_obstacles_;

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
};

int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdToPointCloud>());
    rclcpp::shutdown();
    return 0;
}

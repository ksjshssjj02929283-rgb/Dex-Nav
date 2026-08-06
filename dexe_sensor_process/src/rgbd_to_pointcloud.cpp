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
        this->declare_parameter<std::string>("obstacle_extraction_method", "height_filter");
        this->declare_parameter<double>("uv_disparity_min_z", 0.3);
        this->declare_parameter<double>("uv_disparity_max_z", 3.0);
        this->declare_parameter<double>("uv_disparity_dz", 0.02);
        this->declare_parameter<int>("uv_disparity_v_margin", 15);
        this->declare_parameter<int>("uv_disparity_min_u_count", 5);

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

        // （虚拟点云相关参数已移除）

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

        obstacle_extraction_method_ = this->get_parameter("obstacle_extraction_method").as_string();
        uv_disparity_min_z_ = this->get_parameter("uv_disparity_min_z").as_double();
        uv_disparity_max_z_ = this->get_parameter("uv_disparity_max_z").as_double();
        uv_disparity_dz_ = this->get_parameter("uv_disparity_dz").as_double();
        uv_disparity_v_margin_ = this->get_parameter("uv_disparity_v_margin").as_int();
        uv_disparity_min_u_count_ = this->get_parameter("uv_disparity_min_u_count").as_int();

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

        // 计算 raw（不过滤地面）
        sensor_msgs::msg::PointCloud2::SharedPtr msg_raw;
        if (!computePointClouds(depth_msg, camera_info_msg, msg_raw)) {
            RCLCPP_WARN(get_logger(), "深度图转换点云失败");
            return;
        }

        // 对 msg_filtered 进行滤波处理：按配置使用双视差法或基准高度过滤法
        sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered;
        if (obstacle_extraction_method_ == "uv_disparity") {
            sensor_msgs::msg::PointCloud2::SharedPtr msg_obstacle_raw;
            if (computePointCloudsUVDisparity(depth_msg, camera_info_msg, msg_obstacle_raw)) {
                msg_filtered = filterPointCloudSimple(msg_obstacle_raw);
            }
        } else {
            msg_filtered = filterPointCloud(msg_raw);
        }

        // 发布 raw（未剔除地面）
        m_pointcloud_publisher_raw->publish(*msg_raw);

        if (!msg_filtered) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "本帧 filtered 点云未发布：原始点云有 %zu 点，但地面过滤所需 TF "
                                 "或内部转换暂时不可用。为避免 costmap 被空点云反复清除，本帧只发布 raw 点云",
                                 static_cast<size_t>(msg_raw->width) * msg_raw->height);
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

        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                             "点云发布状态 - 原始点云: %zu 点, 坐标系=%s; 过滤点云: %zu 点, 坐标系=%s; "
                             "地面过滤判断坐标系=%s, 保留高度范围=(%.3f, %.3f] m",
                             static_cast<size_t>(msg_raw->width) * msg_raw->height, msg_raw->header.frame_id.c_str(),
                             static_cast<size_t>(msg_filtered->width) * msg_filtered->height,
                             msg_filtered->header.frame_id.c_str(), ground_filter_frame_.c_str(),
                             ground_filter_min_height_, ground_filter_max_height_);

        if (msg_raw->width > 0 && msg_filtered->width == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "过滤后点云为空：raw 有 %zu 点，但 filtered 为 0 "
                                 "点。可能原因：地面高度阈值过严，或当前视野内没有满足高度范围的障碍点",
                                 static_cast<size_t>(msg_raw->width) * msg_raw->height);
        }
    }

    double getCameraYawInBaseLink(const std::string& camera_frame, tf2_ros::Buffer& tf_buffer) {
        // 1. 若已经成功获取并缓存过静态外参 Yaw 角，直接返回缓存值，免除每帧 TF 查询开销
        if (has_cached_camera_yaw_) {
            return cached_camera_yaw_;
        }

        // 根据相机名称（front/rear）设定初值/回退值
        double default_yaw = (camera_name_ == "rear") ? M_PI : 0.0;

        geometry_msgs::msg::Vector3Stamped v_cam, v_base;
        v_cam.header.frame_id = camera_frame;
        v_cam.header.stamp = rclcpp::Time(0);

        // 相机光轴（相机正前方向量 (0,0,1)）
        v_cam.vector.x = 0.0;
        v_cam.vector.y = 0.0;
        v_cam.vector.z = 1.0;

        try {
            if (!rclcpp::ok() || !tf_buffer.canTransform("base_link", camera_frame, rclcpp::Time(0),
                                                        rclcpp::Duration::from_seconds(0.05))) {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rgbd_to_pointcloud"), *this->get_clock(), 5000,
                                     "等待 base_link -> %s 变换超时或节点正在退出（使用 %s 相机默认 Yaw 角: %.2f rad）",
                                     camera_frame.c_str(), camera_name_.c_str(), default_yaw);
                return default_yaw;
            }

            tf_buffer.transform(v_cam, v_base, "base_link");
            cached_camera_yaw_ = std::atan2(v_base.vector.y, v_base.vector.x);
            has_cached_camera_yaw_ = true;
            RCLCPP_INFO(get_logger(), "成功获取并缓存相机 [%s] (%s) 在 base_link 下的姿态 Yaw 角: %.3f rad (%.1f deg)",
                        camera_name_.c_str(), camera_frame.c_str(), cached_camera_yaw_,
                        cached_camera_yaw_ * 180.0 / M_PI);
            return cached_camera_yaw_;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rgbd_to_pointcloud"), *this->get_clock(), 5000,
                                 "无法获取变换: %s（使用默认 Yaw 角: %.2f rad）", ex.what(), default_yaw);
            return default_yaw;
        }
    }

    sensor_msgs::msg::LaserScan::SharedPtr pointCloudToLaserScanBaseLink(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
        const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
        const rclcpp::Logger& logger) {
        if (!cloud_msg || cloud_msg->width == 0 || cloud_msg->height == 0) {
            return nullptr;
        }
        double center_yaw = getCameraYawInBaseLink(cloud_msg->header.frame_id, *tf_buffer);
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
                tf_buffer->transform(*cloud_msg, cloud_bl, scan->header.frame_id, tf2::durationFromSec(0.05));
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
                tf_buffer_->transform(sensor_msg_after_voxel, filter_frame_msg, ground_filter_frame_,
                                      tf2::durationFromSec(0.05));
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

        constexpr float depth_scale = 1000.0f;
        const auto fx = camera_info_msg->k[0];
        const auto fy = camera_info_msg->k[4];
        const auto cx = camera_info_msg->k[2];
        const auto cy = camera_info_msg->k[5];
        if (fx <= 0.f || fy <= 0.f) {
            RCLCPP_WARN(get_logger(), "相机内参非法 fx=%.3f fy=%.3f", fx, fy);
            return false;
        }

        const size_t pixel_count = static_cast<size_t>(width) * height;
        const auto* depth_ptr = reinterpret_cast<const uint16_t*>(depth_msg->data.data());
        const size_t depth_words = depth_msg->data.size() / sizeof(uint16_t);
        if (depth_words < pixel_count) {
            RCLCPP_WARN(get_logger(), "深度数据长度不足: %zu < %zu", depth_words, pixel_count);
            return false;
        }

        const float min_z = static_cast<float>(uv_disparity_min_z_);
        const float max_z = static_cast<float>(uv_disparity_max_z_);
        const float dz = static_cast<float>(uv_disparity_dz_);
        if (dz <= 0.f) return false;

        const int N_z = static_cast<int>((max_z - min_z) / dz);
        if (N_z <= 0) return false;

        // V-depth 直方图
        std::vector<std::vector<int>> V_depth(height, std::vector<int>(N_z, 0));

        // 统计 V-depth 直方图 (仅对图像下半部分)
        for (int v = height / 2; v < height; v += 3) {
            for (int u = 0; u < width; u += 2) {
                const size_t idx = static_cast<size_t>(v) * width + u;
                uint16_t depth_raw_val = depth_ptr[idx];
                if (depth_raw_val == 0) continue;

                float z = static_cast<float>(depth_raw_val) / depth_scale;
                if (z < min_z || z > max_z) continue;

                int z_idx = static_cast<int>((z - min_z) / dz);
                if (z_idx >= 0 && z_idx < N_z) {
                    V_depth[v][z_idx]++;
                }
            }
        }

        // 提取地面拟合数据点
        struct GroundPoint {
            float z;
            float v;
        };
        std::vector<GroundPoint> sample_points;
        for (int v = height / 2; v < height; v += 3) {
            for (int z_idx = 0; z_idx < N_z; ++z_idx) {
                if (V_depth[v][z_idx] > 2) {
                    float z_val = min_z + z_idx * dz;
                    sample_points.push_back({z_val, static_cast<float>(v)});
                }
            }
        }

        // RANSAC 拟合地面线 v = a * z + b
        float best_a = 0.f;
        float best_b = 0.f;
        int max_inliers = 0;
        const int iterations = 100;
        const float inlier_threshold_v = 15.0f;

        // 1. 如果有上一帧成功拟合的地面参数，在当前点集中评估其一致性，作为初始最佳估算
        if (has_last_fit_) {
            int last_inliers = 0;
            for (const auto& p : sample_points) {
                float expected_v = last_fit_a_ * p.z + last_fit_b_;
                if (std::abs(p.v - expected_v) < inlier_threshold_v) {
                    last_inliers++;
                }
            }
            if (last_inliers > 0) {
                max_inliers = last_inliers;
                best_a = last_fit_a_;
                best_b = last_fit_b_;
            }
        }

        // 2. 运行 RANSAC 搜索潜在的更佳地面模型
        if (sample_points.size() >= 5) {
            std::random_device rd;
            std::mt19937 rng(rd());
            std::uniform_int_distribution<size_t> dist(0, sample_points.size() - 1);

            for (int iter = 0; iter < iterations; ++iter) {
                size_t idx1 = dist(rng);
                size_t idx2 = dist(rng);
                if (idx1 == idx2) continue;

                const auto& p1 = sample_points[idx1];
                const auto& p2 = sample_points[idx2];
                if (std::abs(p1.z - p2.z) < 0.05f) continue;

                float a = (p2.v - p1.v) / (p2.z - p1.z);
                if (a >= 0.f) continue;  // 斜率必须为负数（越远对应的图像行号 v 越小）

                float b = p1.v - a * p1.z;

                int inliers = 0;
                for (const auto& p : sample_points) {
                    float expected_v = a * p.z + b;
                    if (std::abs(p.v - expected_v) < inlier_threshold_v) {
                        inliers++;
                    }
                }

                if (inliers > max_inliers) {
                    max_inliers = inliers;
                    best_a = a;
                    best_b = b;
                }
            }
        }

        bool fit_success = false;
        // 如果一致点比例达到 15% 并且确实有有效一致点支持，判定为本帧拟合成功
        if (max_inliers > 0 && static_cast<float>(max_inliers) / sample_points.size() > 0.15f) {
            fit_success = true;
        }

        // 3. 拟合决策与历史记录更新
        if (fit_success) {
            // 本帧拟合成功，更新上一帧参数供下一帧使用
            last_fit_a_ = best_a;
            last_fit_b_ = best_b;
            has_last_fit_ = true;

            RCLCPP_INFO_ONCE(get_logger(), "双视差地面线拟合成功: v = %.2f * z + %.2f (一致点: %d/%zu)", best_a, best_b,
                             max_inliers, sample_points.size());
        } else {
            // 本帧拟合失败：优先沿用历史高置信度模型；若无历史数据，才回退至默认经验线
            if (has_last_fit_) {
                best_a = last_fit_a_;
                best_b = last_fit_b_;
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "当前帧地面拟合点不足或失败。沿用历史地面参数: v = %.2f * z + %.2f", best_a,
                                     best_b);
            } else {
                best_a = -0.15f * height;
                best_b = 0.95f * height;
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "双视差地面拟合失败且无历史参数。使用默认经验地面线: v = %.2f * z + %.2f", best_a,
                                     best_b);
            }
        }

        // 剔除地面并统计 U-depth 直方图
        std::vector<std::vector<int>> U_depth(width, std::vector<int>(N_z, 0));

        struct TempPoint {
            float x, y, z;
            int u, z_idx;
        };
        std::vector<TempPoint> obstacle_candidates;
        obstacle_candidates.reserve(pixel_count / 6);

        const float v_margin = static_cast<float>(uv_disparity_v_margin_);

        for (int v = 0; v < height; v += 3) {
            for (int u = 0; u < width; u += 2) {
                const size_t idx = static_cast<size_t>(v) * width + u;
                uint16_t depth_raw_val = depth_ptr[idx];
                if (depth_raw_val == 0) continue;

                float z = static_cast<float>(depth_raw_val) / depth_scale;
                if (z < min_z || z > max_z) continue;

                float expected_ground_v = best_a * z + best_b;

                // 地面以上判定为障碍物候选
                if (v < expected_ground_v - v_margin) {
                    float x = (static_cast<float>(u) - cx) * z / fx;
                    float y = (static_cast<float>(v) - cy) * z / fy;

                    int z_idx = static_cast<int>((z - min_z) / dz);
                    if (z_idx >= 0 && z_idx < N_z) {
                        U_depth[u][z_idx]++;
                        obstacle_candidates.push_back({x, y, z, u, z_idx});
                    }
                }
            }
        }

        // U-disparity 去除孤立点噪点
        std::vector<float> buf_obstacles;
        buf_obstacles.reserve(obstacle_candidates.size() * 3);

        const int min_u_count = uv_disparity_min_u_count_;
        for (const auto& pt : obstacle_candidates) {
            if (U_depth[pt.u][pt.z_idx] >= min_u_count) {
                buf_obstacles.push_back(pt.x);
                buf_obstacles.push_back(pt.y);
                buf_obstacles.push_back(pt.z);
            }
        }

        std::string frame_id = depth_msg->header.frame_id.empty() ? "camera_frame" : depth_msg->header.frame_id;
        out_obstacle = createPointCloud2FromXYZBuffer(buf_obstacles, depth_msg->header, frame_id);
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
    double uv_disparity_min_z_ = 0.3;
    double uv_disparity_max_z_ = 3.0;
    double uv_disparity_dz_ = 0.02;
    int uv_disparity_v_margin_ = 15;
    int uv_disparity_min_u_count_ = 5;

    // 上一帧成功拟合的地面参数，用作下一次拟合的初始值及连续性保护
    double last_fit_a_ = 0.0;
    double last_fit_b_ = 0.0;
    bool has_last_fit_ = false;

    // 相机名称与 TF Yaw 角缓存（避免每帧查询 TF，区分 front/rear）
    std::string camera_name_ = "rear";
    double cached_camera_yaw_ = 0.0;
    bool has_cached_camera_yaw_ = false;

    // 堆内存重用缓冲区（避免高频分配/销毁导致内存碎片化与内存高波动）
    std::vector<float> buf_raw_;
    std::vector<float> buf_obstacles_;
};

int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdToPointCloud>());
    rclcpp::shutdown();
    return 0;
}

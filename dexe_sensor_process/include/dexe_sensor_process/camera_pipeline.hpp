#pragma once
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "dexe_sensor_process/rgbd_processing_state.hpp"
#include "dexe_sensor_process/hardware_obstacle_method.hpp"
#include "dexe_sensor_process/camera_extrinsics.hpp"

#include "dexe_sensor_process/obstacle_extractor.hpp"

namespace dexe_sensor_process {
class CameraPipeline {
    rclcpp::Node& node_;
    std::unique_ptr<ObstacleExtractor> extractor_;
public:
    CameraPipeline(rclcpp::Node& node, const std::string& camera_name,
                   const std::string& prefix, const std::string& method,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : node_(node), tf_buffer_(std::move(tf_buffer)), camera_name_(camera_name) {
        auto topic = [&](const std::string& name) {
            return node_.get_parameter(prefix + name).as_string();
        };
        std::string input_topic = topic("input_topic");
        std::string output_topic_raw = topic("output_topic_raw");
        std::string output_topic_filtered = topic("output_topic_filtered");
        std::string output_topic_laserscan = topic("output_topic_laserscan");
        obstacle_extraction_method_ = method;
        extractor_ = makeObstacleExtractor(method, node_);
        depth_max_range_ = node_.get_parameter("depth_max_range").as_double();
        if (!std::isfinite(depth_max_range_) || depth_max_range_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid depth_max_range: %.3f m; falling back to 3.0 m", depth_max_range_);
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
        flag_voxel_filter_ = node_.get_parameter("flag_voxel_filter").as_bool();
        flag_mls_filter_ = node_.get_parameter("flag_mls_filter").as_bool();
        flag_sor_filter_ = node_.get_parameter("flag_sor_filter").as_bool();
        ds_leaf_size_ = node_.get_parameter("ds_leaf_size").as_double();
        mls_polynomial_order_ = node_.get_parameter("mls_polynomial_order").as_int();
        mls_search_radius_ = node_.get_parameter("mls_search_radius").as_double();
        sor_mean_k_ = node_.get_parameter("sor_mean_k").as_int();
        sor_stddev_mul_thresh_ = node_.get_parameter("sor_stddev_mul_thresh").as_double();
        enable_laserscan_ = node_.get_parameter("enable_laserscan").as_bool();
        division_mode_ = node_.get_parameter("division_mode").as_bool();
        frame_division_state_.setEnabled(division_mode_);
        step_u_ = node_.get_parameter("step_u").as_int();
        step_v_ = node_.get_parameter("step_v").as_int();
        ground_filter_frame_ = node_.get_parameter("ground_filter_frame").as_string();
        ground_filter_min_height_ = node_.get_parameter("ground_filter_min_height").as_double();
        ground_filter_max_height_ = node_.get_parameter("ground_filter_max_height").as_double();

        // 同一相机的 CameraInfo/Depth 回调保持互斥；不同相机拥有不同 callback group，
        // 可由 MultiThreadedExecutor 并行处理且不会竞争 pipeline 内部状态。
        callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = callback_group_;

        // 创建原生 ROS 订阅（不再使用时间同步器）
        depth_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
            input_topic + "/depth", rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&CameraPipeline::depthCallback, this, std::placeholders::_1),
            subscription_options);

        camera_info_sub_ = node_.create_subscription<sensor_msgs::msg::CameraInfo>(
            input_topic + "/camera_info", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&CameraPipeline::cameraInfoCallback, this, std::placeholders::_1),
            subscription_options);

        // 发布原始点云
        const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        m_pointcloud_publisher_raw =
            node_.create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_raw, output_qos);

        // 发布预处理后的点云
        m_pointcloud_publisher_filtered =
            node_.create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_filtered, output_qos);

        // 发布压缩成 LaserScan 的点云
        m_laserscan_publisher_ =
            node_.create_publisher<sensor_msgs::msg::LaserScan>(output_topic_laserscan, output_qos);

        RCLCPP_INFO(get_logger(), "RGBD point cloud conversion node started, camera name: %s", camera_name.c_str());
        RCLCPP_INFO(get_logger(), "Input mode: Depth + CameraInfo reconstruction");
        RCLCPP_INFO(get_logger(), "Subscribing to depth topic: %s/depth", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "Subscribing to camera info topic: %s/camera_info", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "Publishing raw point cloud topic: %s", output_topic_raw.c_str());
        RCLCPP_INFO(get_logger(), "Publishing filtered point cloud topic: %s", output_topic_filtered.c_str());
        RCLCPP_INFO(get_logger(), "Point cloud max depth: %.3f m", depth_max_range_);
        RCLCPP_INFO(get_logger(), "Input frame decimation mode: %s",
                    division_mode_ ? "enabled (process one frame, skip one frame)" : "disabled");
        RCLCPP_INFO(get_logger(), "LaserScan topic: %s (published when enable_laserscan=true)", output_topic_laserscan.c_str());
        RCLCPP_INFO(get_logger(), "Preprocessing - downsample: %s, MLS: %s, statistical outlier removal: %s",
                    flag_voxel_filter_ ? "enabled" : "disabled", flag_mls_filter_ ? "enabled" : "disabled",
                    flag_sor_filter_ ? "enabled" : "disabled");
        RCLCPP_INFO(get_logger(), "Ground filter - reference frame: %s, min kept height: %.3f m, max kept height: %.3f m",
                    ground_filter_frame_.c_str(), ground_filter_min_height_, ground_filter_max_height_);
        RCLCPP_INFO(get_logger(),
                    "Ground filter pipeline: points are temporarily transformed into %s and filtered by z height; "
                    "the published cloud keeps the CameraInfo optical frame so the Nav2 costmap can do "
                    "marking/clearing correctly",
                    ground_filter_frame_.c_str());

        RCLCPP_INFO(get_logger(), "Obstacle extraction method: %s", obstacle_extraction_method_.c_str());
    }

private:
    friend class HeightFilterExtractor;
    friend class UVDisparityExtractor;
    rclcpp::Logger get_logger() const { return node_.get_logger().get_child(camera_name_); }
    rclcpp::Clock::SharedPtr get_clock() { return node_.get_clock(); }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
        // 相机内参固定：首次收到后缓存并销毁订阅，避免继续接收重复 CameraInfo。
        last_camera_info_ = camera_info_msg;
        RCLCPP_INFO(get_logger(),
                    "Cached CameraInfo and stopped subscribing; camera info frame: %s, fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f",
                    camera_info_msg->header.frame_id.c_str(), camera_info_msg->k[0], camera_info_msg->k[4],
                    camera_info_msg->k[2], camera_info_msg->k[5]);
        camera_info_sub_.reset();
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
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "CameraInfo not received yet; cannot convert depth image to point cloud");
            return;
        }

        int width = depth_msg->width;
        int height = depth_msg->height;

        // 基本有效性检查
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "Received invalid image size: %dx%d", width, height);
            return;
        }
        const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
        if (depth_msg->step == 0 || depth_msg->data.size() < expected_size) {
            RCLCPP_WARN(get_logger(), "Insufficient depth image data: %zu < %zu", depth_msg->data.size(), expected_size);
            return;
        }

        const bool need_projected_points =
            sub_count_raw > 0 || extractor_->requiresProjectedPoints();

        sensor_msgs::msg::PointCloud2::SharedPtr msg_raw;
        if (need_projected_points) {
            if (!computePointClouds(depth_msg, camera_info_msg, sub_count_raw > 0, msg_raw)) {
                RCLCPP_WARN(get_logger(), "Failed to convert depth image to raw point cloud");
                return;
            }
        }

        frame_valid_point_count_ = 0;
        auto msg_filtered = extractor_->extract(*this, depth_msg, camera_info_msg, msg_raw);

        // 本帧原始点云数据量：优先用已重建的原始点云点数；否则用提取器统计到的有效深度点数。
        const size_t raw_point_count = msg_raw
            ? static_cast<size_t>(msg_raw->width) * msg_raw->height
            : frame_valid_point_count_;

        // 如果订阅者需要原始点云且计算成功，则发布
        if (sub_count_raw > 0 && msg_raw) {
            m_pointcloud_publisher_raw->publish(*msg_raw);
        }

        if (!msg_filtered) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "Filtered point cloud not published for this frame: raw data has %zu points, "
                                 "but the filter extracted no obstacles, or the TF required for ground "
                                 "filtering is temporarily unavailable.", raw_point_count);
            return;
        }

        if (enable_laserscan_) {
            // 过滤后点云为空时：只要本帧原始深度仍有有效数据（看得到场景、只是没有障碍），
            // 就照常发布一帧 ranges 全 inf 的空 LaserScan；若原始数据也为空（遮挡/无有效
            // 深度），则不发布，交由下游按数据源中断处理，并打印告警。
            if (msg_filtered->width == 0 && raw_point_count == 0) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *this->get_clock(), 5000,
                    "No valid raw depth points in this frame (possible occlusion); "
                    "LaserScan is not published");
            } else {
                auto laser_scan_msg = pointCloudToLaserScanBaseLink(msg_filtered, tf_buffer_, get_logger());
                if (laser_scan_msg) {
                    m_laserscan_publisher_->publish(*laser_scan_msg);
                }
            }
        } else {
            m_pointcloud_publisher_filtered->publish(*msg_filtered);
        }

        const char* raw_frame = msg_raw ? msg_raw->header.frame_id.c_str() : "N/A";
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                             "Point cloud publish status - raw: %zu points, frame=%s; filtered: %zu points, frame=%s; "
                             "extraction method=%s, ground filter frame=%s, kept height range=(%.3f, %.3f] m",
                             raw_point_count, raw_frame,
                             static_cast<size_t>(msg_filtered->width) * msg_filtered->height,
                             msg_filtered->header.frame_id.c_str(), obstacle_extraction_method_.c_str(),
                             ground_filter_frame_.c_str(), ground_filter_min_height_, ground_filter_max_height_);

        if (raw_point_count > 0 && msg_filtered->width == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "Filtered point cloud is empty: raw data has %zu points but filtered has 0; "
                                 "publishing an all-inf LaserScan for this empty view. "
                                 "Possible causes: filter parameters are too strict, or there are no obstacle "
                                 "points satisfying the conditions in the current view.",
                                 raw_point_count);
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
            RCLCPP_INFO(get_logger(), "Cached full camera extrinsics (base_link <- %s), optical axis yaw: %.3f rad",
                        camera_frame.c_str(), camera_extrinsics_.opticalAxisYaw());
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                "Camera extrinsics not ready (base_link <- %s); skipping this frame and retrying: %s",
                                camera_frame.c_str(), ex.what());
            return false;
        }
    }

    sensor_msgs::msg::LaserScan::SharedPtr pointCloudToLaserScanBaseLink(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
        const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
        const rclcpp::Logger& logger) {
        // 空旷场景（视野内没有障碍）会得到 0 点的过滤点云，这仍是一帧有效的“无障碍”观测。
        // 必须照常发布全 inf 的 LaserScan，否则下游（collision_monitor、传感器监测）会把
        // “看不到障碍”误判成话题停更，进而报 “timestamps differ / Ignoring the source”，
        // 甚至按数据源失效停车。仅在点云消息本身无效时才放弃本帧。
        if (!cloud_msg) {
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
                RCLCPP_WARN(logger, "Failed to transform point cloud to base_link; cannot generate LaserScan: %s", ex.what());
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

    bool computePointClouds(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                            bool create_raw_message,
                            sensor_msgs::msg::PointCloud2::SharedPtr& out_raw) {
        const int width = depth_msg->width;
        const int height = depth_msg->height;
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "Invalid depth image size: %dx%d", width, height);
            return false;
        }

        const bool is_uint16_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                                     depth_msg->encoding == sensor_msgs::image_encodings::MONO16;
        const bool is_float32_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
        if (!is_uint16_depth && !is_float32_depth) {
            RCLCPP_WARN(get_logger(), "Unsupported depth image encoding: %s (only 16UC1/mono16 and 32FC1 are supported)",
                        depth_msg->encoding.c_str());
            return false;
        }
        if (depth_msg->is_bigendian) {
            RCLCPP_WARN(get_logger(), "Big-endian depth images are not supported yet");
            return false;
        }

        const size_t bytes_per_pixel = is_uint16_depth ? sizeof(uint16_t) : sizeof(float);
        const size_t minimum_step = static_cast<size_t>(width) * bytes_per_pixel;
        if (depth_msg->step < minimum_step) {
            RCLCPP_WARN(get_logger(), "Depth image row step too small: %u < %zu", depth_msg->step, minimum_step);
            return false;
        }
        const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
        if (depth_msg->data.size() < expected_size) {
            RCLCPP_WARN(get_logger(), "Insufficient depth data: %zu < %zu", depth_msg->data.size(), expected_size);
            return false;
        }

        const auto fx = camera_info_msg->k[0];
        const auto fy = camera_info_msg->k[4];
        const auto cx = camera_info_msg->k[2];
        const auto cy = camera_info_msg->k[5];
        if (fx <= 0.f || fy <= 0.f) {
            RCLCPP_WARN(get_logger(), "Invalid camera intrinsics fx=%.3f fy=%.3f", fx, fy);
            return false;
        }

        if (!depth_msg->header.frame_id.empty() && !camera_info_msg->header.frame_id.empty() &&
            depth_msg->header.frame_id != camera_info_msg->header.frame_id) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 50000,
                                 "Depth frame (%s) differs from the CameraInfo optical frame (%s); "
                                 "the reconstructed point cloud uses the CameraInfo frame",
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
        if (create_raw_message) {
            auto projection_header = depth_msg->header;
            // Projection coordinates are defined by CameraInfo intrinsics, so
            // advertise that optical frame even when the driver gives Depth a
            // different (but related) frame id.
            projection_header.frame_id = fallback_frame;
            out_raw = createPointCloud2FromXYZBuffer(buf_raw_, projection_header, fallback_frame);
        } else {
            out_raw.reset();
        }
        return true;
    }

    // 订阅器（原生 ROS 订阅）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // 首次收到后固定使用的相机参数
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;

    // 发布器与 TF 转换
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_raw;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_filtered;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laserscan_publisher_;

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
    // 相机名称与相机到 base_link 的完整静态外参缓存
    std::string camera_name_ = "rear";
    dexe_sensor_process::CameraExtrinsics camera_extrinsics_;

    // 堆内存重用缓冲区（避免高频分配/销毁导致内存碎片化与内存高波动）
    std::vector<float> buf_raw_;

    // 本帧统计到的有效原始深度数据量：高度法=buf_raw_ 点数，UV 视差=有效深度采样数。
    // 用于“过滤后点云为空时是否仍可发布全 inf 空 LaserScan”的判定。
    std::size_t frame_valid_point_count_ = 0;
};
}  // namespace dexe_sensor_process

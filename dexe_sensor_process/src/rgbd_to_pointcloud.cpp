#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <limits>
#include <cmath>
#include <tf2_ros/transform_listener.hpp>  



class RgbdToPointCloud : public rclcpp::Node {
public:
    RgbdToPointCloud() : Node("rgbd_to_pointcloud") {
        // 基础参数
        this->declare_parameter<std::string>("camera_name", "rear");
        this->declare_parameter<std::string>("input_topic", "");
        this->declare_parameter<std::string>("output_topic_raw", "");
        this->declare_parameter<std::string>("output_topic_filtered", "");
        this->declare_parameter<std::string>("output_topic_laserscan", "");
        
        // 预处理参数
        this->declare_parameter<bool>("flag_voxel_filter", true);
        this->declare_parameter<bool>("flag_mls_filter", true);
        this->declare_parameter<bool>("flag_sor_filter", true);
        this->declare_parameter<double>("ds_leaf_size", 0.05);
        this->declare_parameter<int>("mls_polynomial_order", 2);
        this->declare_parameter<double>("mls_search_radius", 0.2);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        this->declare_parameter<double>("ground_filter_threshold", 0.12);
        
        // （虚拟点云相关参数已移除）
        
        std::string camera_name = this->get_parameter("camera_name").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic_raw = this->get_parameter("output_topic_raw").as_string();
        std::string output_topic_filtered = this->get_parameter("output_topic_filtered").as_string();
        std::string output_topic_laserscan = this->get_parameter("output_topic_laserscan").as_string();
        
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
        ground_filter_threshold_ = this->get_parameter("ground_filter_threshold").as_double();
        

        
        // 创建原生 ROS 订阅（不再使用时间同步器）
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic + "/depth",
            rclcpp::SensorDataQoS(),
            std::bind(&RgbdToPointCloud::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            input_topic + "/camera_info",
            rclcpp::QoS(10),
            std::bind(&RgbdToPointCloud::cameraInfoCallback, this, std::placeholders::_1));
        
        // 发布原始点云
        m_pointcloud_publisher_raw = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_raw, 10);
        
        // 发布预处理后的点云
        m_pointcloud_publisher_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_filtered, 10);

        // 发布压缩成 LaserScan 的点云
        m_laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_topic_laserscan, 10);
                
        // TF buffer（Humble 需要 3 个参数）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock(),
            tf2::durationFromSec(10.0)
        );

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(get_logger(), "订阅深度图话题: %s/depth", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "订阅相机参数话题: %s/camera_info", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "发布原始点云话题: %s", output_topic_raw.c_str());
        RCLCPP_INFO(get_logger(), "发布预处理点云话题: %s", output_topic_filtered.c_str());
        RCLCPP_INFO(get_logger(), "发布 LaserScan 话题: %s", output_topic_laserscan.c_str());
        RCLCPP_INFO(get_logger(), "预处理参数 - 降采样: %s, MLS: %s, 统计滤波: %s",
                   flag_voxel_filter_ ? "启用" : "禁用",
                   flag_mls_filter_ ? "启用" : "禁用",
                   flag_sor_filter_ ? "启用" : "禁用");
    }

    ~RgbdToPointCloud() {}

private:
    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
        // 相机内参基本不变，这里只保存最新一帧的 CameraInfo 供深度回调使用
        last_camera_info_ = camera_info_msg;
    }

    void depthCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        // 使用最近一次收到的 CameraInfo
        auto camera_info_msg = last_camera_info_;
        if (!camera_info_msg) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *this->get_clock(), 2000,
                "尚未收到 CameraInfo，无法将深度图转换为点云");
            return;
        }

        int width = depth_msg->width;
        int height = depth_msg->height;

        // 从 CameraInfo 中提取相机内参
        auto fx = camera_info_msg->k[0];
        auto fy = camera_info_msg->k[4];
        auto cx = camera_info_msg->k[2];
        auto cy = camera_info_msg->k[5];

        // 基本有效性检查
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "收到的图像尺寸非法: %dx%d", width, height);
            return;
        }
        if (depth_msg->data.size() < static_cast<size_t>(width) * height * 2) {
            RCLCPP_WARN(get_logger(), "Depth 数据长度不足: %zu < %zu", 
                       depth_msg->data.size(), static_cast<size_t>(width) * height * 2);
            return;
        }

        // 计算 raw（不过滤地面）和 filtered（剔除地面）
        sensor_msgs::msg::PointCloud2::SharedPtr msg_raw;
        sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered;
        if (!computePointClouds(depth_msg, camera_info_msg, msg_raw)) {
            RCLCPP_WARN(get_logger(), "深度图转换点云失败");
            return;
        }
        
        // 对 msg_filtered 进行滤波处理：体素滤波 + Y坐标阈值过滤
        msg_filtered = filterPointCloud(msg_raw);
        
        // 发布 raw（未剔除地面）与 filter（剔除地面）
        m_pointcloud_publisher_raw->publish(*msg_raw);
        // m_pointcloud_publisher_filtered->publish(*msg_filtered);

        // // 将过滤后的点云压缩为 LaserScan 并发布
        // if (m_laserscan_publisher_) {
        //     auto scan_msg = pointCloudToLaserScanBaseLink(*msg_filtered);
        //     if (scan_msg) {
        //         m_laserscan_publisher_->publish(*scan_msg);
        //     }
        // }

        m_pointcloud_publisher_filtered->publish(*msg_filtered);

        // if (m_laserscan_publisher_) {
        //     auto scan_msg = pointCloudToLaserScanBaseLink(
        //         msg_filtered, tf_buffer_, this->get_logger());
        //     if (scan_msg) {
        //         m_laserscan_publisher_->publish(*scan_msg);
        //     }
        // }

    }

    double getCameraYawInBaseLink(
        const std::string & camera_frame,
        tf2_ros::Buffer & tf_buffer)
      {
        geometry_msgs::msg::Vector3Stamped v_cam, v_base;
        v_cam.header.frame_id = camera_frame;
        v_cam.header.stamp = rclcpp::Time(0);
      
        // 相机光轴（注意：如果你的相机是 Z 前，请改成 (0,0,1)）
        v_cam.vector.x = 0.0;
        v_cam.vector.y = 0.0;
        v_cam.vector.z = 1.0;
      
        // tf_buffer.transform(v_cam, v_base, "base_link");
        // 尝试获取变换，最多等待5秒
        try {
            // 先等待变换可用
            if (!tf_buffer.canTransform(
                "base_link", camera_frame, 
                rclcpp::Time(0), 
                rclcpp::Duration::from_seconds(5.0)))  // 等待5秒
            {
                RCLCPP_WARN(rclcpp::get_logger("rgbd_to_pointcloud"),
                    "等待 base_link -> %s 变换超时", camera_frame.c_str());
                return 0.0;  // 返回默认值或上一次的值
            }
            
            tf_buffer.transform(v_cam, v_base, "base_link");
            return std::atan2(v_base.vector.y, v_base.vector.x);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(rclcpp::get_logger("rgbd_to_pointcloud"),
                "无法获取变换: %s", ex.what());
            return 0.0;
        }
    }
      
    sensor_msgs::msg::LaserScan::SharedPtr
    pointCloudToLaserScanBaseLink(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger)
    {
        if (!cloud_msg || cloud_msg->width == 0 || cloud_msg->height == 0) {
            return nullptr;
        }
        double center_yaw = getCameraYawInBaseLink(
            cloud_msg->header.frame_id, *tf_buffer);
        // ================= 参数（写死） =================
        constexpr double DEG2RAD = M_PI / 180.0;
        constexpr double HFOV = 86.0 * DEG2RAD;

        constexpr double ANGLE_INCREMENT = 1.0 * DEG2RAD;

        constexpr double RANGE_MIN = 0.15;
        constexpr double RANGE_MAX = 4.0;

        constexpr double MIN_HEIGHT = -0.25;
        constexpr double MAX_HEIGHT =  1.7;

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
        scan->scan_time = 1.0 / 15.0;  // 与点云频率一致
        scan->range_min = RANGE_MIN;
        scan->range_max = RANGE_MAX;

        // ⚠️ 官方用 ceil，而不是 +1
        const uint32_t ranges_size = std::ceil(
            (scan->angle_max - scan->angle_min) / scan->angle_increment);

        if (USE_INF) {
            scan->ranges.assign(
            ranges_size, std::numeric_limits<float>::infinity());
        } else {
            scan->ranges.assign(
            ranges_size, scan->range_max + INF_EPSILON);
        }

        // ================= TF 到 base_link =================
        sensor_msgs::msg::PointCloud2 cloud_bl;
        if (cloud_msg->header.frame_id != scan->header.frame_id) {
            try {
            tf_buffer->transform(
                *cloud_msg,
                cloud_bl,
                scan->header.frame_id,
                tf2::durationFromSec(0.05));
            } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(logger, "TF transform failed: %s", ex.what());
            return nullptr;
            }
        } else {
            cloud_bl = *cloud_msg;
        }

        // ================= 遍历点云（官方 iterator 方式） =================
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_bl, "x"),
            iter_y(cloud_bl, "y"),
            iter_z(cloud_bl, "z");
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z)
        {
            if (!std::isfinite(*iter_x) ||
                !std::isfinite(*iter_y) ||
                !std::isfinite(*iter_z))
            {
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
            if (angle < scan->angle_min || angle > scan->angle_max) {
            continue;
            }

            const int index =
            static_cast<int>((angle - scan->angle_min) / scan->angle_increment);

            if (index >= 0 && index < static_cast<int>(ranges_size)) {
            if (range < scan->ranges[index]) {
                scan->ranges[index] = range;
            }
            }
        }

        return scan;
    }

    

    // 对点云进行滤波处理：体素滤波 + Y坐标阈值过滤
    sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr& input_msg) {
        if (!input_msg || input_msg->width == 0) {
            return input_msg;
        }

        // 转换为 PCL 格式
        pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*input_msg, *cloud_pcl);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_xyz);

        // 步骤1：体素滤波
        if (flag_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud_xyz);
            voxel_filter.setLeafSize(ds_leaf_size_, ds_leaf_size_, ds_leaf_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*cloud_filtered);
            cloud_xyz = cloud_filtered;
        }

        // 步骤2：在相机坐标系下，剔除 Y 坐标大于阈值的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_y_filtered->header = cloud_xyz->header;
        cloud_y_filtered->is_dense = cloud_xyz->is_dense;
        
        for (const auto& point : cloud_xyz->points) {
            // 只保留 Y 坐标小于等于阈值的点
            if (point.y <= static_cast<float>(ground_filter_threshold_)) {
                cloud_y_filtered->points.push_back(point);
            }
        }
        cloud_y_filtered->width = cloud_y_filtered->points.size();
        cloud_y_filtered->height = 1;

        // 转换回 ROS2 消息格式
        pcl::PCLPointCloud2 output_pcl;
        pcl::toPCLPointCloud2(*cloud_y_filtered, output_pcl);

        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(output_pcl, *output_msg);
        
        // 保持原始消息头
        output_msg->header = input_msg->header;
        
        return output_msg;
    }

    bool computePointClouds(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
        sensor_msgs::msg::PointCloud2::SharedPtr& out_raw) {
        const int width = depth_msg->width;
        const int height = depth_msg->height;
        if (width <= 0 || height <= 0) {
            RCLCPP_WARN(get_logger(), "深度图尺寸非法: %dx%d", width, height);
            return false;
        }

        constexpr float depth_scale = 1000.0f;
        constexpr float depth_trunc = 3.0f;

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

        std::vector<float> buf_raw;              // 不过滤地面
        std::vector<float> buf_filtered;         // 剔除地面
        buf_raw.reserve(pixel_count * 3 / 2);
        buf_filtered.reserve(pixel_count * 3 / 2);

        for (int v = 0; v < height; v += 3) {
            for (int u = 0; u < width; u += 2) {
                const size_t idx = static_cast<size_t>(v) * width + u;
                uint16_t depth_raw_val = depth_ptr[idx];
                if (depth_raw_val == 0) {
                    continue;
                }
                float z = static_cast<float>(depth_raw_val) / depth_scale;
                if (z <= 0.f || z > depth_trunc) {
                    continue;
                }
                float x = (static_cast<float>(u) - cx) * z / fx;
                float y = (static_cast<float>(v) - cy) * z / fy;

                // raw：不过滤地面
                buf_raw.push_back(x);
                buf_raw.push_back(y);
                buf_raw.push_back(z);

                // filtered：剔除地面
                // if (y <= static_cast<float>(ground_filter_threshold_)) {
                // buf_filtered.push_back(x);
                // buf_filtered.push_back(y);
                // buf_filtered.push_back(z);
                // }
            }
        }

        auto make_msg = [&](const std::vector<float>& xyz_buffer) -> sensor_msgs::msg::PointCloud2::SharedPtr {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            msg->header = depth_msg->header;
            msg->header.frame_id = msg->header.frame_id.empty() ? "camera_frame" : msg->header.frame_id;
            msg->header.stamp = depth_msg->header.stamp;
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
        };

        out_raw = make_msg(buf_raw);
        // out_filtered = make_msg(buf_filtered);
        return true;
    }



    // 订阅器（原生 ROS 订阅）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 最近一次收到的相机参数
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
    
    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_raw;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher_filtered;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laserscan_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 预处理参数
    bool flag_voxel_filter_ = true;
    bool flag_mls_filter_ = true;
    bool flag_sor_filter_ = true;
    double ds_leaf_size_ = 0.05;
    int mls_polynomial_order_ = 2;
    double mls_search_radius_ = 0.2;
    int sor_mean_k_ = 50;
    double sor_stddev_mul_thresh_ = 1.0;
    double ground_filter_threshold_ = 0.12;
    

};

int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdToPointCloud>());
    rclcpp::shutdown();
    return 0;
}


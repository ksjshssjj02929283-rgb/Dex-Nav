#define DEXE_SENSOR_PROCESS_NO_MAIN
#include "rgbd_to_pointcloud.cpp"
#include <gtest/gtest.h>
#include <chrono>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

using namespace std::chrono_literals;

class MultiCameraTest : public testing::TestWithParam<std::string> {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    bool waitFor(rclcpp::Executor& executor, const std::function<bool()>& ready) {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            if (ready()) return true;
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }
};

TEST_P(MultiCameraTest, InterleavedFramesKeepCameraStateIndependent) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("cameras", std::vector<std::string>{"front", "rear"}),
        rclcpp::Parameter("obstacle_extraction_method", GetParam()),
        rclcpp::Parameter("enable_laserscan", false),
        rclcpp::Parameter("flag_voxel_filter", false),
        rclcpp::Parameter("step_u", 1), rclcpp::Parameter("step_v", 1),
    });
    auto node = std::make_shared<RgbdToPointCloud>(options);
    ASSERT_EQ(node->cameraCount(), 2u);
    auto peer = std::make_shared<rclcpp::Node>("camera_test_peer");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(peer);

    struct CameraIO {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_sub, filtered_sub;
        std::vector<sensor_msgs::msg::PointCloud2> raw, filtered;
    };
    tf2_ros::StaticTransformBroadcaster broadcaster(peer);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto* frame : {"front_optical", "rear_optical"}) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = frame;
        transform.transform.rotation.w = 1.0;
        transforms.push_back(transform);
    }
    broadcaster.sendTransform(transforms);
    std::array<CameraIO, 2> io;
    for (size_t i = 0; i < io.size(); ++i) {
        auto& camera = io[i];
        const auto topic = std::string("/orbbec/") + (i == 0 ? "front" : "rear");
        camera.depth = peer->create_publisher<sensor_msgs::msg::Image>(topic + "/depth", rclcpp::SensorDataQoS());
        camera.info = peer->create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/camera_info", 10);
        camera.raw_sub = peer->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic + "/pointcloud2_raw", 10,
            [&, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { io[i].raw.push_back(*msg); });
        camera.filtered_sub = peer->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic + "/pointcloud2_filtered", 10,
            [&, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { io[i].filtered.push_back(*msg); });
    }
    ASSERT_TRUE(waitFor(executor, [&] {
        for (auto& camera : io) {
            if (!camera.depth->get_subscription_count() || !camera.info->get_subscription_count() ||
                !camera.raw_sub->get_publisher_count() || !camera.filtered_sub->get_publisher_count()) return false;
        }
        return true;
    }));
    for (size_t i = 0; i < io.size(); ++i) {
        sensor_msgs::msg::CameraInfo info;
        info.header.frame_id = i == 0 ? "front_optical" : "rear_optical";
        info.k = {100.0 + i * 100, 0, 0, 0, 100, 0, 0, 0, 1};
        io[i].info->publish(info);
    }
    // Drain camera-info delivery before sending images.
    const auto info_ready = std::chrono::steady_clock::now() + 100ms;
    ASSERT_TRUE(waitFor(executor, [&] { return std::chrono::steady_clock::now() > info_ready; }));
    for (int frame = 1; frame <= 3; ++frame) {
        for (size_t i = 0; i < io.size(); ++i) {
            sensor_msgs::msg::Image depth;
            depth.header.frame_id = i == 0 ? "front_optical" : "rear_optical";
            depth.header.stamp.sec = frame + static_cast<int>(i) * 10;
            depth.width = 30; depth.height = 30;
            depth.encoding = "32FC1";
            depth.step = depth.width * sizeof(float);
            depth.data.resize(depth.step * depth.height);
            const float z = 1.0f;
            for (size_t offset = 0; offset < depth.data.size(); offset += sizeof(float)) {
                std::memcpy(depth.data.data() + offset, &z, sizeof(z));
            }
            io[i].depth->publish(depth);
        }
        const size_t expected = frame == 3 ? 2 : 1;
        ASSERT_TRUE(waitFor(executor, [&] {
            return io[0].raw.size() >= expected && io[1].raw.size() >= expected &&
                   io[0].filtered.size() >= expected && io[1].filtered.size() >= expected;
        }));
    }
    for (size_t i = 0; i < io.size(); ++i) {
        ASSERT_EQ(io[i].raw.size(), 2u);
        ASSERT_EQ(io[i].filtered.size(), 2u);
        EXPECT_EQ(io[i].raw[0].header.stamp.sec, 1 + i * 10);
        EXPECT_EQ(io[i].raw[1].header.stamp.sec, 3 + i * 10);
        EXPECT_EQ(io[i].filtered[1].header.stamp.sec, 3 + i * 10);
        sensor_msgs::PointCloud2ConstIterator<float> x(io[i].raw[1], "x");
        ++x;
        EXPECT_NEAR(*x, 1.0 / (100.0 + i * 100), 1e-6);
        EXPECT_EQ(io[i].filtered[1].header.frame_id, i == 0 ? "front_optical" : "rear_optical");
        EXPECT_FALSE(io[i].filtered[1].fields.empty());
    }
    // Both extractors must work without constructing or publishing a raw cloud.
    for (auto& camera : io) camera.raw_sub.reset();
    ASSERT_TRUE(waitFor(executor, [&] {
        return node->count_subscribers("/orbbec/front/pointcloud2_raw") == 0 &&
               node->count_subscribers("/orbbec/rear/pointcloud2_raw") == 0;
    }));
    sensor_msgs::msg::Image depth;
    depth.header.frame_id = "base_link";
    depth.header.stamp.sec = 5;
    depth.width = 30; depth.height = 30;
    depth.encoding = "16UC1"; depth.step = 60;
    depth.data.resize(1800, 0);  // valid empty scene, exercises uint16 path
    for (auto& camera : io) camera.depth->publish(depth);  // skipped frame
    const auto skipped_frame_delivered = std::chrono::steady_clock::now() + 50ms;
    ASSERT_TRUE(waitFor(executor, [&] {
        return std::chrono::steady_clock::now() > skipped_frame_delivered;
    }));
    for (auto& camera : io) camera.depth->publish(depth);  // processed frame
    ASSERT_TRUE(waitFor(executor, [&] {
        return io[0].filtered.size() == 3 && io[1].filtered.size() == 3;
    }));
    if (GetParam() == "uv_disparity") {
        EXPECT_EQ(io[0].filtered.back().width, 0u);
        EXPECT_EQ(io[1].filtered.back().width, 0u);
    }
}

TEST_P(MultiCameraTest, RejectsDuplicateCamerasAndUnknownAlgorithm) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("cameras", std::vector<std::string>{"front", "front"}),
        rclcpp::Parameter("obstacle_extraction_method", GetParam()),
    });
    EXPECT_THROW(std::make_shared<RgbdToPointCloud>(options), std::invalid_argument);
    options.parameter_overrides({rclcpp::Parameter("obstacle_extraction_method", "unknown")});
    EXPECT_THROW(std::make_shared<RgbdToPointCloud>(options), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(Algorithms, MultiCameraTest,
    testing::Values("height_filter", "uv_disparity"));

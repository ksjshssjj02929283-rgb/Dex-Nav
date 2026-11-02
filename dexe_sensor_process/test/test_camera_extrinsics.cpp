#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "dexe_sensor_process/camera_extrinsics.hpp"

namespace {
geometry_msgs::msg::TransformStamped makeTransform() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "camera_optical";
    tf.header.stamp.sec = 1;
    // 绕 Y 轴旋转 90 度：(X,Y,Z) -> (Z,Y,-X)。
    tf.transform.rotation.y = std::sqrt(0.5);
    tf.transform.rotation.w = std::sqrt(0.5);
    tf.transform.translation.x = 10.0;
    tf.transform.translation.y = 20.0;
    tf.transform.translation.z = 30.0;
    return tf;
}

sensor_msgs::msg::PointCloud2 makeCloud() {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "camera_optical";
    cloud.header.stamp.sec = 123;
    cloud.header.stamp.nanosec = 456;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 1.0F;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = 2.0F;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 3.0F;
    return cloud;
}
}

TEST(CameraExtrinsics, AppliesFullRotationAndTranslationAndPreservesStamp) {
    dexe_sensor_process::CameraExtrinsics cache;
    cache.set(makeTransform());
    const auto cloud = makeCloud();
    sensor_msgs::msg::PointCloud2 output;
    cache.transformCloud(cloud, output);
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(output, "x"), 13.0F, 1e-5F);
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(output, "y"), 22.0F, 1e-5F);
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(output, "z"), 29.0F, 1e-5F);
    EXPECT_EQ(output.header.frame_id, "base_link");
    EXPECT_EQ(output.header.stamp, cloud.header.stamp);
    EXPECT_EQ(output.width, cloud.width);
    EXPECT_NEAR(cache.opticalAxisYaw(), 0.0, 1e-6);
}

TEST(CameraExtrinsics, RejectsUninitializedOrWrongSourceFrame) {
    dexe_sensor_process::CameraExtrinsics cache;
    auto cloud = makeCloud();
    sensor_msgs::msg::PointCloud2 output;
    EXPECT_FALSE(cache.matches(cloud.header.frame_id));
    EXPECT_THROW(cache.opticalAxisYaw(), std::bad_optional_access);
    EXPECT_THROW(cache.transformCloud(cloud, output), std::invalid_argument);
    cache.set(makeTransform());
    cloud.header.frame_id = "other_camera";
    EXPECT_FALSE(cache.matches(cloud.header.frame_id));
    EXPECT_THROW(cache.transformCloud(cloud, output), std::invalid_argument);
}

TEST(CameraExtrinsics, ReplacementUpdatesFrameAndOpticalAxis) {
    dexe_sensor_process::CameraExtrinsics cache;
    cache.set(makeTransform());
    auto replacement = makeTransform();
    replacement.child_frame_id = "other_camera";
    replacement.transform.rotation.y = 0.0;
    replacement.transform.rotation.x = -std::sqrt(0.5);
    cache.set(replacement);
    EXPECT_FALSE(cache.matches("camera_optical"));
    EXPECT_TRUE(cache.matches("other_camera"));
    EXPECT_NEAR(cache.opticalAxisYaw(), std::acos(-1.0) / 2.0, 1e-6);
}

TEST(CameraExtrinsics, ReusesCacheAcrossFilteringAndScanWithoutChangingSource) {
    dexe_sensor_process::CameraExtrinsics cache;
    cache.set(makeTransform());
    auto cloud = makeCloud();
    sensor_msgs::msg::PointCloud2 filtered_frame;
    cache.transformCloud(cloud, filtered_frame);

    // 模拟后续阶段/帧复用同一外参，不能复用旧点云时间或修改相机系输入。
    cloud.header.stamp.sec = 124;
    sensor_msgs::msg::PointCloud2 scan_frame;
    cache.transformCloud(cloud, scan_frame);
    EXPECT_EQ(scan_frame.data, filtered_frame.data);
    EXPECT_EQ(scan_frame.header.frame_id, "base_link");
    EXPECT_EQ(scan_frame.header.stamp, cloud.header.stamp);
    EXPECT_EQ(filtered_frame.header.stamp.sec, 123);
    EXPECT_EQ(cloud.header.frame_id, "camera_optical");
    EXPECT_FLOAT_EQ(*sensor_msgs::PointCloud2ConstIterator<float>(cloud, "x"), 1.0F);
    EXPECT_TRUE(cache.matches(cloud.header.frame_id));
}

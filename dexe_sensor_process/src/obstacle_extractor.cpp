#include "dexe_sensor_process/camera_pipeline.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

namespace dexe_sensor_process {
UVDisparityExtractor::UVDisparityExtractor(rclcpp::Node& node) {
    uv_disparity_min_z_ = node.get_parameter("uv_disparity_min_z").as_double();
    uv_disparity_max_z_ = node.get_parameter("uv_disparity_max_z").as_double();
    uv_disparity_dz_ = node.get_parameter("uv_disparity_dz").as_double();
    uv_disparity_v_margin_ = node.get_parameter("uv_disparity_v_margin").as_int();
    uv_disparity_min_u_count_ = node.get_parameter("uv_disparity_min_u_count").as_int();
    uv_disparity_ground_margin_h_ = node.get_parameter("uv_disparity_ground_margin_h").as_double();

}

std::unique_ptr<ObstacleExtractor> makeObstacleExtractor(
    const std::string& method, rclcpp::Node& node) {
    if (method == "height_filter") return std::make_unique<HeightFilterExtractor>();
    if (method == "uv_disparity") return std::make_unique<UVDisparityExtractor>(node);
    throw std::invalid_argument("Unknown obstacle extraction method: " + method);
}

sensor_msgs::msg::PointCloud2::SharedPtr HeightFilterExtractor::extract(
    CameraPipeline& camera, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
    const sensor_msgs::msg::PointCloud2::SharedPtr&) {
    const std::string source_frame = !camera_info_msg->header.frame_id.empty()
        ? camera_info_msg->header.frame_id : depth_msg->header.frame_id;
    if (camera.buf_raw_.empty()) {
        auto output_header = depth_msg->header;
        output_header.frame_id = source_frame;
        return camera.makeEmptyXYZPointCloud(output_header);
    }

    // Directly populate the reusable PCL cloud from the projection buffer. This
    // avoids constructing a raw ROS message and immediately converting it back.
    const size_t point_count = camera.buf_raw_.size() / 3;
    cloud_sensor_->clear();
    cloud_sensor_->points.resize(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        auto& point = cloud_sensor_->points[i];
        point.x = camera.buf_raw_[i * 3];
        point.y = camera.buf_raw_[i * 3 + 1];
        point.z = camera.buf_raw_[i * 3 + 2];
    }
    cloud_sensor_->width = point_count;
    cloud_sensor_->height = 1;
    cloud_sensor_->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_for_filter = cloud_sensor_;
    if (camera.flag_voxel_filter_) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_sensor_);
        voxel_filter.setLeafSize(camera.ds_leaf_size_, camera.ds_leaf_size_, camera.ds_leaf_size_);
        cloud_voxel_->clear();
        voxel_filter.filter(*cloud_voxel_);
        cloud_for_filter = cloud_voxel_;
    }

    const bool already_in_filter_frame = source_frame == camera.ground_filter_frame_;
    const bool use_cached_base_transform =
        !already_in_filter_frame && camera.ground_filter_frame_ == "base_link";
    tf2::Transform dynamic_transform;
    try {
        if (!already_in_filter_frame) {
            if (!rclcpp::ok()) {
                return nullptr;
            }
            if (use_cached_base_transform) {
                if (!camera.cacheCameraExtrinsics(source_frame, *camera.tf_buffer_)) {
                    return nullptr;
                }
            } else {
                const auto transform = camera.tf_buffer_->lookupTransform(
                    camera.ground_filter_frame_, source_frame,
                    rclcpp::Time(depth_msg->header.stamp), tf2::durationFromSec(0.05));
                tf2::fromMsg(transform.transform, dynamic_transform);
            }
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(camera.get_logger(), *camera.get_clock(), 2000,
                             "点云无法转换到 %s 做地面过滤，本帧不发布 filtered 点云: %s",
                             camera.ground_filter_frame_.c_str(), ex.what());
        return nullptr;
    }

    // Transform one point at a time only for the height decision. The output
    // remains in the optical frame, preserving the existing public contract.
    cloud_height_filtered_->clear();
    cloud_height_filtered_->points.reserve(cloud_for_filter->points.size());
    for (const auto& sensor_point : cloud_for_filter->points) {
        if (!std::isfinite(sensor_point.x) || !std::isfinite(sensor_point.y) ||
            !std::isfinite(sensor_point.z)) {
            continue;
        }
        tf2::Vector3 height_point(sensor_point.x, sensor_point.y, sensor_point.z);
        if (use_cached_base_transform) {
            height_point = camera.camera_extrinsics_.transformPoint(
                sensor_point.x, sensor_point.y, sensor_point.z);
        } else if (!already_in_filter_frame) {
            height_point = dynamic_transform * height_point;
        }
        if (height_point.z() > camera.ground_filter_min_height_ &&
            height_point.z() <= camera.ground_filter_max_height_) {
            cloud_height_filtered_->points.push_back(sensor_point);
        }
    }
    cloud_height_filtered_->width = cloud_height_filtered_->points.size();
    cloud_height_filtered_->height = 1;
    cloud_height_filtered_->is_dense = false;

    if (cloud_height_filtered_->points.empty()) {
        auto output_header = depth_msg->header;
        output_header.frame_id = source_frame;
        return camera.makeEmptyXYZPointCloud(output_header);
    }

    auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_height_filtered_, *output_msg);
    output_msg->header = depth_msg->header;
    output_msg->header.frame_id = source_frame;
    return output_msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr UVDisparityExtractor::extract(
    CameraPipeline& camera, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
    const sensor_msgs::msg::PointCloud2::SharedPtr&) {
    const int width = depth_msg->width;
    const int height = depth_msg->height;
    if (width <= 0 || height <= 0) {
        RCLCPP_WARN(camera.get_logger(), "深度图尺寸非法: %dx%d", width, height);
        return nullptr;
    }

    const auto fx = camera_info_msg->k[0];
    const auto fy = camera_info_msg->k[4];
    const auto cx = camera_info_msg->k[2];
    const auto cy = camera_info_msg->k[5];
    if (fx <= 0.f || fy <= 0.f) {
        RCLCPP_WARN(camera.get_logger(), "相机内参非法 fx=%.3f fy=%.3f", fx, fy);
        return nullptr;
    }

    const bool is_uint16_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                                 depth_msg->encoding == sensor_msgs::image_encodings::MONO16;
    const bool is_float32_depth = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    if (!is_uint16_depth && !is_float32_depth) {
        RCLCPP_WARN(camera.get_logger(), "双视差不支持深度图编码: %s", depth_msg->encoding.c_str());
        return nullptr;
    }
    if (depth_msg->is_bigendian) {
        RCLCPP_WARN(camera.get_logger(), "双视差暂不支持大端序深度图");
        return nullptr;
    }

    const size_t bytes_per_pixel = is_uint16_depth ? sizeof(uint16_t) : sizeof(float);
    const size_t minimum_step = static_cast<size_t>(width) * bytes_per_pixel;
    const size_t expected_size = static_cast<size_t>(depth_msg->step) * height;
    if (depth_msg->step < minimum_step || depth_msg->data.size() < expected_size) {
        RCLCPP_WARN(camera.get_logger(), "双视差深度数据布局非法: step=%u, 最小step=%zu, data=%zu, 期望=%zu",
                    depth_msg->step, minimum_step, depth_msg->data.size(), expected_size);
        return nullptr;
    }

    const size_t pixel_count = static_cast<size_t>(width) * height;

    const float min_z = static_cast<float>(uv_disparity_min_z_);
    const float max_z = static_cast<float>(uv_disparity_max_z_);
    const float dz = static_cast<float>(uv_disparity_dz_);
    if (dz <= 0.f) return nullptr;

    const int N_z = static_cast<int>((max_z - min_z) / dz);
    if (N_z <= 0) return nullptr;

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
        const bool enable_voxel = camera.flag_voxel_filter_ && camera.ds_leaf_size_ > 0.001;
        const float inv_leaf = enable_voxel ? static_cast<float>(1.0 / camera.ds_leaf_size_) : 1.0f;
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
    return camera.createPointCloud2FromXYZBuffer(buf_obstacles_, depth_msg->header, frame_id);
}



}  // namespace dexe_sensor_process

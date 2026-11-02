#include "path_follower/path_follower.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_follower
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;

        /// base_type 字符串 -> BaseType。
        /// "diff"->kDiff；"omni_translation"->kOmniTranslation；
        /// "omni"(兼容旧值)/"omni_trans_rotation"/其它 -> kOmniTransRotation。
        BaseType parseBaseType(const std::string& s)
        {
            if (s == "diff")
            {
                return BaseType::kDiff;
            }
            if (s == "omni_translation")
            {
                return BaseType::kOmniTranslation;
            }
            return BaseType::kOmniTransRotation;
        }

        const char* baseTypeToString(BaseType b)
        {
            switch (b)
            {
            case BaseType::kDiff: return "diff";
            case BaseType::kOmniTranslation: return "omni_translation";
            case BaseType::kOmniTransRotation: return "omni_trans_rotation";
            }
            return "unknown";
        }

        /// inputHeadingUsable 用：判定"朝向是否为零"的阈值。
        constexpr double kHeadingZeroEps = 1e-9;
        /// inputHeadingUsable 用：朝向与弦向夹角的容差（rad），超过即视为上游填错。
        constexpr double kHeadingConsistencyTol = 30.0 * M_PI / 180.0;
        /// inputHeadingUsable 用：一致性校验的抽样点数。
        constexpr std::size_t kHeadingCheckSamples = 16;

        /// 从四元数取偏航角（rad）。
        double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
        {
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            return tf2::getYaw(quat);
        }

        /// 钳位。
        inline double clamp(double v, double lo, double hi)
        {
            return std::max(lo, std::min(hi, v));
        }

        inline double applyNonzeroVelocityFloor(double v, double min_abs)
        {
            if (std::fabs(v) > kEpsilon && std::fabs(v) < min_abs)
            {
                return std::copysign(min_abs, v);
            }
            return v;
        }

        /// [实验功能] kOmniTransRotation 舵角限位可行性分析结果。
        /// 仅用于计算+打印，不参与任何控制/限速逻辑，独立于节点状态机。
        /// 约定：steer(t) = phi_path(t) - theta_heading(t)（车体系舵角，即世界系速度方向
        /// 旋转 -theta_heading 转到车体系下的角度；theta=0 时 steer=phi_path，符号与
        /// 世界系一致，比如车往右前方走 phi_path 为负，steer 也应为负）。
        struct SteerRangeAnalysisResult
        {
            bool ok{false};              // 是否完成计算（false：输入轨迹为空，未分析）
            double phi_min_deg{0.0};     // 位置轨迹切向 unwrap 后的最小值
            double phi_max_deg{0.0};     // 位置轨迹切向 unwrap 后的最大值
            double theta_min_deg{0.0};   // 朝向轨迹最小值
            double theta_max_deg{0.0};   // 朝向轨迹最大值
            double raw_min_deg{0.0};     // 最坏情况 steer 下界（未做360°分支平移）
            double raw_max_deg{0.0};     // 最坏情况 steer 上界（未做360°分支平移）
            double raw_steer0_deg{0.0};  // steer(0)（未做分支平移）
            double raw_steer_end_deg{0.0};  // steer(T)（未做分支平移）
            double span_deg{0.0};        // raw_max_deg - raw_min_deg
            bool feasible{false};        // 是否存在落在 ±steer_limit_deg 内的分支
            bool use_flip{false};        // true：选中分支为"反接"（+180°，需整体反转速度符号）
            bool both_branches_ok{false};  // 正接/反接两支都可行（此时按 |终点舵角| 择优）
            double branch_min_deg{0.0};  // 选中分支平移后的下界
            double branch_max_deg{0.0};  // 选中分支平移后的上界
            double theta0_deg{0.0};      // 选中分支下的起步舵角
            double steer_end_deg{0.0};   // 选中分支下的终点舵角
        };

        /// [实验功能] 计算 kOmniTransRotation 模式下、位置轨迹(position)与朝向轨迹(heading_align)
        /// 并行执行时，舵角(=路径切向-朝向)的理论最坏情况摆动范围，并判断是否存在正接/反接
        /// 分支能落在 ±steer_limit_deg 限位内。仅做计算，不影响调用方任何状态。
        SteerRangeAnalysisResult analyzeOmniSteerRange(
            const Route& position, const HeadingRoute& heading_align,
            double start_theta, double steer_limit_deg)
        {
            SteerRangeAnalysisResult res;
            if (position.points.empty() || heading_align.points.empty())
            {
                return res;
            }

            constexpr double kRad2Deg = 180.0 / M_PI;

            // 角度归约进 (-180,180]
            auto wrapDeg = [](double d) {
                double w = std::fmod(d + 180.0, 360.0);
                if (w < 0.0) w += 360.0;
                return w - 180.0;
            };
            // 把区间 [lo,hi] 平移 360° 整数倍，使区间中点落进 (-180,180] 附近
            auto branchShiftDeg = [](double lo, double hi) {
                const double mid = 0.5 * (lo + hi);
                return std::round(-mid / 360.0) * 360.0;
            };

            // 1) 位置切向 unwrap：heading_angle 逐点由 atan2 独立算出，未保证连续（见
            //    velocity_planner.cpp computeArcLengthAndHeading），需先展开再取 min/max，
            //    否则路径切向恰好摆过 ±180° 时会得到虚假的巨大跨度。
            std::vector<double> phi_unwrap_deg(position.points.size());
            phi_unwrap_deg[0] = position.points.front().heading_angle * kRad2Deg;
            for (std::size_t i = 1; i < position.points.size(); ++i)
            {
                const double d = angles::normalize_angle(
                    position.points[i].heading_angle - position.points[i - 1].heading_angle);
                phi_unwrap_deg[i] = phi_unwrap_deg[i - 1] + d * kRad2Deg;
            }
            res.phi_min_deg = *std::min_element(phi_unwrap_deg.begin(), phi_unwrap_deg.end());
            res.phi_max_deg = *std::max_element(phi_unwrap_deg.begin(), phi_unwrap_deg.end());

            // 2) 朝向轨迹：HeadingRoute.theta 已经是 unwrap 连续值（planHeading 生成时保证），
            //    直接扫 min/max 即可。
            double theta_min = heading_align.points.front().theta;
            double theta_max = theta_min;
            for (const auto& hp : heading_align.points)
            {
                theta_min = std::min(theta_min, hp.theta);
                theta_max = std::max(theta_max, hp.theta);
            }
            res.theta_min_deg = theta_min * kRad2Deg;
            res.theta_max_deg = theta_max * kRad2Deg;

            // 3) steer = phi_path - theta_heading（世界系速度方向转到车体系下的角度，
            //    theta=0 时 steer=phi_path，符号与世界系一致）：位置/朝向两条轨迹各自
            //    独立的梯形剖面，无法确定谁快谁慢、哪个时刻恰好对应哪个时刻，因此按
            //    最坏情况区间作差（Minkowski 差），保证覆盖真实可能出现的所有 steer(t) 取值。
            res.raw_min_deg = res.phi_min_deg - res.theta_max_deg;
            res.raw_max_deg = res.phi_max_deg - res.theta_min_deg;
            res.span_deg = res.raw_max_deg - res.raw_min_deg;

            // 4) 起步/终点舵角（未平移）：t=0 与 t=T 时刻 theta/phi 都是确定值（两条轨迹的
            //    首末点必然同时到达），不受最坏情况区间影响，不需要按区间估计。
            res.raw_steer0_deg = phi_unwrap_deg.front() - start_theta * kRad2Deg;
            res.raw_steer_end_deg =
                phi_unwrap_deg.back() - heading_align.points.back().theta * kRad2Deg;

            res.ok = true;

            // 5) 摆幅超过 2×限位，正接/反接分支都不可能同时落在窗口内，直接判不可行。
            if (res.span_deg > 2.0 * steer_limit_deg)
            {
                res.feasible = false;
                return res;
            }

            // 6) 分支A（正接，不翻转）与分支B（反接，+180°、该段速度整体取反）
            //    分别做 360° 归约后与 ±steer_limit_deg 比较。
            const double shift_a = branchShiftDeg(res.raw_min_deg, res.raw_max_deg);
            const double a_lo = res.raw_min_deg + shift_a;
            const double a_hi = res.raw_max_deg + shift_a;

            const double lo_b = res.raw_min_deg + 180.0;
            const double hi_b = res.raw_max_deg + 180.0;
            const double shift_b = branchShiftDeg(lo_b, hi_b);
            const double b_lo = lo_b + shift_b;
            const double b_hi = hi_b + shift_b;

            const bool a_ok = (a_lo >= -steer_limit_deg && a_hi <= steer_limit_deg);
            const bool b_ok = (b_lo >= -steer_limit_deg && b_hi <= steer_limit_deg);

            // 分支B是把整条 steer(t) 曲线先 +180°（反接）、再用 shift_b 归约得到的；
            // raw_steer0 / raw_steer_end 在同一条曲线上，必须做同样的 +180+shift_b 变换，
            // 否则算出来的还是"正接"那个角度，只是branch标记成了flip。
            const double a_theta0 = wrapDeg(res.raw_steer0_deg + shift_a);
            const double b_theta0 = wrapDeg(res.raw_steer0_deg + 180.0 + shift_b);
            const double a_steer_end = wrapDeg(res.raw_steer_end_deg + shift_a);
            const double b_steer_end = wrapDeg(res.raw_steer_end_deg + 180.0 + shift_b);

            // 7) 择支：只有一支可行时别无选择；两支都可行时选终点舵角绝对值更小的那支
            //    （末端舵角决定停车姿态与接续下一段时的限位余量；起步舵角反正都要靠
            //    /steering_angle_control 预摆过去，代价相同，不作为判据）。
            //    A/B 恒相差 180°，等价于选 |steer_end| < 90° 的那支；恰好相等（都是 90°）
            //    时保持正接优先，与改动前行为一致。
            res.both_branches_ok = (a_ok && b_ok);
            bool pick_b = false;
            if (a_ok && b_ok)
            {
                pick_b = std::fabs(b_steer_end) < std::fabs(a_steer_end);
            }
            else if (b_ok)
            {
                pick_b = true;
            }

            if (a_ok || b_ok)
            {
                res.feasible = true;
                res.use_flip = pick_b;
                res.branch_min_deg = pick_b ? b_lo : a_lo;
                res.branch_max_deg = pick_b ? b_hi : a_hi;
                res.theta0_deg = pick_b ? b_theta0 : a_theta0;
                res.steer_end_deg = pick_b ? b_steer_end : a_steer_end;
            }
            else
            {
                res.feasible = false;
            }

            return res;
        }
    } // namespace

    PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions& options)
        : nav2_util::LifecycleNode("path_follower_node", "", options)
    {
        declareAllParameters();

        // 时间戳成员必须用节点时钟初始化：默认构造的 rclcpp::Time 是 SYSTEM_TIME，
        // 与 get_clock()->now() 的时钟源不一致时相减会直接抛异常。
        last_obstacle_stop_time_ = get_clock()->now();
        last_obstacle_slowdown_time_ = last_obstacle_stop_time_;
        last_obstacle_status_time_ = last_obstacle_stop_time_;
        jump_stop_deadline_ = last_obstacle_stop_time_;
        last_map_odom_stamp_ = last_obstacle_stop_time_;
        last_odom_time_ = last_obstacle_stop_time_;
        last_odom_stamp_ = last_obstacle_stop_time_;
        odom_stale_stop_deadline_ = last_obstacle_stop_time_;

        velocity_planner_.setParams(planner_params_);
        applyPidConfig();

        RCLCPP_INFO(
            get_logger(),
            "path_follower constructed (unconfigured): base=%s rate=%.1fHz tracking_num=%zu "
            "resolution=%.4fm",
            baseTypeToString(base_type_),
            control_rate_, tracking_num_, trajectory_resolution_);
    }

    // ---------------------------------------------------------------------------
    // 生命周期
    // ---------------------------------------------------------------------------
    nav2_util::CallbackReturn PathFollowerNode::on_configure(const rclcpp_lifecycle::State&)
    {
        // 参数已在构造函数里声明并读好（declare 只能做一次）。这里只建接口，
        // **控制 timer 刻意不启动**——inactive 不允许出任何速度。
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, rclcpp::QoS(1),
            std::bind(&PathFollowerNode::onPath, this, std::placeholders::_1));
        tracking_path_sub_ = create_subscription<nav_interfaces::msg::TrackingPath>(
            tracking_path_topic_, rclcpp::QoS(1),
            std::bind(&PathFollowerNode::onTrackingPath, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(10),
            std::bind(&PathFollowerNode::onOdom, this, std::placeholders::_1));
        obstacle_status_sub_ = create_subscription<nav_interfaces::msg::ObstacleMonitorStatus>(
            obstacle_status_topic_, rclcpp::QoS(10),
            std::bind(&PathFollowerNode::onObstacleStatus, this, std::placeholders::_1));
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
        debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic_, 1);
        navigation_status_pub_ = create_publisher<nav_interfaces::msg::NavigationStatus>(
            navigation_status_topic_, rclcpp::QoS(10));
        stop_service_ = create_service<std_srvs::srv::Trigger>(
            stop_service_name_,
            std::bind(&PathFollowerNode::onStopService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // [实验功能] 舵角限位分析用：试调用 mobile_cmd_omni 的 /steering_angle_control action。
        // client 挂在独立的 steer_helper_node_ 上而非 this，原因见头文件里该成员的注释。
        steer_helper_node_ = std::make_shared<rclcpp::Node>(
            std::string(get_name()) + "_steer_helper", get_namespace());
        steering_angle_control_client_ = rclcpp_action::create_client<nav_interfaces::action::SteeringAngleControl>(
            steer_helper_node_, "/steering_angle_control");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        param_callback_ = add_on_set_parameters_callback(
            std::bind(&PathFollowerNode::onParametersChanged, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Configured: cmd_vel=%s status=%s (control loop stopped until active)",
            cmd_vel_topic_.c_str(), navigation_status_topic_.c_str());
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PathFollowerNode::on_activate(const rclcpp_lifecycle::State&)
    {
        cmd_vel_pub_->on_activate();
        debug_path_pub_->on_activate();
        navigation_status_pub_->on_activate();

        // 失活期间可能积了半条旧路线/旧速度记忆，激活时一律从干净状态起步：
        // 恢复后不继承旧 goal、旧路线、旧索引
        resetRouteState();
        active_.store(true);
        startControlTimer();

        createBond();
        RCLCPP_INFO(get_logger(), "Activated: control loop running at %.1f Hz", control_rate_);
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PathFollowerNode::on_deactivate(const rclcpp_lifecycle::State&)
    {
        // 顺序是硬要求：先关门停 timer -> 清路线 -> 趁 publisher 还 active 把零速和 IDLE
        // 发出去 -> 最后才失活 publisher。反过来的话零速会被 LifecyclePublisher 悄悄丢掉，
        // 车带着最后一条非零速度继续走（下游 smoother 的 timeout 才会兜住，但那是好几百 ms）。
        active_.store(false);
        stopControlTimer();
        resetRouteState();
        publishCmdVel(Velocity{});
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE,
                                "path_follower deactivated");

        cmd_vel_pub_->on_deactivate();
        debug_path_pub_->on_deactivate();
        navigation_status_pub_->on_deactivate();

        destroyBond();
        RCLCPP_INFO(get_logger(), "Deactivated: control loop stopped, zero velocity published");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PathFollowerNode::on_cleanup(const rclcpp_lifecycle::State&)
    {
        stopControlTimer();
        param_callback_.reset();
        stop_service_.reset();
        navigation_status_pub_.reset();
        debug_path_pub_.reset();
        cmd_vel_pub_.reset();
        obstacle_status_sub_.reset();
        odom_sub_.reset();
        tracking_path_sub_.reset();
        path_sub_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        steering_angle_control_client_.reset();
        steer_helper_node_.reset();
        RCLCPP_INFO(get_logger(), "Cleaned up");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PathFollowerNode::on_shutdown(const rclcpp_lifecycle::State&)
    {
        // 尽力而为：先把车停住，再让它关
        active_.store(false);
        stopControlTimer();
        if (cmd_vel_pub_ && cmd_vel_pub_->is_activated())
        {
            publishCmdVel(Velocity{});
        }
        RCLCPP_INFO(get_logger(), "Shutting down");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    void PathFollowerNode::startControlTimer()
    {
        stopControlTimer();
        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / control_rate_));
        control_timer_ = create_wall_timer(
            period_ns, std::bind(&PathFollowerNode::update, this));
        have_control_update_time_ = false;
    }

    void PathFollowerNode::stopControlTimer()
    {
        if (control_timer_)
        {
            control_timer_->cancel();
            control_timer_.reset();
        }
    }

    // ---------------------------------------------------------------------------
    // 参数
    // ---------------------------------------------------------------------------
    void PathFollowerNode::declareAllParameters()
    {
        // 基础 / 周期 / 底盘
        const std::string base = declare_parameter<std::string>("base_type", "omni");
        base_type_ = parseBaseType(base);
        control_rate_ = declare_parameter<double>("control_rate", 30.0);
        update_timeout_ = declare_parameter<double>("update_timeout", 1.0);

        // 话题 / 坐标系
        path_topic_ = declare_parameter<std::string>("path_topic", "~/path");
        tracking_path_topic_ = declare_parameter<std::string>("tracking_path_topic", "~/tracking_path");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "~/odom");
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "~/cmd_vel");
        debug_path_topic_ = declare_parameter<std::string>("debug_path_topic", "~/debug/reference_path");
        navigation_status_topic_ = declare_parameter<std::string>("navigation_status_topic", "~/navigation_status");
        obstacle_status_topic_ = declare_parameter<std::string>("obstacle_status_topic", "/obstacle_status");
        stop_service_name_ = declare_parameter<std::string>("stop_service", "~/stop");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

        // 轨迹分辨率与前视
        trajectory_resolution_ = declare_parameter<double>("trajectory_resolution", 0.01);
        tracking_num_ = static_cast<std::size_t>(declare_parameter<int>("tracking_num", 5));
        max_search_number_ = static_cast<std::size_t>(declare_parameter<int>("max_search_number", 100));

        // 到点判定
        reach_distance_ = declare_parameter<double>("reach_distance", 0.01);
        reach_angle_ = declare_parameter<double>("reach_angle", 0.03);
        end_extended_distance_ = declare_parameter<double>("end_extended_distance", 0.01);
        heading_back_window_ = static_cast<std::size_t>(
            declare_parameter<int>("heading_back_window", 8));
        enable_heading_lock_ = declare_parameter<bool>("enable_heading_lock", false);
        heading_lock_distance_ = declare_parameter<double>("heading_lock_distance", 0.5);

        // 路径截断：接收路径时若车已在路径中段，剪掉车之前的历史段，从最近点开始跟踪。
        enable_path_truncate_ = declare_parameter<bool>("enable_path_truncate", true);

        // 跟踪偏差保护：跟踪过程中车到最近点距离超阈值则立即报失败、发零速停止。
        enable_track_deviation_check_ = declare_parameter<bool>("enable_track_deviation_check", true);
        track_deviation_limit_ = declare_parameter<double>("track_deviation_limit", 0.1);
        enable_reverse_ = declare_parameter<bool>("enable_reverse", false);

        // [实验功能] kOmniTransRotation 舵角限位可行性分析：不可行时报导航失败并终止本次移动。
        enable_steer_range_analysis_ = declare_parameter<bool>("enable_steer_range_analysis", true);
        experimental_steer_limit_deg_ = declare_parameter<double>("experimental_steer_limit_deg", 135.0);

        // 末端定位切换：距目标点剩余距离/角度 <= 阈值时 map(TF)->odom 锚定，消除到点阶段定位抖动；0=关闭。
        end_localization_switch_distance_ = declare_parameter<double>("end_localization_switch_distance", 0.1);
        end_localization_switch_angle_ = declare_parameter<double>("end_localization_switch_angle", 0.15);

        // 定位跳变兜底：map->odom 变化投影到车当前位置超阈值时判跳变，改用 odom 递推位姿供控制使用。
        enable_localization_jump_recovery_ =
            declare_parameter<bool>("enable_localization_jump_recovery", true);
        localization_jump_threshold_ =
            declare_parameter<double>("localization_jump_threshold", 0.07);
        localization_jump_recover_scale_ =
            declare_parameter<double>("localization_jump_recover_scale", 2.0);
        localization_jump_stop_wait_duration_ =
            declare_parameter<double>("localization_jump_stop_wait_duration", 30.0);
        map_to_odom_timeout_ = declare_parameter<double>("map_to_odom_timeout", 1.0);
        max_tf_fallback_frames_ = declare_parameter<int>("max_tf_fallback_frames", 5);
        tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.005);

        // odom 接收看门狗（DDS 阻塞检测）
        odom_stale_stop_timeout_ = declare_parameter<double>("odom_stale_stop_timeout", 0.2);
        odom_stale_stop_hold_duration_ =
            declare_parameter<double>("odom_stale_stop_hold_duration", 5.0);
        odom_gap_log_threshold_ = declare_parameter<double>("odom_gap_log_threshold", 0.2);

        // 速度规划（Ruckig jerk 限制曲线，路径下发时一次性整段规划）
        planner_params_.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 1.2);
        planner_params_.max_linear_acceleration = declare_parameter<double>("max_linear_acceleration", 0.2);
        planner_params_.max_linear_deceleration = declare_parameter<double>("max_linear_deceleration", 0.2);
        planner_params_.max_linear_jerk = declare_parameter<double>("max_linear_jerk", 1.0);
        planner_params_.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 0.8);
        planner_params_.max_angular_acceleration = declare_parameter<double>("max_angular_acceleration", 0.35);
        planner_params_.max_angular_deceleration = declare_parameter<double>("max_angular_deceleration", 0.35);
        planner_params_.max_angular_jerk = declare_parameter<double>("max_angular_jerk", 1.0);
        // min_*：既是规划器起点/终点速度（覆盖底盘死区），也是发布层等比抬升的阈值。
        planner_params_.min_linear_velocity = declare_parameter<double>("min_linear_velocity", 0.02);
        planner_params_.min_angular_velocity = declare_parameter<double>("min_angular_velocity", 0.02);
        planner_params_.ruckig_control_dt = declare_parameter<double>("ruckig_control_dt", 0.02);
        planner_params_.curvature_deceleration = declare_parameter<bool>("curvature_deceleration", true);
        planner_params_.curvature_dec_gain = declare_parameter<double>("curvature_dec_gain", 0.3);
        planner_params_.curvature_dec_ratio_min = declare_parameter<double>("curvature_dec_ratio_min", 0.2);
        planner_params_.acceleration_envelope = declare_parameter<bool>("acceleration_envelope", true);
        planner_params_.velocity_profile_mode =
            declare_parameter<std::string>("velocity_profile_mode", "legacy");
        planner_params_.angular_accel_margin =
            declare_parameter<double>("angular_accel_margin", 0.8);
        planner_params_.angle_resolution = declare_parameter<double>("angle_resolution", 0.0034906585);
        planner_params_.trajectory_resolution = trajectory_resolution_;

        // 障碍减速/停止/恢复的两阶段步进模型（见 velocity_step_model.hpp）。
        // jerk 不单独配，由 jerk = 最大减速度 / 加速度衰减时间 派生。
        linear_accel_decay_time_ = declare_parameter<double>("linear_accel_decay_time", 0.3);
        angular_accel_decay_time_ = declare_parameter<double>("angular_accel_decay_time", 0.3);
        resume_converge_linear_velocity_ =
            declare_parameter<double>("resume_converge_linear_velocity", 0.02);
        slowdown_decel_scale_ = declare_parameter<double>("slowdown_decel_scale", 1.5);
        stop_decel_scale_ = declare_parameter<double>("stop_decel_scale", 2.0);

        // 障碍状态看门狗：超过该时长没收到 obstacle_status 就停车报失败；<=0 关闭。
        obstacle_status_timeout_ = declare_parameter<double>("obstacle_status_timeout", 1.0);

        // 10 级速度分档
        const auto levels = declare_parameter<std::vector<double>>(
            "vel_levels",
            std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0});
        for (std::size_t i = 0; i < kGainLevelCount && i < levels.size(); ++i)
        {
            vel_levels_[i] = levels[i];
        }

        // PID 增益（x/y/theta，各 10 级数组）
        const auto def = std::vector<double>(kGainLevelCount, 1.0);
        auto read_gain = [&](const std::string& name, std::array<double, kGainLevelCount>& arr)
        {
            const auto v = declare_parameter<std::vector<double>>(name, def);
            for (std::size_t i = 0; i < kGainLevelCount && i < v.size(); ++i) arr[i] = v[i];
        };
        read_gain("kp_x", x_gains_.kp);
        read_gain("ki_x", x_gains_.ki);
        read_gain("kd_x", x_gains_.kd);
        read_gain("kp_y", y_gains_.kp);
        read_gain("ki_y", y_gains_.ki);
        read_gain("kd_y", y_gains_.kd);
        read_gain("kp_theta", theta_gains_.kp);
        read_gain("ki_theta", theta_gains_.ki);
        read_gain("kd_theta", theta_gains_.kd);

        // 纠偏限幅
        max_correct_vy_ = declare_parameter<double>("max_correct_vy", 0.4);
        max_correct_w_ = declare_parameter<double>("max_correct_w", 0.3);
        max_percentage_vy_to_vx_ = declare_parameter<double>("max_percentage_vy_to_vx", 0.2);
        diff_lateral_gain_ = declare_parameter<double>("diff_lateral_gain", 1.5);
        kp_heading_ = declare_parameter<double>("kp_heading", 1.0);
    }

    rcl_interfaces::msg::SetParametersResult PathFollowerNode::onParametersChanged(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // 参数校验：增益数组长度必须为 10。
        const auto check_size = [&](const std::string& name, const rclcpp::Parameter& p)
        {
            if (p.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY)
            {
                if (p.as_double_array().size() != kGainLevelCount)
                {
                    result.successful = false;
                    result.reason = name + " must have exactly " + std::to_string(kGainLevelCount) + " elements";
                }
            }
        };

        bool gains_changed = false;
        bool planner_changed = false;
        for (const auto& p : parameters)
        {
            check_size(p.get_name(), p);
            if (p.get_name().find("kp_") != std::string::npos ||
                p.get_name().find("ki_") != std::string::npos ||
                p.get_name().find("kd_") != std::string::npos)
            {
                gains_changed = true;
            }
            if (p.get_name().find("max_linear") != std::string::npos ||
                p.get_name().find("max_angular") != std::string::npos ||
                p.get_name().find("min_linear") != std::string::npos ||
                p.get_name().find("min_angular") != std::string::npos ||
                p.get_name() == "ruckig_control_dt" ||
                p.get_name() == "curvature_deceleration" ||
                p.get_name() == "curvature_dec_gain" ||
                p.get_name() == "curvature_dec_ratio_min" ||
                p.get_name() == "acceleration_envelope" ||
                p.get_name() == "velocity_profile_mode" ||
                p.get_name() == "angular_accel_margin" ||
                p.get_name() == "trajectory_resolution" ||
                p.get_name() == "angle_resolution")
            {
                planner_changed = true;
            }
            if ((p.get_name() == "heading_lock_distance" ||
                 p.get_name() == "end_localization_switch_distance" ||
                 p.get_name() == "end_localization_switch_angle") &&
                p.as_double() < 0.0)
            {
                result.successful = false;
                result.reason = p.get_name() + " must be >= 0";
            }
            if (p.get_name() == "experimental_steer_limit_deg" &&
                (p.as_double() <= 0.0 || p.as_double() > 180.0))
            {
                result.successful = false;
                result.reason = p.get_name() + " must be in (0, 180]";
            }
            // 步进模型：衰减时间是 jerk = a_max/T 的分母，倍数是减速度的乘子，都必须为正。
            if ((p.get_name() == "linear_accel_decay_time" ||
                 p.get_name() == "angular_accel_decay_time" ||
                 p.get_name() == "slowdown_decel_scale" ||
                 p.get_name() == "stop_decel_scale") &&
                p.as_double() <= 0.0)
            {
                result.successful = false;
                result.reason = p.get_name() + " must be > 0";
            }
            if (p.get_name() == "resume_converge_linear_velocity" && p.as_double() < 0.0)
            {
                result.successful = false;
                result.reason = p.get_name() + " must be >= 0";
            }
            if (p.get_name() == "velocity_profile_mode" &&
                p.as_string() != "legacy" && p.as_string() != "toppra")
            {
                result.successful = false;
                result.reason = p.get_name() + " must be \"legacy\" or \"toppra\"";
            }
            // 角加速度余量：<=0 会把约束压成 0（车停住），>1 等于放大上限，都无意义。
            if (p.get_name() == "angular_accel_margin" &&
                (p.as_double() <= 0.0 || p.as_double() > 1.0))
            {
                result.successful = false;
                result.reason = p.get_name() + " must be in (0, 1]";
            }
            // 曲率减速软钳：gain 是分母加数，必须为正；ratio_min 超过 1 会把速度放大到
            // Ruckig 规划值之上（越过全局速度上限），必须夹在 (0, 1]。
            if (p.get_name() == "curvature_dec_gain" && p.as_double() <= 0.0)
            {
                result.successful = false;
                result.reason = p.get_name() + " must be > 0";
            }
            if (p.get_name() == "curvature_dec_ratio_min" &&
                (p.as_double() <= 0.0 || p.as_double() > 1.0))
            {
                result.successful = false;
                result.reason = p.get_name() + " must be in (0, 1]";
            }
            // 障碍状态看门狗超时：<=0 表示关闭，允许；只拒绝 NaN。
            if (p.get_name() == "obstacle_status_timeout" && std::isnan(p.as_double()))
            {
                result.successful = false;
                result.reason = p.get_name() + " must be a finite number";
            }
        }
        if (!result.successful)
        {
            return result;
        }

        // 即时应用：ROS 参数在回调时已写入，此处重新同步本地缓存。
        for (const auto& p : parameters)
        {
            if (p.get_name() == "base_type")
            {
                base_type_ = parseBaseType(p.as_string());
            }
            if (p.get_name() == "control_rate")
            {
                control_rate_ = p.as_double();
                // 只有 active 时才重建 timer：inactive 期间改这个参数不能把控制环复活
                // （新频率已经记进 control_rate_，下次 activate 自然生效）
                if (active_.load())
                {
                    startControlTimer();
                }
            }
            if (p.get_name() == "trajectory_resolution")
            {
                planner_params_.trajectory_resolution = p.as_double();
                trajectory_resolution_ = p.as_double();
                planner_changed = true;
            }
            if (p.get_name() == "heading_back_window")
            {
                heading_back_window_ = static_cast<std::size_t>(p.as_int());
            }
            if (p.get_name() == "enable_heading_lock")
            {
                enable_heading_lock_ = p.as_bool();
            }
            if (p.get_name() == "heading_lock_distance")
            {
                heading_lock_distance_ = p.as_double();
            }
            if (p.get_name() == "enable_path_truncate")
            {
                enable_path_truncate_ = p.as_bool();
            }
            if (p.get_name() == "enable_track_deviation_check")
            {
                enable_track_deviation_check_ = p.as_bool();
            }
            if (p.get_name() == "enable_reverse")
            {
                enable_reverse_ = p.as_bool();
            }
            if (p.get_name() == "track_deviation_limit")
            {
                track_deviation_limit_ = p.as_double();
            }
            if (p.get_name() == "enable_steer_range_analysis")
            {
                enable_steer_range_analysis_ = p.as_bool();
            }
            if (p.get_name() == "experimental_steer_limit_deg")
            {
                experimental_steer_limit_deg_ = p.as_double();
            }
        }

        // add_on_set_parameters_callback 在参数真正提交**之前**被调用（它的职责是校验），
        // 此时 get_parameter() 拿到的仍然是旧值。直接拿它刷新缓存会让每次在线调参都滞后
        // 一拍：本次改动读不到，要等下一次任意参数变更才生效（实测 param set 把阈值改了，
        // 节点里用的还是上上次的值）。所以优先从本次传入的 parameters 里取新值。
        const auto pval = [&parameters, this](const std::string& name) -> rclcpp::Parameter
        {
            for (const auto& p : parameters)
            {
                if (p.get_name() == name)
                {
                    return p;
                }
            }
            return get_parameter(name);
        };

        // 重新加载增益表与规划参数
        auto read_gain = [&](const std::string& name, std::array<double, kGainLevelCount>& arr)
        {
            const auto v = pval(name).as_double_array();
            for (std::size_t i = 0; i < kGainLevelCount && i < v.size(); ++i) arr[i] = v[i];
        };
        read_gain("kp_x", x_gains_.kp);
        read_gain("ki_x", x_gains_.ki);
        read_gain("kd_x", x_gains_.kd);
        read_gain("kp_y", y_gains_.kp);
        read_gain("ki_y", y_gains_.ki);
        read_gain("kd_y", y_gains_.kd);
        read_gain("kp_theta", theta_gains_.kp);
        read_gain("ki_theta", theta_gains_.ki);
        read_gain("kd_theta", theta_gains_.kd);
        max_correct_vy_ = pval("max_correct_vy").as_double();
        max_correct_w_ = pval("max_correct_w").as_double();
        max_percentage_vy_to_vx_ = pval("max_percentage_vy_to_vx").as_double();
        diff_lateral_gain_ = pval("diff_lateral_gain").as_double();
        kp_heading_ = pval("kp_heading").as_double();
        enable_heading_lock_ = pval("enable_heading_lock").as_bool();
        heading_lock_distance_ = pval("heading_lock_distance").as_double();
        enable_path_truncate_ = pval("enable_path_truncate").as_bool();
        enable_track_deviation_check_ = pval("enable_track_deviation_check").as_bool();
        track_deviation_limit_ = pval("track_deviation_limit").as_double();
        enable_reverse_ = pval("enable_reverse").as_bool();
        enable_steer_range_analysis_ = pval("enable_steer_range_analysis").as_bool();
        experimental_steer_limit_deg_ = pval("experimental_steer_limit_deg").as_double();
        end_localization_switch_distance_ = pval("end_localization_switch_distance").as_double();
        end_localization_switch_angle_ = pval("end_localization_switch_angle").as_double();
        enable_localization_jump_recovery_ = pval("enable_localization_jump_recovery").as_bool();
        localization_jump_threshold_ = pval("localization_jump_threshold").as_double();
        localization_jump_recover_scale_ = pval("localization_jump_recover_scale").as_double();
        localization_jump_stop_wait_duration_ =
            pval("localization_jump_stop_wait_duration").as_double();
        map_to_odom_timeout_ = pval("map_to_odom_timeout").as_double();
        max_tf_fallback_frames_ = pval("max_tf_fallback_frames").as_int();
        tf_lookup_timeout_ = pval("tf_lookup_timeout").as_double();
        odom_stale_stop_timeout_ = pval("odom_stale_stop_timeout").as_double();
        odom_stale_stop_hold_duration_ = pval("odom_stale_stop_hold_duration").as_double();
        odom_gap_log_threshold_ = pval("odom_gap_log_threshold").as_double();
        linear_accel_decay_time_ = pval("linear_accel_decay_time").as_double();
        angular_accel_decay_time_ = pval("angular_accel_decay_time").as_double();
        resume_converge_linear_velocity_ = pval("resume_converge_linear_velocity").as_double();
        slowdown_decel_scale_ = pval("slowdown_decel_scale").as_double();
        stop_decel_scale_ = pval("stop_decel_scale").as_double();
        obstacle_status_timeout_ = pval("obstacle_status_timeout").as_double();

        const auto levels = pval("vel_levels").as_double_array();
        for (std::size_t i = 0; i < kGainLevelCount && i < levels.size(); ++i) vel_levels_[i] = levels[i];

        applyPidConfig();
        if (planner_changed)
        {
            planner_params_.max_linear_velocity = pval("max_linear_velocity").as_double();
            planner_params_.max_linear_acceleration = pval("max_linear_acceleration").as_double();
            planner_params_.max_linear_deceleration = pval("max_linear_deceleration").as_double();
            planner_params_.max_linear_jerk = pval("max_linear_jerk").as_double();
            planner_params_.max_angular_velocity = pval("max_angular_velocity").as_double();
            planner_params_.max_angular_acceleration = pval("max_angular_acceleration").as_double();
            planner_params_.max_angular_deceleration = pval("max_angular_deceleration").as_double();
            planner_params_.max_angular_jerk = pval("max_angular_jerk").as_double();
            planner_params_.min_linear_velocity = pval("min_linear_velocity").as_double();
            planner_params_.min_angular_velocity = pval("min_angular_velocity").as_double();
            planner_params_.ruckig_control_dt = pval("ruckig_control_dt").as_double();
            planner_params_.curvature_deceleration = pval("curvature_deceleration").as_bool();
            planner_params_.curvature_dec_gain = pval("curvature_dec_gain").as_double();
            planner_params_.curvature_dec_ratio_min = pval("curvature_dec_ratio_min").as_double();
            planner_params_.acceleration_envelope = pval("acceleration_envelope").as_bool();
            planner_params_.velocity_profile_mode =
                pval("velocity_profile_mode").as_string();
            planner_params_.angular_accel_margin =
                pval("angular_accel_margin").as_double();
            planner_params_.angle_resolution = pval("angle_resolution").as_double();
            velocity_planner_.setParams(planner_params_);
        }

        if (gains_changed)
        {
            RCLCPP_INFO(get_logger(), "PID gains updated online.");
        }
        return result;
    }

    void PathFollowerNode::applyPidConfig()
    {
        PidAxisConfig x_cfg, y_cfg, theta_cfg;
        x_cfg.gains = x_gains_;
        x_cfg.output_limit = 0.0;
        x_cfg.vel_levels = vel_levels_;
        y_cfg.gains = y_gains_;
        y_cfg.output_limit = max_correct_vy_;
        y_cfg.vel_levels = vel_levels_;
        theta_cfg.gains = theta_gains_;
        theta_cfg.output_limit = max_correct_w_;
        theta_cfg.vel_levels = vel_levels_;
        pid_.configure(x_cfg, y_cfg, theta_cfg, vel_levels_, max_correct_vy_, max_correct_w_);
    }

    // ---------------------------------------------------------------------------
    // 订阅回调
    // ---------------------------------------------------------------------------
    std::vector<TrajectoryPoint> PathFollowerNode::pathToTrajectoryPoints(
        const nav_msgs::msg::Path& path) const
    {
        std::vector<TrajectoryPoint> pts;
        pts.reserve(path.poses.size());
        for (const auto& pose_stamped : path.poses)
        {
            TrajectoryPoint p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            // 上游若填了解析切向就带下来；未填时四元数是单位阵，yaw 恰为 0，
            // 由 inputHeadingUsable 统一判定要不要采信。
            p.heading_angle = yawFromQuaternion(pose_stamped.pose.orientation);
            pts.push_back(p);
        }
        return pts;
    }

    std::vector<TrajectoryPoint> PathFollowerNode::posesToTrajectoryPoints(
        const std::vector<geometry_msgs::msg::Pose>& poses) const
    {
        std::vector<TrajectoryPoint> pts;
        pts.reserve(poses.size());
        for (const auto& pose : poses)
        {
            TrajectoryPoint p;
            p.x = pose.position.x;
            p.y = pose.position.y;
            p.heading_angle = yawFromQuaternion(pose.orientation);
            pts.push_back(p);
        }
        return pts;
    }

    bool PathFollowerNode::inputHeadingUsable(const std::vector<TrajectoryPoint>& pts) const
    {
        if (pts.size() < 2)
        {
            return false;
        }

        const bool any_nonzero = std::any_of(
            pts.begin(), pts.end(),
            [](const TrajectoryPoint& p) { return std::fabs(p.heading_angle) > kHeadingZeroEps; });
        if (!any_nonzero)
        {
            RCLCPP_INFO(get_logger(),
                        "Input path carries no orientation (all identity); "
                        "heading/curvature will be derived from positions.");
            return false;
        }

        // 抽样比较朝向与弦向。间距很小时弦向本身滞后半格（约 kappa*ds/2），
        // 所以容差取得很松（默认 30 度），只用来抓"根本没对上"这类错误。
        const std::size_t step =
            std::max<std::size_t>(1, (pts.size() - 1) / kHeadingCheckSamples);
        double worst = 0.0;
        std::size_t worst_index = 0;
        for (std::size_t i = 0; i + 1 < pts.size(); i += step)
        {
            const double dx = pts[i + 1].x - pts[i].x;
            const double dy = pts[i + 1].y - pts[i].y;
            if (std::hypot(dx, dy) < kHeadingZeroEps)
            {
                continue; // 重复点，弦向无定义
            }
            const double chord = std::atan2(dy, dx);
            const double diff =
                std::fabs(angles::shortest_angular_distance(pts[i].heading_angle, chord));
            if (diff > worst)
            {
                worst = diff;
                worst_index = i;
            }
        }

        if (worst > kHeadingConsistencyTol)
        {
            RCLCPP_WARN(get_logger(),
                        "Input orientation inconsistent with path geometry "
                        "(max %.1f deg at point %zu > %.1f deg tolerance); "
                        "falling back to self-computed heading. "
                        "Check that the publisher fills the travel-direction tangent, "
                        "not the vehicle yaw.",
                        worst * 180.0 / M_PI, worst_index,
                        kHeadingConsistencyTol * 180.0 / M_PI);
            return false;
        }

        RCLCPP_INFO(get_logger(),
                    "Using input orientation as path tangent "
                    "(max chord deviation %.2f deg over %zu points).",
                    worst * 180.0 / M_PI, pts.size());
        return true;
    }

    void PathFollowerNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // inactive 期间收到的路线一律丢弃，不缓存：缓存下来就等于下一次 activate 时
        // 继承了一条不知道多旧的路线
        if (!active_.load())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Not active (%s), dropping incoming /path.",
                                 get_current_state().label().c_str());
            return;
        }
        if (msg->poses.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received empty path, ignored.");
            return;
        }

        // 目标朝向 = 末点朝向（输入路径唯一被采信的朝向）。
        const double goal_theta = yawFromQuaternion(msg->poses.back().pose.orientation);

        // onPath 走节点参数：base_type 决定 mode，enable_reverse 决定倒车。
        // hold_direction 留 NaN：omni_translation 时由 buildAndLoadRoute 解析为 start_theta（保持起步朝向）。
        SegmentProfile profile;
        switch (base_type_)
        {
        case BaseType::kDiff:             profile.mode = TrackingMode::kDiff; break;
        case BaseType::kOmniTranslation:  profile.mode = TrackingMode::kOmniTranslation; break;
        case BaseType::kOmniTransRotation: profile.mode = TrackingMode::kOmniTransRotation; break;
        }
        profile.enable_reverse = enable_reverse_;
        profile.has_target_yaw = true;
        profile.target_yaw = goal_theta;

        auto raw = pathToTrajectoryPoints(*msg);
        buildAndLoadRoute(std::move(raw), msg->header.frame_id, profile);
    }

    void PathFollowerNode::onTrackingPath(const nav_interfaces::msg::TrackingPath::SharedPtr msg)
    {
        // 同 onPath：失活期间的输入直接丢，不进缓存
        if (!active_.load())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Not active (%s), dropping incoming /tracking_path.",
                                 get_current_state().label().c_str());
            return;
        }
        if (msg->poses.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Received empty tracking_path, ignored.");
            return;
        }

        SegmentProfile profile;
        switch (msg->mode)
        {
        case nav_interfaces::msg::TrackingPath::MODE_DIFF:
            profile.mode = TrackingMode::kDiff;
            break;
        case nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION:
            profile.mode = TrackingMode::kOmniTranslation;
            break;
        case nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION:
            profile.mode = TrackingMode::kOmniTransRotation;
            break;
        default:
            RCLCPP_WARN(get_logger(),
                        "TrackingPath mode=%u unknown, fall back to OMNI_TRANS_ROTATION.",
                        msg->mode);
            profile.mode = TrackingMode::kOmniTransRotation;
            break;
        }

        // base_type_ 固定为 omni_trans_rotation 时强制使用该跟踪模式，不受 TrackingPath.mode
        // 影响：该底盘不支持 diff/omni_translation 的三阶段状态机，避免上游误发导致模式错配。
        if (base_type_ == BaseType::kOmniTransRotation && profile.mode != TrackingMode::kOmniTransRotation)
        {
            RCLCPP_WARN(get_logger(),
                        "base_type=omni_trans_rotation: forcing TrackingMode to OMNI_TRANS_ROTATION "
                        "(TrackingPath.mode=%u ignored).",
                        msg->mode);
            profile.mode = TrackingMode::kOmniTransRotation;
        }

        // 倒车仅 diff 生效；非 diff 下收到 REVERSE 告警并忽略。
        profile.enable_reverse =
            (profile.mode == TrackingMode::kDiff) &&
            (msg->backward == nav_interfaces::msg::TrackingPath::REVERSE);
        if (msg->backward == nav_interfaces::msg::TrackingPath::REVERSE &&
            profile.mode != TrackingMode::kDiff)
        {
            RCLCPP_WARN(get_logger(),
                        "TrackingPath backward=REVERSE ignored under mode=%u (non-diff).",
                        msg->mode);
        }

        // target_yaw：NaN=到点不做末段旋转。
        profile.has_target_yaw = !std::isnan(msg->target_yaw);
        profile.target_yaw = profile.has_target_yaw ? static_cast<double>(msg->target_yaw) : 0.0;
        // float32->double，NaN 在转换中保留。
        profile.hold_direction = static_cast<double>(msg->hold_direction);
        // profile.max_speed = static_cast<double>(msg->max_speed);  # 暂时关闭指定最大速度的接收
        profile.padding = static_cast<double>(msg->padding);

        RCLCPP_INFO(get_logger(),
                    "TrackingPath: mode=%u backward=%u reverse=%d target_yaw=%s(%.4f) "
                    "hold_dir=%s max_speed=%s padding=%s poses=%zu",
                    msg->mode, msg->backward, static_cast<int>(profile.enable_reverse),
                    profile.has_target_yaw ? "set" : "nan", profile.target_yaw,
                    std::isnan(profile.hold_direction) ? "nan" : "set",
                    std::isnan(profile.max_speed) ? "nan" : "set",
                    std::isnan(profile.padding) ? "nan" : "set",
                    msg->poses.size());

        auto raw = posesToTrajectoryPoints(msg->poses);
        buildAndLoadRoute(std::move(raw), msg->header.frame_id, profile);
    }

    void PathFollowerNode::buildAndLoadRoute(
        std::vector<TrajectoryPoint> raw,
        const std::string& frame_id,
        SegmentProfile profile)
    {
        // 起步前置检查：障碍状态在任务下发时就已经陈旧，说明停障能力当下就是缺失的，
        // 直接拒掉这次任务，不要接下来再靠控制环的看门狗兜底——那等于先开出去一段再报错。
        if (obstacle_status_timeout_ > 0.0)
        {
            const double status_age = obstacleStatusAge();
            if (status_age > obstacle_status_timeout_)
            {
                RCLCPP_ERROR(get_logger(),
                             "Refusing new path: obstacle status already stale for %.3fs > %.3fs "
                             "(topic=%s). Obstacle avoidance is unavailable.",
                             status_age, obstacle_status_timeout_, obstacle_status_topic_.c_str());
                publishCmdVel(Velocity{});
                char buf[200];
                std::snprintf(buf, sizeof(buf),
                              "obstacle status already stale for %.3fs > %.3fs (topic=%s), "
                              "refusing to start tracking",
                              status_age, obstacle_status_timeout_, obstacle_status_topic_.c_str());
                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, buf);
                return;
            }
        }

        path_frame_ = frame_id.empty() ? odom_frame_ : frame_id;
        RCLCPP_INFO(get_logger(), "Path frame: '%s' -> %s closed-loop tracking.",
                    path_frame_.c_str(),
                    (path_frame_ == odom_frame_) ? "odom" : "TF");

        // 新路线：末端定位切换状态重置为未激活，改用新目标点重新判定（避免沿用上一路线的锚定状态）。
        odom_anchor_active_ = false;
        // 定位跳变兜底同理：滚动锚定/冻结计时都不应跨路线沿用。
        resetLocalizationJumpGuard();
        // odom 强制停车状态同理：换了新路线就重新计时，否则会带着上一条路线的截止时刻。
        odom_stale_stop_active_ = false;

        // 起点朝向 = 当前车在参考坐标系（map/odom）下的实时朝向。
        // allow_localization_switch=false：此处仅为取新路线起点朝向，route_ 还是旧路线，
        // 不应基于旧目标点判定/激活末端定位切换。
        auto state = getCurrentState(false);
        if (!state)
        {
            RCLCPP_ERROR(get_logger(), "No tf pose received!");
            publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED,
                                  "no tf pose received, cannot start tracking");
            return;
        }
        const double start_theta = state->first.theta;
        // omni 平移：hold_direction NaN -> start_theta（保持起步朝向，阶段一有具体目标）。
        if (profile.mode == TrackingMode::kOmniTranslation && std::isnan(profile.hold_direction))
        {
            profile.hold_direction = start_theta;
        }
        // has_target_yaw=false 时占位为 start_theta：控制环据此跳过末段旋转，不会用到该值。
        const double goal_theta =
            profile.has_target_yaw ? profile.target_yaw : start_theta;

        // 路径截断：若车已经在路径中段，丢掉车之前的历史段，
        // 从最近点开始跟踪；末端至少保留 2 个点，避免规划器无法生成段。
        if (enable_path_truncate_ && state && raw.size() >= 2)
        {
            const double cx = state->first.x;
            const double cy = state->first.y;
            double best_d2 = std::numeric_limits<double>::max();
            std::size_t best_i = 0;
            for (std::size_t i = 0; i < raw.size(); ++i)
            {
                const double dx = raw[i].x - cx;
                const double dy = raw[i].y - cy;
                const double d2 = dx * dx + dy * dy;
                if (d2 < best_d2)
                {
                    best_d2 = d2;
                    best_i = i;
                }
            }
            if (best_i > 0)
            {
                // 保证末端至少还有 2 个点（起点 + 终点）能形成一段。
                const std::size_t max_from = raw.size() - 2;
                const std::size_t truncate_from = std::min(best_i, max_from);
                raw.erase(raw.begin(), raw.begin() + static_cast<std::ptrdiff_t>(truncate_from));
                RCLCPP_INFO(get_logger(),
                            "Path truncated: skipped %zu head point(s), nearest_d=%.3fm, "
                            "remaining %zu points.",
                            truncate_from, std::sqrt(best_d2), raw.size());
            }
        }

        PlannedRoute pr;
        const bool trust_heading = inputHeadingUsable(raw);
        pr.position = velocity_planner_.planPosition(raw, trust_heading);
        pr.goal_theta = goal_theta;

        // 轨迹首末切向（planPosition 已算好 heading_angle）。
        if (!pr.position.points.empty())
        {
            pr.path_start_heading = pr.position.points.front().heading_angle;
            pr.path_end_heading = pr.position.points.back().heading_angle;
        }

        if (profile.mode == TrackingMode::kDiff)
        {
            // diff：阶段一旋转 current→path_start_heading；阶段三预规划 path_end→goal。
            // 倒车模式：阶段一对准切线反向（+π）。
            const double align_target = profile.enable_reverse
                ? angles::normalize_angle(pr.path_start_heading + M_PI)
                : pr.path_start_heading;
            pr.heading_align = velocity_planner_.planHeading(start_theta, align_target);
            if (profile.has_target_yaw)
            {
                // 倒车模式：阶段三起点对准切线反向（+π）。
                const double align_start = profile.enable_reverse
                    ? angles::normalize_angle(pr.path_end_heading + M_PI)
                    : pr.path_end_heading;
                pr.heading_final = velocity_planner_.planHeading(align_start, goal_theta);
            }
        }
        else if (profile.mode == TrackingMode::kOmniTranslation)
        {
            // omni 平移（三阶段，与 diff 同构）：
            //   阶段一旋转 start_theta -> hold_direction；
            //   阶段二朝向锁定 hold_direction（不查朝向轨迹）；
            //   阶段三旋转 hold_direction -> target_yaw（有 target_yaw 时预规划）。
            pr.heading_align = velocity_planner_.planHeading(start_theta, profile.hold_direction);
            if (profile.has_target_yaw)
            {
                pr.heading_final = velocity_planner_.planHeading(profile.hold_direction, goal_theta);
            }
        }
        else if (profile.mode == TrackingMode::kOmniTransRotation)
        {
            // omni 平移+旋转：位置与朝向并行（start->goal），单阶段。
            // has_target_yaw=false 时 goal_theta 上面已置为 start_theta（无末段旋转），
            // 直接规划即可得到零旋转的退化 heading_align，无需为此单独分支。
            pr.heading_align = velocity_planner_.planHeading(start_theta, goal_theta);

            // ---- [实验功能] 舵角限位可行性分析：不可行时报导航失败、终止本次移动 ----
            if (enable_steer_range_analysis_)
            {
                const auto steer_analysis = analyzeOmniSteerRange(
                    pr.position, pr.heading_align, start_theta, experimental_steer_limit_deg_);
                if (steer_analysis.ok)
                {
                    RCLCPP_INFO(get_logger(),
                                "[steer-range][experimental] phi(path切向,unwrap)=[%.2f, %.2f]deg "
                                "theta(heading)=[%.2f, %.2f]deg raw_steer(=phi-theta)=[%.2f, %.2f]deg "
                                "span=%.2fdeg raw_steer0=%.2fdeg raw_steer_end=%.2fdeg",
                                steer_analysis.phi_min_deg, steer_analysis.phi_max_deg,
                                steer_analysis.theta_min_deg, steer_analysis.theta_max_deg,
                                steer_analysis.raw_min_deg, steer_analysis.raw_max_deg,
                                steer_analysis.span_deg, steer_analysis.raw_steer0_deg,
                                steer_analysis.raw_steer_end_deg);
                    if (steer_analysis.feasible)
                    {
                        RCLCPP_INFO(get_logger(),
                                    "[steer-range][experimental] FEASIBLE branch=%s (%s) "
                                    "branch_range=[%.2f, %.2f]deg (limit=+-%.1fdeg) theta0=%.2fdeg "
                                    "steer_end=%.2fdeg",
                                    steer_analysis.use_flip ? "flip(反接/反转速度)" : "normal(正接)",
                                    steer_analysis.both_branches_ok ? "两支均可行,按|终点舵角|择优"
                                                                    : "仅此支可行",
                                    steer_analysis.branch_min_deg, steer_analysis.branch_max_deg,
                                    experimental_steer_limit_deg_, steer_analysis.theta0_deg,
                                    steer_analysis.steer_end_deg);

                        // ---- [实验功能] 试调用 /steering_angle_control action 把四轮统一摆到 theta0 ----
                        // SteeringAngleControl 目标是 target_steer_angles_rad[4]（FL/FR/RL/RR 各自独立目标角），
                        // 这里四轮都填同一个 theta0，即"全轮同向对齐"，与可行性分析的物理含义一致。
                        if (steering_angle_control_client_ && steering_angle_control_client_->action_server_is_ready())
                        {
                            auto goal = nav_interfaces::action::SteeringAngleControl::Goal();
                            const float theta0_rad =
                                static_cast<float>(steer_analysis.theta0_deg * M_PI / 180.0);
                            goal.target_steer_angles_rad = {theta0_rad, theta0_rad, theta0_rad, theta0_rad};
                            goal.timeout_sec = 2.0f;
                            goal.settle_tolerance_rad = 0.05f;

                            rclcpp_action::Client<nav_interfaces::action::SteeringAngleControl>::SendGoalOptions
                                send_goal_options;
                            send_goal_options.feedback_callback =
                                [this](
                                    rclcpp_action::ClientGoalHandle<
                                        nav_interfaces::action::SteeringAngleControl>::SharedPtr /*handle*/,
                                    const std::shared_ptr<
                                        const nav_interfaces::action::SteeringAngleControl::Feedback> fb)
                            {
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                            "[steer-range][experimental] steering_angle_control feedback: "
                                            "max_steer_error=%.4frad",
                                            fb->max_steer_error);
                            };

                            // [实验功能] 同步等待舵轮转到位：整体预算 3 秒（含 goal 接受 + 执行结果），
                            // 超时/被拒绝/执行失败都视为本次移动失败，直接报 FAILED 并终止本次路由加载。
                            // 同步等待期间本节点的单线程执行器被独占（spin 的是 steer_helper_node_），
                            // 自己的订阅回调一次都执行不到，需要记录这段被独占的时长，等待结束后从
                            // 看门狗里扣掉，见下方 excludeBlockedWindowFromWatchdogs。
                            const rclcpp::Time steer_block_begin = now();
                            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
                            auto remaining = [&deadline]() {
                                const auto now = std::chrono::steady_clock::now();
                                return now < deadline ? (deadline - now) : std::chrono::steady_clock::duration::zero();
                            };

                            std::string steer_fail_reason;
                            auto goal_handle_future =
                                steering_angle_control_client_->async_send_goal(goal, send_goal_options);
                            if (rclcpp::spin_until_future_complete(steer_helper_node_, goal_handle_future,
                                                                    remaining()) !=
                                rclcpp::FutureReturnCode::SUCCESS)
                            {
                                steer_fail_reason = "steering_angle_control goal send timed out (>3s)";
                            }
                            else if (auto goal_handle = goal_handle_future.get(); !goal_handle)
                            {
                                steer_fail_reason = "steering_angle_control goal rejected by server";
                            }
                            else
                            {
                                auto result_future = steering_angle_control_client_->async_get_result(goal_handle);
                                if (rclcpp::spin_until_future_complete(steer_helper_node_, result_future,
                                                                        remaining()) !=
                                    rclcpp::FutureReturnCode::SUCCESS)
                                {
                                    steer_fail_reason = "steering_angle_control result timed out (>3s)";
                                }
                                else
                                {
                                    const auto wrapped = result_future.get();
                                    RCLCPP_INFO(get_logger(),
                                                "[steer-range][experimental] steering_angle_control result: "
                                                "code=%d success=%d message=%s",
                                                static_cast<int>(wrapped.code),
                                                wrapped.result ? static_cast<int>(wrapped.result->success) : 0,
                                                wrapped.result ? wrapped.result->message.c_str() : "");
                                    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
                                        !wrapped.result->success)
                                    {
                                        steer_fail_reason = "steering_angle_control failed: " +
                                            (wrapped.result ? wrapped.result->message : std::string("no result"));
                                    }
                                }
                            }

                            // 预摆窗口不计入看门狗：上面所有分支（成功/超时/被拒/失败）都汇到这里，
                            // 统一在此扣除，避免遗漏某条退出路径。
                            excludeBlockedWindowFromWatchdogs(steer_block_begin,
                                                             "steering pre-rotate");

                            if (!steer_fail_reason.empty())
                            {
                                RCLCPP_WARN(get_logger(), "[steer-range][experimental] %s",
                                            steer_fail_reason.c_str());
                                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED,
                                                       steer_fail_reason);
                                return;
                            }
                        }
                        else
                        {
                            RCLCPP_WARN(get_logger(),
                                        "[steer-range][experimental] /steering_angle_control action server not "
                                        "ready, skip pre-rotate call");
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(),
                                    "[steer-range][experimental] INFEASIBLE: no branch fits within "
                                    "+-%.1fdeg (span=%.2fdeg)",
                                    experimental_steer_limit_deg_, steer_analysis.span_deg);
                        char buf[256];
                        std::snprintf(buf, sizeof(buf),
                                      "steer range infeasible: span=%.2fdeg exceeds +-%.1fdeg limit "
                                      "(raw_steer=[%.2f, %.2f]deg)",
                                      steer_analysis.span_deg, experimental_steer_limit_deg_,
                                      steer_analysis.raw_min_deg, steer_analysis.raw_max_deg);
                        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, buf);
                        return;
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(route_mutex_);
            route_ = std::move(pr);
            segment_ = profile;
            nearest_index_ = 0;
            last_index_ = 0;
            track_index_ = 0;
            heading_index_ = 0;
            last_heading_index_ = 0;
            track_phase_ = TrackPhase::kAlignToPath;
            route_finished_ = false;
            route_failed_ = false;
            is_need_reset_ = true;
            // 新路线的全局速度曲线已规划完成，从跟踪参考速度的稳态起步；
            // 若此刻障碍状态并非 FREE，下一控制周期 updateVelocityTrackMode
            // 会立即切到对应的减速态，不需要在这里特殊处理。
            resetVelocityTrack();
        }
        {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "tracking started: %zu points, total_s=%.3fm",
                          route_ ? route_->position.points.size() : 0,
                          route_ ? route_->position.total_s : 0.0);
            publishNavigationStatus(nav_interfaces::msg::NavigationStatus::NAVIGATING, buf);
        }
        last_path_time_ = now();
        have_control_update_time_ = false;

        RCLCPP_INFO(
            get_logger(),
            "New path: %zu position points (total_s=%.3fm), heading_align %zu points "
            "(start=%.4f %s=%.4f rad, span=%.4f rad)%s.",
            route_ ? route_->position.points.size() : 0,
            route_ ? route_->position.total_s : 0.0,
            route_ ? route_->heading_align.points.size() : 0,
            start_theta,
            profile.mode == TrackingMode::kDiff ? "path_h0" : "goal",
            profile.mode == TrackingMode::kDiff ? route_->path_start_heading : goal_theta,
            route_ ? route_->heading_align.total_a : 0.0,
            profile.mode == TrackingMode::kDiff ? " [diff: phase1 align->path]" : "");


        publishDebugPath();
    }

    void PathFollowerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const rclcpp::Time recv_time = now();
        // 显式带上节点时钟的 clock type：默认构造出来的是 RCL_ROS_TIME，与 now() 不一致时相减会抛。
        const rclcpp::Time msg_stamp(msg->header.stamp, recv_time.get_clock_type());

        std::lock_guard<std::mutex> lock(odom_mutex_);

        // 相邻两帧的接收间隔诊断。这里是唯一能同时拿到"上一帧与当前帧 msg 时间戳"的地方
        // ——控制环里的年龄看门狗触发时当前帧压根还没到，只能打上一帧。两处日志分工互补。
        if (have_odom_ && odom_gap_log_threshold_ > 0.0)
        {
            const double recv_dt = (recv_time - last_odom_time_).seconds();
            if (recv_dt > odom_gap_log_threshold_)
            {
                const double stamp_dt = (msg_stamp - last_odom_stamp_).seconds();
                const double latency = (recv_time - msg_stamp).seconds();
                // 三个量组合才能把病因分开：
                //   stamp_dt 也大            -> 发布端自己卡了，是里程计模块的问题；
                //   stamp_dt 正常而 recv_dt 大 -> 传输/执行器阻塞，消息在 DDS 队列里积压；
                //   latency 持续偏大          -> 吞吐跟不上，而不是偶发卡顿；
                //   1 次大间隔后紧跟几帧 ~0   -> 本节点自阻塞，积压帧被连续派发（见
                //                               excludeBlockedWindowFromWatchdogs）。
                RCLCPP_ERROR(get_logger(),
                             "[odom-gap] recv_dt=%.3fs > %.3fs (stamp_dt=%.3fs latency=%.3fs): "
                             "prev_stamp=%.6f cur_stamp=%.6f",
                             recv_dt, odom_gap_log_threshold_, stamp_dt, latency,
                             last_odom_stamp_.seconds(), msg_stamp.seconds());
            }
        }

        latest_odom_ = *msg;
        last_odom_time_ = recv_time;
        last_odom_stamp_ = msg_stamp;
        have_odom_ = true;
    }

    void PathFollowerNode::onObstacleStatus(
        const nav_interfaces::msg::ObstacleMonitorStatus::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(obstacle_status_mutex_);
        latest_obstacle_status_ = msg->status;
        // 用本地接收时刻而不是 msg->header.stamp 作为看门狗基准：这里要判的是
        // "障碍检测模块还活着吗"，用发布方自己的时间戳无法覆盖"节点卡死但时钟仍在走"。
        last_obstacle_status_time_ = now();
        // 迟滞时间戳记录"最后一次真正收到该状态的时刻"，必须在这里更新而不是控制环里：
        // 控制环看到的是缓存值，发布方静默时缓存值不变，会被每周期误当成"又收到一次"，
        // 使迟滞永不过期。
        if (msg->status == nav_interfaces::msg::ObstacleMonitorStatus::STOP)
        {
            last_obstacle_stop_time_ = last_obstacle_status_time_;
        }
        else if (msg->status == nav_interfaces::msg::ObstacleMonitorStatus::SLOWDOWN)
        {
            last_obstacle_slowdown_time_ = last_obstacle_status_time_;
        }
    }

    void PathFollowerNode::excludeBlockedWindowFromWatchdogs(
        const rclcpp::Time& block_begin, const char* reason)
    {
        // 本节点是单线程执行器（main 里 rclcpp::spin(node)），一旦在某个回调里同步等待，
        // 期间自己的 obstacle_status 回调一次都执行不到——消息其实已经躺在 DDS 队列里了，
        // 只是没人取。而看门狗基准用的是"本地接收时刻"（见 onObstacleStatus 的注释），
        // 于是这段自阻塞会被原样记成"障碍检测链路静默"，阻塞一解除，执行器又优先跑到期的
        // 控制定时器（rclcpp 单线程执行器 timer 排在 subscription 前面），第一拍就拿着旧
        // 时间戳判 stale 并终止导航——而积压的 obstacle_status 就排在这一拍后面。
        //
        // 处理方式是把被独占的这段时长从时间轴上剪掉：三个时间戳整体后移同样的量。
        // 不是直接置为 now()——那等于无条件宣称"刚收到过"，发布方真的挂了也发现不了；
        // 后移之后，若发布方确实静默，年龄仍会从剪裁点继续增长，超时判定只是相应延后。
        // 迟滞时间戳一并后移，保持三者在同一条时间轴上：STOP/SLOWDOWN 闩锁因此不会被
        // 预摆窗口白白耗掉，方向上偏保守。
        const rclcpp::Time block_end = now();
        if (block_end <= block_begin)
        {
            return;
        }
        const rclcpp::Duration blocked = block_end - block_begin;

        {
            std::lock_guard<std::mutex> lock(obstacle_status_mutex_);
            // 阻塞期间回调跑不了，这三个时间戳必然都 <= block_begin，后移之后不会越过 now()。
            last_obstacle_status_time_ = last_obstacle_status_time_ + blocked;
            last_obstacle_stop_time_ = last_obstacle_stop_time_ + blocked;
            last_obstacle_slowdown_time_ = last_obstacle_slowdown_time_ + blocked;
        }

        {
            // odom 接收年龄看门狗同理：基准也是"本地接收时刻"，自阻塞期间 onOdom 一次都跑不到，
            // 不剪窗的话阻塞一解除，控制定时器第一拍就会拿着旧时间戳判 stale 并刹停——而积压的
            // odom 就排在这一拍后面。两个锁分开加，不嵌套。
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (have_odom_)
            {
                last_odom_time_ = last_odom_time_ + blocked;
            }
        }

        RCLCPP_INFO(get_logger(),
                    "Watchdogs: excluded %.3fs blocked window (%s) from staleness.",
                    blocked.seconds(), reason);
    }

    double PathFollowerNode::obstacleStatusAge() const
    {
        rclcpp::Time status_time;
        {
            std::lock_guard<std::mutex> lock(obstacle_status_mutex_);
            status_time = last_obstacle_status_time_;
        }
        // 用 Node::now()（const）而不是 get_clock()->now()：后者在 const 方法里拿到的是
        // Clock::ConstSharedPtr，而 Clock::now() 非 const，编译不过。
        return (now() - status_time).seconds();
    }

    void PathFollowerNode::updateVelocityTrackMode()
    {
        uint8_t status;
        rclcpp::Time status_time;
        rclcpp::Time stop_time;
        rclcpp::Time slowdown_time;
        {
            std::lock_guard<std::mutex> lock(obstacle_status_mutex_);
            status = latest_obstacle_status_;
            status_time = last_obstacle_status_time_;
            stop_time = last_obstacle_stop_time_;
            slowdown_time = last_obstacle_slowdown_time_;
        }

        const auto now_t = get_clock()->now();

        std::lock_guard<std::mutex> lock(route_mutex_);
        // 无路线时不做判定：没有正在执行的任务可停、可判失败。任务下发那一刻的新鲜度
        // 由 buildAndLoadRoute 的起步前置检查负责，本函数只保护"正在跟踪的过程中"。
        if (!route_.has_value())
        {
            return;
        }

        // 障碍状态看门狗：超时说明障碍检测链路已经不可信，继续沿用上一帧的状态就等于
        // 带着"看不见障碍"往前开，停车并把本次导航判失败。
        const double status_age = (now_t - status_time).seconds();
        if (obstacle_status_timeout_ > 0.0 && status_age > obstacle_status_timeout_)
        {
            RCLCPP_ERROR(get_logger(),
                         "Obstacle status STALE: no update for %.3fs > %.3fs (topic=%s). "
                         "Stopping and aborting trajectory.",
                         status_age, obstacle_status_timeout_, obstacle_status_topic_.c_str());
            publishCmdVel(Velocity{});
            route_.reset();
            route_failed_ = true;
            resetVelocityTrack();
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "obstacle status stale: no update for %.3fs > %.3fs (topic=%s)",
                          status_age, obstacle_status_timeout_, obstacle_status_topic_.c_str());
            publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, buf);
            return;
        }

        VelocityTrackMode new_mode = vel_track_mode_;
        switch (status)
        {
        case nav_interfaces::msg::ObstacleMonitorStatus::STOP:
            new_mode = VelocityTrackMode::kStop;
            break;
        case nav_interfaces::msg::ObstacleMonitorStatus::SLOWDOWN:
            // STOP 解除迟滞：刚刚还在 STOP 的 2s 内不允许降级到 SLOWDOWN。
            if (vel_track_mode_ == VelocityTrackMode::kStop &&
                now_t - stop_time < rclcpp::Duration(2, 0))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[speed-track] STOP->SLOWDOWN ignored, delay 2.0s");
                return;
            }
            new_mode = VelocityTrackMode::kSlowdown;
            break;
        default: // FREE
            // 障碍解除：从 kStop/kSlowdown 出来必须经 kResume 平滑加速回参考速度，
            // 不能直接跳 kFollowRef（那会让输出从模型低速一步跳回轨迹参考速度）。
            // kFollowRef/kResume 本身维持不变——kResume 的退出由线速度通道的收敛判定负责。
            if (vel_track_mode_ == VelocityTrackMode::kStop)
            {
                if (now_t - stop_time < rclcpp::Duration(2, 0))
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "[speed-track] STOP->FREE ignored, delay 2.0s");
                    return;
                }
                new_mode = VelocityTrackMode::kResume;
            }
            else if (vel_track_mode_ == VelocityTrackMode::kSlowdown)
            {
                if (now_t - slowdown_time < rclcpp::Duration(2, 0))
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "[speed-track] SLOWDOWN->FREE ignored, delay 2.0s");
                    return;
                }
                new_mode = VelocityTrackMode::kResume;
            }
            break;
        }

        if (new_mode != vel_track_mode_)
        {
            RCLCPP_INFO(get_logger(),
                        "[speed-track] obstacle status=%d -> mode %d->%d "
                        "(last_v=%.3f last_w=%.3f)",
                        status, static_cast<int>(vel_track_mode_), static_cast<int>(new_mode),
                        last_track_linear_v_, last_track_angular_w_);
            vel_track_mode_ = new_mode;
        }
    }

    double PathFollowerNode::stepTrackedLinearVelocity(double route_ref_v)
    {
        if (vel_track_mode_ == VelocityTrackMode::kFollowRef)
        {
            // 稳态：直接跟踪全局规划速度，模型不介入；但要记住本帧值，
            // 这样下一帧一旦切到减速态，模型能从真实的"上一帧速度"起步而不是从 0。
            last_track_linear_v_ = route_ref_v;
            return route_ref_v;
        }

        VelocityStepParams sp;
        sp.max_accel = planner_params_.max_linear_acceleration;
        sp.max_decel = planner_params_.max_linear_deceleration;
        sp.accel_decay_time = linear_accel_decay_time_;

        double v_target = route_ref_v;
        switch (vel_track_mode_)
        {
        case VelocityTrackMode::kStop:
            v_target = 0.0;
            sp.max_decel *= stop_decel_scale_;
            break;
        case VelocityTrackMode::kSlowdown:
        {
            // 上限语义：已经比 max/2 慢就跟着参考速度走，不会因为 SLOWDOWN 反而把车加速。
            const double ceiling = planner_params_.max_linear_velocity * 0.5;
            v_target = std::copysign(std::min(std::fabs(route_ref_v), ceiling), route_ref_v);
            sp.max_decel *= slowdown_decel_scale_;
            break;
        }
        case VelocityTrackMode::kResume:
        default:
            v_target = route_ref_v; // 恢复段用常规加速度追回参考速度
            break;
        }

        const double dt = 1.0 / std::max(control_rate_, 1e-6);
        double v_new = stepVelocityTowards(last_track_linear_v_, v_target, dt, sp);

        // 轨迹参考速度是硬上限：模型只负责"障碍时压得更低"，绝不能让输出高于全局规划。
        // 否则末端会发散：参考速度是空间剖面，其时间下降率 = a_ref·(v_actual/v_ref)，
        // 越靠近终点 v_ref 越小、这个比值越大，而模型的可用减速度是有限的（且差值小于
        // delta_v 时还要按 sqrt 衰减）。一旦追不上，车就比规划快 -> 更快啃掉剩余路程 ->
        // 参考塌得更快 -> 差距更大，正反馈，结果是带着残余速度冲到终点、再被阶段切换
        // 一步砍到 0。封顶后 SLOWDOWN 在"参考速度本来就低于 max/2"时自然退化为无操作。
        if (std::fabs(v_new) > std::fabs(route_ref_v))
        {
            v_new = std::copysign(std::fabs(route_ref_v), v_new);
        }
        // 必须把封顶后的值写回：模型状态要等于真正发出去的速度，否则下一帧仍从虚高的
        // "上一帧速度"起步，封顶就只是掩盖了发散而没有消除它。
        last_track_linear_v_ = v_new;

        // kResume 退出判定：输出已经追上参考速度就回到稳态，避免长期挂在模型上。
        if (vel_track_mode_ == VelocityTrackMode::kResume &&
            std::fabs(v_new - route_ref_v) <= resume_converge_linear_velocity_)
        {
            RCLCPP_INFO(get_logger(),
                        "[speed-track] resume converged (v=%.3f ref=%.3f) -> follow reference.",
                        v_new, route_ref_v);
            vel_track_mode_ = VelocityTrackMode::kFollowRef;
        }
        return v_new;
    }

    double PathFollowerNode::stepTrackedAngularVelocity(double route_ref_w)
    {
        if (vel_track_mode_ == VelocityTrackMode::kFollowRef)
        {
            last_track_angular_w_ = route_ref_w;
            return route_ref_w;
        }

        VelocityStepParams sp;
        sp.max_accel = planner_params_.max_angular_acceleration;
        sp.max_decel = planner_params_.max_angular_deceleration;
        sp.accel_decay_time = angular_accel_decay_time_;

        double w_target = route_ref_w;
        switch (vel_track_mode_)
        {
        case VelocityTrackMode::kStop:
            w_target = 0.0;
            sp.max_decel *= stop_decel_scale_;
            break;
        case VelocityTrackMode::kSlowdown:
        {
            const double ceiling = planner_params_.max_angular_velocity * 0.5;
            w_target = std::copysign(std::min(std::fabs(route_ref_w), ceiling), route_ref_w);
            sp.max_decel *= slowdown_decel_scale_;
            break;
        }
        case VelocityTrackMode::kResume:
        default:
            w_target = route_ref_w;
            break;
        }

        const double dt = 1.0 / std::max(control_rate_, 1e-6);
        double w_new = stepVelocityTowards(last_track_angular_w_, w_target, dt, sp);

        // 参考角速度同样是硬上限，机理与线速度通道一致（见 stepTrackedLinearVelocity）：
        // 原地旋转段的参考角速度也是按剩余转角排的空间剖面，末端不封顶会残留角速度。
        if (std::fabs(w_new) > std::fabs(route_ref_w))
        {
            w_new = std::copysign(std::fabs(route_ref_w), w_new);
        }
        last_track_angular_w_ = w_new;
        // 模式切换统一由线速度通道驱动（见头文件注释），这里只跟随、不判定。
        return w_new;
    }

    void PathFollowerNode::resetVelocityTrack()
    {
        vel_track_mode_ = VelocityTrackMode::kFollowRef;
        last_track_linear_v_ = 0.0;
        last_track_angular_w_ = 0.0;
        // 注意：这里绝不能刷新 last_obstacle_status_time_。障碍状态的新鲜度是外部事实，
        // 跟"换了一条新路线"没有任何关系；一旦在这里重置基准，collision_monitor 早就挂了
        // 也会被白送一个完整的超时窗口，车照样往前开满一个窗口才报错。
        // （曾经这么做是因为上游 collision_monitor 只在 cmd_vel 流动时才发布，车不动就没
        //   数据；现在它由 100 Hz 控制环驱动、空闲时也发布，状态持续流动，这个补偿既无必要也有害。）
    }

    Pose2D PathFollowerNode::propagateFromAnchor(
        const Pose2D& anchor_map, const Pose2D& anchor_odom, const Pose2D& odom_now)
    {
        // 视 odom->map 为锚定时刻标定的恒定刚体变换：
        //   dtheta = 锚定 map 朝向 - 锚定 odom 朝向；
        //   当前 map 位姿 = 锚定 map 位姿 + (当前 odom 位姿 - 锚定 odom 位姿) 旋转 dtheta 后的增量。
        const double dtheta = anchor_map.theta - anchor_odom.theta;
        const double c = std::cos(dtheta);
        const double s = std::sin(dtheta);
        const double dx = odom_now.x - anchor_odom.x;
        const double dy = odom_now.y - anchor_odom.y;

        Pose2D pose;
        pose.x = anchor_map.x + c * dx - s * dy;
        pose.y = anchor_map.y + s * dx + c * dy;
        pose.theta = angles::normalize_angle(
            anchor_map.theta + (odom_now.theta - anchor_odom.theta));
        return pose;
    }

    Pose2D PathFollowerNode::applyOdomAnchor(const Pose2D& odom_now) const
    {
        return propagateFromAnchor(odom_anchor_map_pose_, odom_anchor_odom_pose_, odom_now);
    }

    void PathFollowerNode::resetLocalizationJumpGuard()
    {
        jump_ref_valid_ = false;
        jump_stop_active_ = false;
        jump_abort_pending_ = false;
        jump_resume_pending_ = false;
        tf_fail_count_ = 0;
    }

    bool PathFollowerNode::lookupMapToOdom(Pose2D& map_odom, rclcpp::Time& stamp)
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                path_frame_, odom_frame_, tf2::TimePointZero,
                tf2::durationFromSec(tf_lookup_timeout_));
            map_odom.x = tf.transform.translation.x;
            map_odom.y = tf.transform.translation.y;
            map_odom.theta = yawFromQuaternion(tf.transform.rotation);
            stamp = rclcpp::Time(tf.header.stamp, now().get_clock_type());
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[loc-jump] TF lookup %s->%s failed: %s",
                                 path_frame_.c_str(), odom_frame_.c_str(), ex.what());
            return false;
        }
    }

    Pose2D PathFollowerNode::guardLocalizationJump(
        const Pose2D& map_pose, bool have_map_pose, const Pose2D& odom_pose)
    {
        const auto now_t = now();

        // ---- 旁路：取 path_frame->odom 变换（判据的唯一输入）----
        Pose2D map_odom;
        rclcpp::Time map_odom_stamp;
        const bool have_map_odom = lookupMapToOdom(map_odom, map_odom_stamp);
        if (have_map_odom)
        {
            last_map_odom_stamp_ = map_odom_stamp;
        }
        // 新鲜度看门狗：定位模块停发时 lookupTransform 依然会成功——TimePointZero 取的是
        // 最新可用那条，于是 T_cur 冻结、帧间差恒为 0、判据永远静默，而这恰恰是最该报的
        // 一种故障。所以必须按 stamp 年龄单独判。（AMCL 会把 stamp 按 transform_tolerance
        // 推到未来，正常时这个年龄是负的。）
        if (map_to_odom_timeout_ > 0.0)
        {
            const double tf_age = (now_t - last_map_odom_stamp_).seconds();
            if (tf_age > map_to_odom_timeout_)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                      "[loc-jump] %s->%s stale for %.2fs > %.2fs: "
                                      "localization link silent?",
                                      path_frame_.c_str(), odom_frame_.c_str(),
                                      tf_age, map_to_odom_timeout_);
            }
        }

        // ---- 首帧 / 重置后：两套基准一起建立 ----
        if (!jump_ref_valid_)
        {
            // 两路都到齐才建基准，否则本帧还谈不上判定，直接透传主路位姿。
            // （基准未建立时 getCurrentState 的 can_fallback 必为 false，走不到这里还没有
            //   主路位姿的情况；这里的 have_map_pose 判断只是防御，避免透传一个全零位姿。）
            if (have_map_pose && have_map_odom)
            {
                jump_ref_map_odom_ = map_odom;
                jump_anchor_map_pose_ = map_pose;
                jump_anchor_odom_pose_ = odom_pose;
                jump_ref_valid_ = true;
            }
            return map_pose;
        }

        // 锚定对递推出的"我们认为车在哪"。非冻结期锚定对是上一帧的，此时该值 = 上一帧 map
        // 位姿 + 这一帧 odom 增量；冻结期锚定对停在跳变前那一帧，该值就是控制真正在用的连续位姿。
        const Pose2D dr_pose =
            propagateFromAnchor(jump_anchor_map_pose_, jump_anchor_odom_pose_, odom_pose);

        // ---- 判据：把 map->odom 的变化量投影到车当前位置 ----
        // propagateFromAnchor(T, {}, p) 就是刚体复合 T * p：锚定 odom 位姿取零位时，公式退化成
        // "把 p 用 T 变换到 map 系"。两次投影乘的是**同一帧 odom_pose**，车的真实运动被完全
        // 抵消（T_cur==T_ref 时 disagree 恒为 0，无论车在哪、跑多快），odom 话题的传输延迟
        // 同样被完全抵消——这正是换掉老判据的原因，见头文件注释。
        double d_xy = 0.0;
        double d_theta = 0.0;
        double disagree = 0.0;
        if (have_map_odom)
        {
            const Pose2D pose_from_cur = propagateFromAnchor(map_odom, Pose2D{}, odom_pose);
            const Pose2D pose_from_ref =
                propagateFromAnchor(jump_ref_map_odom_, Pose2D{}, odom_pose);
            d_xy = std::hypot(pose_from_cur.x - pose_from_ref.x,
                              pose_from_cur.y - pose_from_ref.y);
            // 朝向差必须先归一化到 (-pi, pi]：两个估计各自累积，完全可能一个在 +3.13、另一个
            // 在 -3.13，裸相减得到 6.26 的假跳变。归一化后同样的情形只有 0.02。
            d_theta = angles::normalize_angle(pose_from_cur.theta - pose_from_ref.theta);
            // 位置(m)与朝向(rad)同权合成：1 rad 记作 1 m。位置分量来自投影项 (R(dphi)-I)*p，
            // 它只覆盖"旋转导致的位置跳变"——车恰好在 odom 原点附近时 ||p||~0，纯旋转跳变的
            // 位移趋近于 0，这时全靠朝向分量兜住；而朝向本身也会直接进 delta_theta 前馈和
            // 航向对准，本来就该独立判。
            disagree = std::hypot(d_xy, d_theta);
        }

        // ---- 强制停车等待期 ----
        if (jump_stop_active_)
        {
            if (now_t < jump_stop_deadline_)
            {
                // have_map_odom 一并打出来：旁路查不到时 disagree 恒为 0，不标出来会被
                // 误读成"已经恢复了"。
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[loc-jump] stop-and-wait, %.1fs remaining "
                                     "(current disagreement %.4f vs thr %.4f, "
                                     "d_xy=%.4fm d_theta=%.4frad, have_map_odom=%d).",
                                     (jump_stop_deadline_ - now_t).seconds(),
                                     disagree, localization_jump_threshold_, d_xy, d_theta,
                                     static_cast<int>(have_map_odom));
                return dr_pose;
            }

            // 等满，做唯一一次判定。任一路 TF 不可用都无从判定，按未恢复处理：
            // 定位链路本身有问题时，继续跑比停下更危险。
            // 恢复判据比触发判据放宽 recover_scale 倍：小范围跳变多半是系统噪声，停下来
            // 观望满一个窗口之后允许继续走，不必因为一次噪声就判导航失败。
            const double recover_threshold =
                localization_jump_threshold_ * localization_jump_recover_scale_;
            if (!have_map_odom || !have_map_pose || disagree > recover_threshold)
            {
                RCLCPP_ERROR(get_logger(),
                             "[loc-jump] localization NOT recovered after %.1fs stop-and-wait: "
                             "disagreement %.4f > %.4f (d_xy=%.4fm d_theta=%.4frad, "
                             "have_map_odom=%d have_map_pose=%d). Aborting trajectory.",
                             localization_jump_stop_wait_duration_, disagree,
                             recover_threshold, d_xy, d_theta,
                             static_cast<int>(have_map_odom), static_cast<int>(have_map_pose));
                jump_abort_pending_ = true;
                return dr_pose;
            }

            // 已恢复：两套基准一起重建，并让控制环从 0 速走 kResume 复速（细节见 update()）。
            // 恢复判据比触发判据放宽一倍：小范围跳变多半是系统噪声，停下来观望满一个窗口
            // 之后允许继续走。代价是交还带来的位姿台阶最多到 2 倍阈值（而不是一倍）——
            // 基准在这里一并重建，disagree 归零，不会立刻再次触发判据；但这个台阶会原样
            // 进到 checkTrackDeviation，所以 2*threshold 必须 <= track_deviation_limit，
            // 否则落在两者之间的跳变会"判恢复之后下一帧被跟踪偏差打掉"。
            jump_stop_active_ = false;
            jump_resume_pending_ = true;
            jump_ref_map_odom_ = map_odom;
            jump_anchor_map_pose_ = map_pose;
            jump_anchor_odom_pose_ = odom_pose;
            RCLCPP_INFO(get_logger(),
                        "[loc-jump] localization recovered after %.1fs stop-and-wait "
                        "(%.4f <= %.4f, d_xy=%.4fm d_theta=%.4frad): back to %s localization.",
                        localization_jump_stop_wait_duration_, disagree,
                        recover_threshold, d_xy, d_theta, path_frame_.c_str());
            return map_pose;
        }

        // ---- 正常期 ----
        if (!have_map_odom)
        {
            // 旁路本帧不可用：无从判定。基准保持不变——绝不能拿旧基准配新位姿去刷新，那等于
            // 把一帧还没判过的数据偷偷洗成"正常"。链路真静默了由上面的新鲜度看门狗来报。
            return have_map_pose ? map_pose : dr_pose;
        }

        if (disagree <= localization_jump_threshold_)
        {
            // 正常：两套基准一起滚动刷新。主路 TF 本帧失败时只刷新旁路基准，锚定对留在上一帧
            // ——递推撑住这几帧靠的正是它。
            jump_ref_map_odom_ = map_odom;
            if (have_map_pose)
            {
                jump_anchor_map_pose_ = map_pose;
                jump_anchor_odom_pose_ = odom_pose;
            }
            return have_map_pose ? map_pose : dr_pose;
        }

        // 触发跳变：两套基准**都不刷新**，就此冻结在跳变前的最后一帧，从这里开始纯 odom 递推。
        // 直接进强制停车等待，不设一级冻结窗口——那个窗口原本是为了给老判据争取"等车停稳再判"
        // 的时间（递推项会在刹车过程中持续累积漂移，判据不可信），新判据与车动不动无关，
        // 从触发那一刻起就一直有效。
        jump_stop_active_ = true;
        jump_stop_deadline_ =
            now_t + rclcpp::Duration::from_seconds(localization_jump_stop_wait_duration_);
        RCLCPP_ERROR(get_logger(),
                     "[loc-jump] %s localization jump %.4f > %.4f "
                     "(d_xy=%.4fm d_theta=%.4frad): stop and wait %.1fs before deciding. "
                     "map->odom now=(%.3f,%.3f,%.3f) ref=(%.3f,%.3f,%.3f)",
                     path_frame_.c_str(), disagree, localization_jump_threshold_,
                     d_xy, d_theta, localization_jump_stop_wait_duration_,
                     map_odom.x, map_odom.y, map_odom.theta,
                     jump_ref_map_odom_.x, jump_ref_map_odom_.y, jump_ref_map_odom_.theta);
        return dr_pose;
    }

    std::optional<std::pair<Pose2D, Velocity>> PathFollowerNode::getCurrentState(
        bool allow_localization_switch)
    {
        if (!have_odom_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No odometry received yet.");
            return std::nullopt;
        }
        // 注意：控制环里 guardOdomStale 用 odom_stale_stop_timeout_（默认 0.2s）先一步早退，
        // 所以从 update() 进来时这条 1.0s 的判定实际已经够不着了，留着是给 buildAndLoadRoute
        // 那条调用路径（allow_localization_switch=false）兜底——它不经过 guardOdomStale。
        const auto dt = (now() - last_odom_time_).seconds();
        if (dt > update_timeout_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry stale (%.2fs).", dt);
            return std::nullopt;
        }

        nav_msgs::msg::Odometry odom_snapshot;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_snapshot = latest_odom_;
        }
        Pose2D odom_pose;
        odom_pose.x = odom_snapshot.pose.pose.position.x;
        odom_pose.y = odom_snapshot.pose.pose.position.y;
        odom_pose.theta = yawFromQuaternion(odom_snapshot.pose.pose.orientation);

        Pose2D pose;

        if (path_frame_ == odom_frame_)
        {
            // odom 闭环：直接用里程计位姿。不经过 TF，跳变判据无从谈起，状态清掉。
            pose = odom_pose;
            resetLocalizationJumpGuard();
        }
        else if (allow_localization_switch && odom_anchor_active_)
        {
            // 已进入末端定位切换：纯用 odom 平滑递推，避开 map/TF 到点阶段的定位抖动。
            // 这期间不查 TF，跳变兜底的滚动锚定会失效（退出锚定后第一帧会拿到几秒前的
            // 基准而必然误判），所以在这里清掉，退出后由第一帧重新建立。
            pose = applyOdomAnchor(odom_pose);
            resetLocalizationJumpGuard();
        }
        else
        {
            // map 闭环：通过 TF 获取 path_frame → base_link 变换（主路，控制位姿来源）。
            // 定位跳变判据走的是另一条独立的 path_frame → odom 查询（旁路，见
            // guardLocalizationJump）。两条分开是刻意的：将来定位跳变问题从根上解决后，
            // 整条判据支路可以直接摘掉，主路这段一行都不用动。
            Pose2D map_pose;
            bool have_map_pose = false;
            try
            {
                const auto tf = tf_buffer_->lookupTransform(
                    path_frame_, base_frame_, tf2::TimePointZero,
                    tf2::durationFromSec(tf_lookup_timeout_));
                map_pose.x = tf.transform.translation.x;
                map_pose.y = tf.transform.translation.y;
                map_pose.theta = yawFromQuaternion(tf.transform.rotation);
                have_map_pose = true;
                tf_fail_count_ = 0;
            }
            catch (const tf2::TransformException& ex)
            {
                ++tf_fail_count_;
                // 超时压到 5ms 之后偶发失败是常态，一失败就放弃本帧会让 update() 发出一帧
                // 零速、造成速度断续抖动。锚定对已建立时可以用 odom 递推撑住几帧（跳变期
                // 本来就在这么做），但撑久了递推会漂，所以限帧。
                // 停车等待期不限帧：车已经停住，odom 增量归零，递推位姿退化成"跳变前的那个
                // 固定点"，撑多久都不会漂。
                const bool can_fallback =
                    enable_localization_jump_recovery_ && jump_ref_valid_ &&
                    (jump_stop_active_ || tf_fail_count_ <= max_tf_fallback_frames_);
                if (!can_fallback)
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                        "TF lookup %s->%s failed (%d consecutive): %s",
                        path_frame_.c_str(), base_frame_.c_str(), tf_fail_count_, ex.what());
                    return std::nullopt;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TF lookup %s->%s failed (%d/%d), keep dead-reckoning: %s",
                    path_frame_.c_str(), base_frame_.c_str(),
                    tf_fail_count_, max_tf_fallback_frames_, ex.what());
            }

            if (!enable_localization_jump_recovery_)
            {
                pose = map_pose; // 功能关闭：have_map_pose 必为 true（否则上面已返回）
                resetLocalizationJumpGuard();
            }
            else
            {
                pose = guardLocalizationJump(map_pose, have_map_pose, odom_pose);
                if (jump_abort_pending_)
                {
                    // 终止动作交给控制环统一处理（见 update()），这里只放弃本帧位姿。
                    return std::nullopt;
                }
            }

            // 末端定位切换：按当前所处阶段选用对应判据，锚定当前 (map, odom) 位姿对后
            // 改用 odom 平滑递推，消除到点阶段 map 定位抖动导致的速度忽正忽负。
            //   - omni 平移+旋转（单阶段，位置与朝向并行，不走三阶段状态机）：看距离。
            //   - kFollowTrack（阶段二，沿 route_->position 平移）：只看距离；
            //   - kAlignToPath/kAlignToGoal（阶段一、三，仅原地旋转、位置不动）：只看角度；
            // jump_stop_active_ 期间必须跳过：那段时间位姿仍在按 odom 正常递推（只是锚定对
            // 冻在跳变前那一帧），刹车过程车还会往前走一段，位姿真有可能跨过末端切换阈值。
            // 一旦激活 odom 锚定，下一帧 getCurrentState 就走锚定分支 ->
            // resetLocalizationJumpGuard() -> 停车状态被清掉，车带着未复位的速度记忆开走。
            if (!jump_stop_active_ && allow_localization_switch &&
                (end_localization_switch_distance_ > 0.0 || end_localization_switch_angle_ > 0.0))
            {
                std::lock_guard<std::mutex> lock(route_mutex_);
                if (route_ && !route_->position.points.empty())
                {
                    const auto& goal_point = route_->position.points.back();
                    const double dist_to_goal = std::hypot(pose.x - goal_point.x, pose.y - goal_point.y);
                    const double angle_to_goal = std::fabs(
                        angles::normalize_angle(route_->goal_theta - pose.theta));

                    const bool check_dist =
                        segment_.mode == TrackingMode::kOmniTransRotation ||
                        track_phase_ == TrackPhase::kFollowTrack;
                    const bool check_angle = !check_dist;

                    const bool trigger_by_dist =
                        check_dist &&
                        end_localization_switch_distance_ > 0.0 &&
                        dist_to_goal <= end_localization_switch_distance_;
                    const bool trigger_by_angle =
                        check_angle &&
                        end_localization_switch_angle_ > 0.0 &&
                        angle_to_goal <= end_localization_switch_angle_;
                    if (trigger_by_dist || trigger_by_angle)
                    {
                        odom_anchor_map_pose_ = pose;
                        odom_anchor_odom_pose_ = odom_pose;
                        odom_anchor_active_ = true;
                        RCLCPP_INFO(get_logger(),
                                    "Near goal (dist=%.3fm angle=%.3frad): switch pose reference "
                                    "map->odom to suppress localization jitter.",
                                    dist_to_goal, angle_to_goal);
                    }
                }
            }
        }

        // 速度始终从 odom twist 获取（body frame 下，与全局坐标系无关）
        Velocity vel;
        vel.vx = odom_snapshot.twist.twist.linear.x;
        vel.vy = odom_snapshot.twist.twist.linear.y;
        vel.wz = odom_snapshot.twist.twist.angular.z;
        return std::make_pair(pose, vel);
    }

    // ---------------------------------------------------------------------------
    // 控制环
    // ---------------------------------------------------------------------------
    std::size_t PathFollowerNode::findNearestIndex(
        const Pose2D& pose, std::size_t last_index) const
    {
        std::size_t index = last_index;
        double min_dist = std::numeric_limits<double>::max();
        const std::size_t n = route_->position.points.size();
        const std::size_t search_end = std::min(n, last_index + max_search_number_);
        for (std::size_t i = last_index; i < search_end; ++i)
        {
            const double dx = pose.x - route_->position.points[i].x;
            const double dy = pose.y - route_->position.points[i].y;
            const double d = std::hypot(dx, dy);
            if (d < min_dist)
            {
                min_dist = d;
                index = i;
            }
        }
        if (index == n - 1)
        {
            index = (n > 1) ? n - 2 : 0;
        }
        return index;
    }

    std::size_t PathFollowerNode::offsetPointIndex(
        const Velocity& v, double dt, std::size_t min_points,
        const Route& route, std::size_t current_index) const
    {
        const double linear_v = std::hypot(v.vx, v.vy);
        const double omega = v.wz;
        const double offset_dis = std::fabs(linear_v * dt);
        const double offset_s = (route.mode == RouteMode::kSpin) ? std::fabs(omega * dt) : offset_dis;
        const double init_s = route.points[current_index].current_s;
        const std::size_t last = route.points.size() - 1;

        for (std::size_t i = current_index; i < route.points.size(); ++i)
        {
            if (std::fabs(route.points[i].current_s - init_s) > offset_s)
            {
                std::size_t ret = i;
                if (ret - current_index < min_points)
                {
                    ret = current_index + min_points;
                }
                return std::min(ret, last);
            }
        }
        return last;
    }

    TrackingError PathFollowerNode::calcDelta(
        const Pose2D& current, const Pose2D& start, const Pose2D& aim) const
    {
        // 忠实复现 CalcLinearVectorDelta：对 start→aim 线段投影。
        TrackingError err;
        const double A = current.x - start.x;
        const double B = current.y - start.y;
        const double C = aim.x - start.x;
        const double D = aim.y - start.y;
        const double denom = C * C + D * D;
        double K = (denom > kEpsilon) ? (A * C + B * D) / denom : 0.0;

        const double proj_x = start.x + K * C;
        const double proj_y = start.y + K * D;

        // Δx：投影点到前视点的距离，超过为负
        double dist = std::hypot(aim.x - proj_x, aim.y - proj_y);
        if (K >= 1.0)
        {
            dist *= -1.0;
        }
        err.delta_x = dist;

        // Δy：当前点到线段的垂直距离，右侧正、左侧负
        dist = std::hypot(current.x - proj_x, current.y - proj_y);
        if (B * C - A * D >= 0.0)
        {
            // 点在路段左侧
            dist *= -1.0;
        }
        err.delta_y = dist;

        // Δθ：aim 朝向 - 当前航向，归一化（左偏为负、右偏为正）
        err.delta_theta = angles::normalize_angle(aim.theta - current.theta);
        return err;
    }

    void PathFollowerNode::velProjection(
        const Pose2D& current, std::size_t index, const Route& route,
        double& target_linear_v, double& target_w) const
    {
        std::size_t next_index = (index == 0) ? 1 : index + 1;
        std::size_t start_index = (index == 0) ? 0 : index - 1;
        if (next_index > route.points.size() - 1)
        {
            next_index = route.points.size() - 1;
        }

        const double A = current.x - route.points[start_index].x;
        const double B = current.y - route.points[start_index].y;
        const double C = route.points[next_index].x - route.points[start_index].x;
        const double D = route.points[next_index].y - route.points[start_index].y;
        const double denom = C * C + D * D;
        if (denom < 1e-12)
        {
            target_linear_v = route.points[start_index].linear_v;
            target_w = route.points[start_index].curvature_omega;
            return;
        }
        double K = (A * C + B * D) / denom;
        double ratio = clamp(K, 0.0, 1.0);
        target_linear_v = route.points[next_index].linear_v - route.points[start_index].linear_v;
        target_linear_v = target_linear_v * ratio + route.points[start_index].linear_v;
        target_w = route.points[next_index].curvature_omega - route.points[start_index].curvature_omega;
        target_w = target_w * ratio + route.points[start_index].curvature_omega;
    }

    std::size_t PathFollowerNode::findHeadingIndex(
        double current_theta, std::size_t last_index, const HeadingRoute& route) const
    {
        const auto& hp = route.points;
        const std::size_t n = hp.size();
        if (n == 0)
        {
            return 0;
        }
        const std::size_t lo = (last_index > heading_back_window_)
                                   ? last_index - heading_back_window_
                                   : 0;
        const std::size_t hi = std::min(n, last_index + max_search_number_);
        std::size_t best = std::min(last_index, n - 1);
        double best_d = std::numeric_limits<double>::max();
        for (std::size_t i = lo; i < hi; ++i)
        {
            // 存储朝向为 unwrap 单调，比较时对差值归一化即可兼容 ±π 跳变。
            const double d = std::fabs(angles::normalize_angle(hp[i].theta - current_theta));
            if (d < best_d)
            {
                best_d = d;
                best = i;
            }
        }
        return best;
    }

    void PathFollowerNode::headingProjection(
        double current_theta, std::size_t index, const HeadingRoute& route,
        double& target_heading_omega, double& target_theta) const
    {
        if (route.points.empty())
        {
            target_heading_omega = 0.0;
            target_theta = current_theta;
            return;
        }
        const std::size_t last = route.points.size() - 1;
        const std::size_t i0 = index;
        const std::size_t i1 = std::min(index + 1, last);
        if (i1 == i0)
        {
            target_heading_omega = route.points[i0].heading_omega;
            target_theta = route.points[i0].theta;
            return;
        }
        // 朝向序列已 unwrap 单调，步进无需归一化；当前朝向差归一化以兼容 ±π 跳变。
        const double step = route.points[i1].theta - route.points[i0].theta;
        const double d = angles::normalize_angle(current_theta - route.points[i0].theta);
        const double ratio = (std::fabs(step) > kEpsilon)
                                 ? clamp(d / step, 0.0, 1.0)
                                 : 0.0;
        target_heading_omega = route.points[i0].heading_omega +
            ratio * (route.points[i1].heading_omega - route.points[i0].heading_omega);
        target_theta = route.points[i0].theta + ratio * step;
    }

    bool PathFollowerNode::checkTrackDeviation(
        const Pose2D& pose, const Route& pos, std::size_t nearest_idx)
    {
        if (!enable_track_deviation_check_ || pos.points.empty())
        {
            return false;
        }
        nearest_idx = std::min(nearest_idx, pos.points.size() - 1);
        const auto& np = pos.points[nearest_idx];
        const double dev = std::hypot(pose.x - np.x, pose.y - np.y);
        if (dev <= track_deviation_limit_)
        {
            return false;
        }
        RCLCPP_ERROR(get_logger(),
                     "Track deviation too large: %.3fm > %.3fm (nearest_idx=%zu). "
                     "pose=(x=%.3f, y=%.3f, theta=%.3f), nearest_point=(x=%.3f, y=%.3f, heading_angle=%.3f). "
                     "Stopping and aborting trajectory.",
                     dev, track_deviation_limit_, nearest_idx,
                     pose.x, pose.y, pose.theta,
                     np.x, np.y, np.heading_angle);
        publishCmdVel(Velocity{});
        {
            std::lock_guard<std::mutex> lock(route_mutex_);
            route_.reset();
        }
        route_failed_ = true;
        {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "track deviation too large: %.3fm > %.3fm (nearest_idx=%zu), "
                          "pose=(%.3f, %.3f, %.3f), nearest_point=(%.3f, %.3f, %.3f)",
                          dev, track_deviation_limit_, nearest_idx,
                          pose.x, pose.y, pose.theta,
                          np.x, np.y, np.heading_angle);
            publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, buf);
        }
        return true;
    }

    void PathFollowerNode::logArrivalPose(
        const char* phase_tag, const Pose2D& pose,
        double target_x, double target_y, double target_theta) const
    {
        const double dx = pose.x - target_x;
        const double dy = pose.y - target_y;
        const double dist = std::hypot(dx, dy);
        const double dtheta = angles::normalize_angle(pose.theta - target_theta);
        // 把世界系残差旋到目标位姿坐标系：+x 为目标朝向前方（超调为正、未到为负），
        // +y 为目标朝向左侧。
        const double ct = std::cos(target_theta);
        const double st = std::sin(target_theta);
        const double ex = dx * ct + dy * st;
        const double ey = -dx * st + dy * ct;
        RCLCPP_INFO(get_logger(),
                    "[%s] arrival residual: "
                    "cur=(%.4f, %.4f, %.4f rad)  target=(%.4f, %.4f, %.4f rad)  "
                    "in_target_frame=(fwd=%.4fm, left=%.4fm, yaw=%.4f rad)  dist=%.4fm",
                    phase_tag,
                    pose.x, pose.y, pose.theta,
                    target_x, target_y, target_theta,
                    ex, ey, dtheta, dist);
    }

    double PathFollowerNode::followPath(
        const TrackingError& error, double v, double kpy, double /*kvy*/) const
    {
        // 忠实复现 FollowPath 非线性律（差速折叠）。
        const double t1 = std::cos(error.delta_theta / 2.0);
        // const double t2 = std::sin(error.delta_theta / 2.0);
        const double t3 = t1;
        const double t4 = kpy * t1 * t3 * error.delta_y;
        // const double t5 = t2 * (kvy * t1 * (t3 > 0.0 ? 1.0 : -1.0));
        // return std::fabs(v) * t3 * (t4 + t5);
        return std::fabs(v) * (t4);
    }

    Velocity PathFollowerNode::trajVelToAgvVel(
        const Velocity& traj_vel, double heading_angle, const Pose2D& current) const
    {
        // 忠实复现 trajVelToAgvVel：用 (当前航向 - 段航向) 旋转。
        Velocity agv;
        const double diff = current.theta - heading_angle;
        const double c = std::cos(diff);
        const double s = std::sin(diff);
        agv.vx = traj_vel.vx * c + traj_vel.vy * s;
        agv.vy = -traj_vel.vx * s + traj_vel.vy * c;
        agv.wz = traj_vel.wz;
        return agv;
    }

    void PathFollowerNode::publishCmdVel(const Velocity& v)
    {
        // 速度总出口。LifecyclePublisher 在 inactive 时本来就会丢弃消息，这里再显式挡一道。
        // 判的是 publisher 的激活状态而不是 active_：on_deactivate 要在失活 publisher
        // **之前**发一次零速，那一刻 active_ 已经是 false 了。
        if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated())
        {
            return;
        }
        geometry_msgs::msg::Twist twist;
        twist.linear.x = v.vx;
        twist.linear.y = v.vy;
        twist.angular.z = v.wz;
        cmd_vel_pub_->publish(twist);
        // odom 卡顿时的平滑刹停要从"上一帧真正发出去的车体系指令"等比衰减：那条路径上没有
        // 位姿，做不了轨迹系->车体系的旋转，只能拿已经旋好的这一帧来缩放。见 guardOdomStale。
        last_published_cmd_ = v;
    }

    void PathFollowerNode::applyMinCommandFloor(Velocity& v) const
    {
        const double vmin = planner_params_.min_linear_velocity;
        const double wmin = planner_params_.min_angular_velocity;
        const double speed = std::hypot(v.vx, v.vy);
        const double aw = std::fabs(v.wz);
        const bool has_lin = (speed > 0.0);
        const bool has_ang = (aw > 0.0);

        if (!has_lin && !has_ang)
        {
            return; // 停车帧，穿透
        }

        // 单维：仅线 / 仅角，直接抬到 min（无曲率可保）。
        if (has_lin && !has_ang)
        {
            if (speed < vmin)
            {
                const double k = vmin / speed;
                v.vx *= k;
                v.vy *= k;
            }
            return;
        }
        if (has_ang && !has_lin)
        {
            if (aw < wmin)
            {
                v.wz = std::copysign(wmin, v.wz);
            }
            return;
        }

        // 双维都非零：只在"都低于各自 min"时等比缩放，保持曲率 (vx:vy:wz)。
        // k = 1/max(rl, rw)：把"更接近阈值的那维"恰好抬到 min，另一维仍按比例，可能仍低于 min。
        const double rl = speed / vmin;
        const double rw = aw / wmin;
        if (rl < 1.0 && rw < 1.0)
        {
            const double k = 1.0 / std::max(rl, rw);
            v.vx *= k;
            v.vy *= k;
            v.wz *= k;
        }
    }

    void PathFollowerNode::publishDebugPath()
    {
        if (!route_ || route_->position.empty())
        {
            return;
        }
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = path_frame_;
        msg.poses.reserve(route_->position.points.size());
        for (const auto& p : route_->position.points)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, p.heading_angle);
            ps.pose.orientation = tf2::toMsg(q);
            msg.poses.push_back(ps);
        }
        debug_path_pub_->publish(msg);
    }

    void PathFollowerNode::publishNavigationStatus(int32_t status, const std::string& message)
    {
        nav_interfaces::msg::NavigationStatus msg;
        msg.status = status;
        msg.message = message;
        navigation_status_pub_->publish(msg);
    }

    void PathFollowerNode::resetRouteState()
    {
        {
            std::lock_guard<std::mutex> lock(route_mutex_);
            route_.reset();
            route_finished_ = false;
            route_failed_ = false;
            is_need_reset_ = true;
            track_phase_ = TrackPhase::kAlignToPath;
            resetVelocityTrack();
        }
        odom_anchor_active_ = false;
        resetLocalizationJumpGuard();
        odom_stale_stop_active_ = false;
    }

    void PathFollowerNode::onStopService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        RCLCPP_INFO(get_logger(), "Stop requested via ~/stop service. Resetting route.");
        resetRouteState();
        publishCmdVel(Velocity{});
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE, "stopped via ~/stop service");
        res->success = true;
        res->message = "stopped";
    }

    bool PathFollowerNode::guardOdomStale()
    {
        if (odom_stale_stop_timeout_ <= 0.0)
        {
            return false;
        }

        rclcpp::Time odom_recv;
        rclcpp::Time odom_stamp;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_recv = last_odom_time_;
            odom_stamp = last_odom_stamp_;
        }
        const auto now_t = now();
        const double age = (now_t - odom_recv).seconds();
        const bool odom_ok = (age <= odom_stale_stop_timeout_);

        // ---- 触发：只在从正常态跨进超时的那一帧设截止时刻，之后不续期 ----
        if (!odom_ok && !odom_stale_stop_active_)
        {
            odom_stale_stop_active_ = true;
            odom_stale_stop_deadline_ =
                now_t + rclcpp::Duration::from_seconds(odom_stale_stop_hold_duration_);
            // 注意这里打不出"当前帧"的时间戳——当前帧压根还没到，那正是年龄看门狗要判的事；
            // 两帧 stamp 的对比在 onOdom 里打（[odom-gap]）。两处日志分工互补。
            RCLCPP_ERROR(get_logger(),
                         "[odom-stale] no odom for %.3fs > %.3fs: braking and holding %.1fs. "
                         "last_msg_stamp=%.6f last_cmd=(vx=%.3f vy=%.3f wz=%.3f)",
                         age, odom_stale_stop_timeout_, odom_stale_stop_hold_duration_,
                         odom_stamp.seconds(), last_published_cmd_.vx,
                         last_published_cmd_.vy, last_published_cmd_.wz);
        }

        if (!odom_stale_stop_active_)
        {
            return false;
        }

        // ---- 恢复：必须同时满足"停够了"和"odom 已经新鲜" ----
        // 前者保证强制停车时长（卡顿往往成串出现，立刻复速会走走停停）；后者保证复速那一刻
        // 手里的位姿是新的。odom 迟迟不恢复时窗口早已过期，此时复速条件就退化成"等 odom"，
        // 一恢复即走，不会再白等一个完整窗口。
        if (now_t >= odom_stale_stop_deadline_ && odom_ok)
        {
            odom_stale_stop_active_ = false;
            // 这里不需要显式清零 last_track_*：强制停车窗口内每帧都在递推，到这一刻它们
            // 已经收敛到 0 附近，kResume 自然从 0 起步。（定位跳变那边必须手动清零，因为
            // 它的停车分支是直接发 0 速、根本没维护过 last_track_*。）
            vel_track_mode_ = VelocityTrackMode::kResume;
            // 卡顿期间误差已经变了，增量式 PID 的累积量必须清。
            is_need_reset_ = true;
            // 早退期间 last_control_update_time_ 没有维护，不复位会误报一条"控制频率过低"。
            have_control_update_time_ = false;
            RCLCPP_INFO(get_logger(),
                        "[odom-stale] hold finished and odom fresh (age=%.3fs): "
                        "resume from v=%.3f w=%.3f.",
                        age, last_track_linear_v_, last_track_angular_w_);
            return false;
        }

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[odom-stale] holding, %.1fs remaining (odom age=%.3fs, fresh=%d).",
                             std::max(0.0, (odom_stale_stop_deadline_ - now_t).seconds()),
                             age, static_cast<int>(odom_ok));

        // 平滑刹停。刻意不进 getCurrentState/PID/坐标变换：位姿已经过期，拿它算纠偏
        // 只会得到错误的修正量。做法是把上一帧真正发出去的车体系指令按两阶段步进模型
        // 衰减到 0——线速度和角速度各走各的通道，与正常控制路径一致。
        //
        // vx/vy 共用线速度通道的比值而不是各自归零：全向底盘直线运动时 vy 可能才是主
        // 分量，单独归零会让车横向甩一下；共用比值则平移方向保持不变，只是模长收敛。
        // wz 必须用角速度通道自己的比值——原地旋转阶段线速度本来就是 0，套用线速度比值
        // 会让角速度一步归零，平滑刹停退化成硬停（实测验证过）。
        //
        // cap 传上一帧输出而不是 0：stepTracked*Velocity 末尾会用 route_ref 给结果封顶，
        // 传 0 会把递推值直接砍成 0，同样退化成硬停。
        const double v_prev = last_track_linear_v_;
        const double w_prev = last_track_angular_w_;
        vel_track_mode_ = VelocityTrackMode::kStop;
        const double v_new = stepTrackedLinearVelocity(v_prev);
        const double w_new = stepTrackedAngularVelocity(w_prev);
        const double lin_scale = (std::fabs(v_prev) > kEpsilon)
                                     ? std::clamp(v_new / v_prev, 0.0, 1.0)
                                     : 0.0;
        const double ang_scale = (std::fabs(w_prev) > kEpsilon)
                                     ? std::clamp(w_new / w_prev, 0.0, 1.0)
                                     : 0.0;

        Velocity v = last_published_cmd_;
        v.vx *= lin_scale;
        v.vy *= lin_scale;
        v.wz *= ang_scale;
        publishCmdVel(v);
        return true;
    }

    void PathFollowerNode::update()
    {
        if (!have_odom_ || !route_.has_value())
        {
            // 无轨迹或无里程计 -> 停车
            // publishCmdVel(Velocity{});
            return;
        }

        // odom 接收年龄看门狗：位姿已经过期时先把车刹住，不必再去查 TF、跑判据和 PID。
        // 放在 route_ 检查之后——没路线本来就不动，不需要刹车。
        if (guardOdomStale())
        {
            return;
        }

        auto state_opt = getCurrentState();
        if (!state_opt)
        {
            publishCmdVel(Velocity{});
            if (jump_abort_pending_)
            {
                // 强制停车等满仍未恢复：定位已经不可信，终止本次导航。
                // 放在这里而不是 getCurrentState 里，是因为 buildAndLoadRoute 也会调
                // getCurrentState，在取位姿的路径上反手复位 route_ 在锁序上不安全。
                {
                    std::lock_guard<std::mutex> lock(route_mutex_);
                    route_.reset();
                }
                route_failed_ = true;
                resetVelocityTrack();
                char buf[200];
                std::snprintf(buf, sizeof(buf),
                              "localization jump unrecovered after %.1fs stop-and-wait "
                              "(threshold %.3fm)",
                              localization_jump_stop_wait_duration_, localization_jump_threshold_);
                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FAILED, buf);
                resetLocalizationJumpGuard();
            }
            return;
        }

        // 定位跳变强制停车期：直接发 0 速后返回，刻意不进参考速度/PID/障碍状态机那一套。
        // 位姿仍在正常采集（等满后的恢复判定要用它），只是不参与控制。
        // 实际减速曲线由下游 velocity_smoother 的限幅决定，本节点不再介入。
        if (jump_stop_active_)
        {
            publishCmdVel(Velocity{});
            return;
        }

        auto [pose, vel] = *state_opt;

        if (jump_resume_pending_)
        {
            jump_resume_pending_ = false;
            // 上面那条早退跳过了 last_track_* 的维护，它们还停在跳变那一刻的值。这里必须
            // 清零，否则 kResume 第一帧就 stepVelocityTowards(跳变前速度, ref)，输出直接
            // 是原速——车从静止一步蹦回去，比不做恢复更糟。
            // （与 guardOdomStale 的恢复分支正好相反：那边车还在动，清零才是制造跳变。）
            // 也不能图省事调 resetVelocityTrack()：它会把模式设成 kFollowRef，那是直接输出
            // 参考速度，同样是一步跳变，只是换了个姿势。
            last_track_linear_v_ = 0.0;
            last_track_angular_w_ = 0.0;
            vel_track_mode_ = VelocityTrackMode::kResume;
            // 增量式 PID 的输出累积量里还留着停车前的旧值，而误差已经变了，必须清。
            is_need_reset_ = true;
            // 停车期间控制环一直在早退，last_control_update_time_ 停在几十秒前，不复位的话
            // 恢复后第一帧必然误报一条"控制频率过低"。
            have_control_update_time_ = false;
            RCLCPP_INFO(get_logger(),
                        "[loc-jump] stop-and-wait finished, localization recovered: "
                        "resume from zero velocity via kResume.");
        }

        updateVelocityTrackMode();

        std::optional<PlannedRoute> route_local;
        SegmentProfile segment_local;
        {
            std::lock_guard<std::mutex> lock(route_mutex_);
            route_local = route_;
            segment_local = segment_;
        }
        if (!route_local || route_local->position.empty())
        {
            publishCmdVel(Velocity{});
            return;
        }

        // 控制周期超限检测（放在 early return 之后，只在真正执行控制时检测）
        const auto now_steady = std::chrono::steady_clock::now();
        if (have_control_update_time_)
        {
            const double expected_dt = 1.0 / std::max(control_rate_, 1e-6);
            const double actual_dt = std::chrono::duration<double>(now_steady - last_control_update_time_).count();
            if (actual_dt > expected_dt * 1.2)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Control loop frequency too low: target=%.2fHz actual=%.2fHz (target_dt=%.4fs actual_dt=%.4fs)",
                    control_rate_, 1.0 / std::max(actual_dt, 1e-6), expected_dt, actual_dt);
            }
        }
        last_control_update_time_ = now_steady;
        have_control_update_time_ = true;

        if (is_need_reset_)
        {
            pid_.reset();
            is_need_reset_ = false;
            RCLCPP_INFO_ONCE(get_logger(), "PID reset for new trajectory.");
        }

        // 按段级 mode 分派：diff/omni平移 三阶段状态机；omni平移+旋转 单阶段双路并行。
        switch (segment_local.mode)
        {
        case TrackingMode::kDiff:
            updateDiff(pose, vel, *route_local, segment_local, now_steady);
            break;
        case TrackingMode::kOmniTranslation:
            updateOmniTranslation(pose, vel, *route_local, segment_local, now_steady);
            break;
        case TrackingMode::kOmniTransRotation:
            updateOmni(pose, vel, *route_local, segment_local, now_steady);
            break;
        }
    }

    // ---------------------------------------------------------------------------
    // omni：位置与朝向并行跟踪（单阶段，与解耦重构前等价）
    // ---------------------------------------------------------------------------
    void PathFollowerNode::updateOmni(
        const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
        const SegmentProfile& segment,
        const std::chrono::steady_clock::time_point& /*now_steady*/)
    {
        const Route& pos = route.position;
        const HeadingRoute& hed = route.heading_align;

        // ---------------- 位置维度 ----------------
        // 1) 找最近点（禁后退）
        std::size_t index = findNearestIndex(pose, last_index_);
        if (index < last_index_)
        {
            index = last_index_;
        }
        last_index_ = index;
        nearest_index_ = index;

        // 跟踪偏差检测：车到最近点距离超限 -> 发零速、中止轨迹。
        if (checkTrackDeviation(pose, pos, index))
        {
            return;
        }

        // 2) 前视点
        const double dt = 1.0 / control_rate_;
        std::size_t t_index = offsetPointIndex(vel, dt, tracking_num_, pos, index);
        track_index_ = t_index;

        // 3) 参考线速度（速度投影）+ 曲率减速已在规划阶段处理
        double ref_v = 0.0, ref_w = 0.0;
        velProjection(pose, index, pos, ref_v, ref_w);
        // 段限速：NaN=默认上限，不限制。
        if (!std::isnan(segment.max_speed) && std::fabs(ref_v) > segment.max_speed)
        {
            ref_v = std::copysign(segment.max_speed, ref_v);
        }
        // 障碍减速/停止/恢复：改由两阶段步进模型基于上一帧速度实时递推（不再重规划轨迹）。
        // ref_w 是曲率前馈（w = v·curvature），必须跟着 ref_v 等比缩放才能保持转弯半径。
        {
            const double route_ref_v = ref_v;
            ref_v = stepTrackedLinearVelocity(route_ref_v);
            ref_w = (std::fabs(route_ref_v) > kEpsilon) ? ref_w * (ref_v / route_ref_v) : 0.0;
        }

        // 4) 位置误差（aim 朝向无意义，置 0；航向误差由朝向维度统一给出）
        const std::size_t last = pos.points.size() - 1;
        const std::size_t safe_idx = std::max<std::size_t>(0, index);
        Pose2D start_pose{pos.points[safe_idx].x, pos.points[safe_idx].y};
        Pose2D aim_pose{pos.points[t_index].x, pos.points[t_index].y};

        // 终点延长（仅位置 x/y；朝向由朝向维度统一给出）
        if (t_index >= last)
        {
            aim_pose.x = pos.points.back().x + end_extended_distance_ * std::cos(pos.points.back().heading_angle);
            aim_pose.y = pos.points.back().y + end_extended_distance_ * std::sin(pos.points.back().heading_angle);
        }

        TrackingError error = calcDelta(pose, start_pose, aim_pose);
        // pos_overshoot：越过终点超出容差（定位抖动/惯性导致）。参考速度前馈恒为正，
        // 若此时仍判定"到点"并放弃位置纠偏，抖动会让车越冲越远；因此不算到点，反向修正拉回容差。
        const bool pos_overshoot = (error.delta_x < -reach_distance_);
        const bool pos_done = !pos_overshoot && (error.delta_x <= reach_distance_);
        if (pos_overshoot)
        {
            ref_v = -ref_v;
        }
        else if (pos_done)
        {
            ref_v = 0.0; // 位置已到、朝向未到：停平移，继续旋转
        }

        // ---------------- 朝向维度（独立索引 + 梯形前馈投影）----------------
        const double current_s = pos.points[safe_idx].current_s;
        const bool heading_locked = enable_heading_lock_ &&
                                    current_s < heading_lock_distance_ &&
                                    !pos_done;
        // 保持朝向平移：mode 3 且无 target_yaw 时退化（mode 2 已走 updateOmniTranslation，不会到此）。
        const bool hold_heading = !segment.has_target_yaw;
        double ref_heading_omega = 0.0;
        double target_theta;
        bool head_done;
        if (hold_heading)
        {
            // 无 target_yaw：保持当前朝向，无末段旋转。
            target_theta = pose.theta;
            ref_heading_omega = 0.0;
            head_done = true;
        }
        else
        {
            target_theta = heading_locked ? pose.theta : route.goal_theta;
            if (!hed.empty() && !heading_locked)
            {
                const std::size_t h_index = findHeadingIndex(pose.theta, last_heading_index_, hed);
                last_heading_index_ = h_index;
                heading_index_ = h_index;
                headingProjection(pose.theta, h_index, hed, ref_heading_omega, target_theta);
            }

            head_done = !heading_locked &&
                        (hed.empty() || heading_index_ + 1 >= hed.points.size());
            if (head_done)
            {
                ref_heading_omega = 0.0; // 朝向已到：旋转前馈归零
            }
        }
        // 朝向维度是独立的旋转速度规划，走自己的步进通道（与线速度通道共享 vel_track_mode_）。
        ref_heading_omega = stepTrackedAngularVelocity(ref_heading_omega);

        // 航向误差锚定在朝向维度（target_theta - 当前航向，归一化）
        error.delta_theta = angles::normalize_angle(target_theta - pose.theta);

        // 5) 双到点：位置与朝向都到才结束
        //    hold_heading 时目标朝向=hold_direction（或当前），否则=goal_theta。
        const double completion_theta = hold_heading ? target_theta : route.goal_theta;
        if (pos_done &&
            head_done && std::fabs(angles::normalize_angle(completion_theta - pose.theta)) <= reach_angle_)
        {
            route_finished_ = true;
            logArrivalPose("omni-trans phase3->goal",
                           pose,
                           pos.points.back().x, pos.points.back().y,
                           completion_theta);
            {
                std::lock_guard<std::mutex> lock(route_mutex_);
                route_.reset();
            }
            pid_.reset();
            publishCmdVel(Velocity{});
            publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED,
                                  "trajectory reached (pos+heading done)");
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Trajectory reached. Stopping.");
            return;
        }

        // 6) wz = 角速度梯形前馈 + 朝向比例纠偏
        double wz = ref_heading_omega + kp_heading_ * error.delta_theta;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        // 7) PID 横向纠偏（omni 有 vy；朝向已由上式独立处理，PID 的 theta 增量环弃用）
        CorrectionVelocity corr = pid_.compute(
            error, std::hypot(vel.vx, vel.vy), ref_v, vel.wz,
            true, diff_lateral_gain_, max_percentage_vy_to_vx_);

        // 8) 合成轨迹坐标系速度
        Velocity traj_v;
        traj_v.vx = ref_v; // 纵向：参考速度前馈
        traj_v.vy = corr.vy; // 横向纠偏
        traj_v.wz = wz; // 朝向：梯形前馈 + 比例纠偏

        // 9) 低速 Vy/Vx 比例限幅
        if (std::fabs(traj_v.vx) > kEpsilon && std::fabs(traj_v.vy) > kEpsilon)
        {
            const double ratio_max = max_percentage_vy_to_vx_;
            double ratio = 0.3;
            if (std::fabs(traj_v.vx) < 0.15)
            {
                ratio = std::pow(std::fabs(traj_v.vx) / 0.15, 2) * ratio_max;
                if (ratio < 0.1 * ratio_max) ratio = 0.1 * ratio_max;
            }
            const double ratio_actual = std::fabs(traj_v.vy / traj_v.vx);
            const double ratio_use = std::min(ratio, ratio_actual);
            traj_v.vy = std::copysign(ratio_use * std::fabs(traj_v.vx), traj_v.vy);
        }

        // 10) 轨迹坐标系 -> 车体坐标系（位置维度的 heading_angle 旋转平移分量）
        const double heading = pos.points[index].heading_angle;
        Velocity out = trajVelToAgvVel(traj_v, heading, pose);

        // STOP 悬停兜底：参考速度已经是 0 且当前处于障碍 STOP 状态，直接清零最终输出。
        // 不管前面横向 PID / 朝向比例纠偏算出了什么残留量，STOP 时车必须真正静止。
        if (vel_track_mode_ == VelocityTrackMode::kStop && std::fabs(ref_v) <= kEpsilon)
        {
            out = Velocity{};
            // 抗积分饱和：输出整体被否决了，横向纠偏 PID 就不能再把这份消不掉的
            // 误差往积分里堆。否则长时间避障停车后，攒下的 vy 会在恢复行驶时被
            // Vy/Vx 占比限幅按上限释放，把车横推出 track_deviation_limit 直接 abort。
            // 实测 2026-09-18 W1-217：停 65 s 攒到 0.209 m/s，恢复 2 s 后横推 0.146 m。
            pid_.reset();
        }

        applyMinCommandFloor(out);
        publishCmdVel(out);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "idx=%zu/%zu h_idx=%zu/%zu s=%.3f lock=%d err=(x%.3f,y%.3f,th%.3f) "
            "ref=(v%.2f,w%.2f,hw%.3f) traj=(%.2f,%.3f,%.3f) out=(%.3f,%.3f,%.3f)",
            index, last, heading_index_, hed.size(),
            current_s, heading_locked ? 1 : 0,
            error.delta_x, error.delta_y, error.delta_theta,
            ref_v, ref_w, ref_heading_omega,
            traj_v.vx, traj_v.vy, traj_v.wz,
            out.vx, out.vy, out.wz);
    }

    // ---------------------------------------------------------------------------
    // omni 平移（mode 2）：三阶段状态机（与 diff 同构，阶段一/三目标不同）
    //   阶段一：原地旋转到 hold_direction（保持朝向）。
    //   阶段二：沿轨迹平移，车体朝向锁定 hold_direction（全向有 vy 横向纠偏）。
    //   阶段三：到点后旋转到 target_yaw（复用 diffPhaseAlignToGoal）。
    // ---------------------------------------------------------------------------
    void PathFollowerNode::updateOmniTranslation(
        const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
        const SegmentProfile& segment,
        const std::chrono::steady_clock::time_point& /*now_steady*/)
    {
        Velocity out;
        switch (track_phase_)
        {
        case TrackPhase::kAlignToPath:
            out = omniTransPhaseAlignHold(pose, route, segment);
            break;
        case TrackPhase::kFollowTrack:
            out = omniTransPhaseFollowTrack(pose, vel, route, segment);
            break;
        case TrackPhase::kAlignToGoal:
            out = diffPhaseAlignToGoal(pose, route, segment); // 复用：旋转到 target_yaw
            break;
        }
        applyMinCommandFloor(out);
        publishCmdVel(out);
    }

    // 阶段一：原地旋转到 hold_direction。
    // 同 diffPhaseAlignToPath：反馈锚定 headingProjection 插值目标，完成判定锚定固定 hold_direction。
    Velocity PathFollowerNode::omniTransPhaseAlignHold(
        const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& segment)
    {
        // 全流程偏差检测：阶段一原地旋转期间车应仍在轨迹起点附近，偏差超限则中止（同阶段二）。
        if (checkTrackDeviation(pose, route.position, 0))
        {
            return Velocity{};
        }

        const HeadingRoute& hed = route.heading_align;
        // hold_direction 已在 buildAndLoadRoute 解析为具体值（NaN -> start_theta）。
        const double goal_theta = segment.hold_direction;
        double ref_heading_omega = 0.0;
        double target_theta = goal_theta;
        if (!hed.empty())
        {
            const std::size_t h_index = findHeadingIndex(pose.theta, last_heading_index_, hed);
            last_heading_index_ = h_index;
            heading_index_ = h_index;
            headingProjection(pose.theta, h_index, hed, ref_heading_omega, target_theta);
        }

        // 已转过目标朝向超出容差（定位抖动/惯性导致）：前馈方向与实际需要的纠偏方向相反，
        // 反向而非继续按原方向转，避免越冲后仍用原方向前馈把车转得更远。
        const double goal_err = angles::normalize_angle(goal_theta - pose.theta);
        // if (std::fabs(goal_err) > reach_angle_ && ref_heading_omega * goal_err < 0.0)
        // {
        //     ref_heading_omega = -ref_heading_omega;
        // }
        // 障碍减速/停止/恢复：原地旋转阶段同样受控（以前靠 STOP 重规划 hed 实现，现由模型逐帧递推）。
        ref_heading_omega = stepTrackedAngularVelocity(ref_heading_omega);

        const double err_theta = angles::normalize_angle(target_theta - pose.theta);
        double wz = ref_heading_omega + kp_heading_ * err_theta;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        if (std::fabs(goal_err) <= reach_angle_)
        {
            track_phase_ = TrackPhase::kFollowTrack;
            last_index_ = 0;
            nearest_index_ = 0;
            track_index_ = 0;
            pid_.reset();
            odom_anchor_active_ = false; // 阶段切换：重新按新阶段判据评估末端定位切换
            RCLCPP_INFO(get_logger(),
                        "omni-trans phase1 done (hold=%.4f goal_err=%.4f rad) -> phase2 follow track.",
                        goal_theta, goal_err);
            wz = 0.0;
        }

        Velocity out;
        out.vx = 0.0;
        out.vy = 0.0;
        out.wz = wz;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "omni-trans[phase1 align->hold] h_idx=%zu/%zu goal=%.4f target=%.4f cur=%.4f err=%.4f "
            "goal_err=%.4f hw=%.3f wz=%.3f",
            heading_index_, hed.size(), goal_theta, target_theta, pose.theta, err_theta,
            goal_err, ref_heading_omega, wz);
        return out;
    }

    // 阶段二：沿轨迹平移，车体朝向锁定 hold_direction。
    // 位置跟踪同 updateOmni（全向 vy 横向纠偏）；朝向不跟随轨迹，仅按 hold_direction 比例保持。
    Velocity PathFollowerNode::omniTransPhaseFollowTrack(
        const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
        const SegmentProfile& segment)
    {
        const Route& pos = route.position;

        // 1) 找最近点（禁后退）
        std::size_t index = findNearestIndex(pose, last_index_);
        if (index < last_index_)
        {
            index = last_index_;
        }
        last_index_ = index;
        nearest_index_ = index;

        // 跟踪偏差检测
        if (checkTrackDeviation(pose, pos, index))
        {
            return Velocity{};
        }

        // 2) 前视点
        const double dt = 1.0 / control_rate_;
        std::size_t t_index = offsetPointIndex(vel, dt, tracking_num_, pos, index);
        track_index_ = t_index;

        // 3) 参考线速度 + 段限速
        double ref_v = 0.0, ref_w = 0.0;
        velProjection(pose, index, pos, ref_v, ref_w);
        (void)ref_w; // 平移模式朝向不跟随曲率，ref_w 不用
        if (!std::isnan(segment.max_speed) && std::fabs(ref_v) > segment.max_speed)
        {
            ref_v = std::copysign(segment.max_speed, ref_v);
        }
        // 障碍减速/停止/恢复：两阶段步进模型逐帧递推（详见 stepTrackedLinearVelocity）。
        ref_v = stepTrackedLinearVelocity(ref_v);

        // 4) 位置误差
        const std::size_t last = pos.points.size() - 1;
        const std::size_t safe_idx = std::max<std::size_t>(0, index);
        Pose2D start_pose{pos.points[safe_idx].x, pos.points[safe_idx].y};
        Pose2D aim_pose{pos.points[t_index].x, pos.points[t_index].y};
        if (t_index >= last)
        {
            aim_pose.x = pos.points.back().x + end_extended_distance_ * std::cos(pos.points.back().heading_angle);
            aim_pose.y = pos.points.back().y + end_extended_distance_ * std::sin(pos.points.back().heading_angle);
        }
        TrackingError error = calcDelta(pose, start_pose, aim_pose);
        // pos_overshoot：越过终点超出容差（定位抖动/惯性导致），反向修正拉回而非判定到点。
        const bool pos_overshoot = (error.delta_x < -reach_distance_);
        const bool pos_done = !pos_overshoot && (error.delta_x <= reach_distance_);
        if (pos_overshoot)
        {
            ref_v = -ref_v;
        }
        else if (pos_done)
        {
            ref_v = 0.0;
        }

        // 5) 朝向锁定 hold_direction（无前馈，仅比例保持）
        const double target_theta = segment.hold_direction;
        error.delta_theta = angles::normalize_angle(target_theta - pose.theta);
        double wz = kp_heading_ * error.delta_theta;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        // 6) PID 横向纠偏（omni vy；朝向已由上式独立处理）
        CorrectionVelocity corr = pid_.compute(
            error, std::hypot(vel.vx, vel.vy), ref_v, vel.wz,
            true, diff_lateral_gain_, max_percentage_vy_to_vx_);

        Velocity traj_v;
        traj_v.vx = ref_v;
        traj_v.vy = corr.vy;
        traj_v.wz = wz;

        // 7) 低速 Vy/Vx 比例限幅
        if (std::fabs(traj_v.vx) > kEpsilon && std::fabs(traj_v.vy) > kEpsilon)
        {
            const double ratio_max = max_percentage_vy_to_vx_;
            double ratio = 0.3;
            if (std::fabs(traj_v.vx) < 0.15)
            {
                ratio = std::pow(std::fabs(traj_v.vx) / 0.15, 2) * ratio_max;
                if (ratio < 0.1 * ratio_max) ratio = 0.1 * ratio_max;
            }
            const double ratio_actual = std::fabs(traj_v.vy / traj_v.vx);
            const double ratio_use = std::min(ratio, ratio_actual);
            traj_v.vy = std::copysign(ratio_use * std::fabs(traj_v.vx), traj_v.vy);
        }

        // 8) 轨迹坐标系 -> 车体坐标系（位置切向旋转平移分量）
        const double heading = pos.points[index].heading_angle;
        Velocity out = trajVelToAgvVel(traj_v, heading, pose);

        // STOP 悬停兜底：参考速度已经是 0 且当前处于障碍 STOP 状态，直接清零最终输出。
        // 不管前面横向 PID / 朝向比例纠偏算出了什么残留量，STOP 时车必须真正静止。
        if (vel_track_mode_ == VelocityTrackMode::kStop && std::fabs(ref_v) <= kEpsilon)
        {
            out = Velocity{};
            // 抗积分饱和：输出整体被否决了，横向纠偏 PID 就不能再把这份消不掉的
            // 误差往积分里堆。否则长时间避障停车后，攒下的 vy 会在恢复行驶时被
            // Vy/Vx 占比限幅按上限释放，把车横推出 track_deviation_limit 直接 abort。
            // 实测 2026-09-18 W1-217：停 65 s 攒到 0.209 m/s，恢复 2 s 后横推 0.146 m。
            pid_.reset();
        }

        // 9) 到点判定：位置到 + 朝向已对准 hold_direction
        if (pos_done && std::fabs(angles::normalize_angle(target_theta - pose.theta)) <= reach_angle_)
        {
            out = Velocity{}; // 切换瞬间停车，避免带入阶段三
            if (segment.has_target_yaw)
            {
                track_phase_ = TrackPhase::kAlignToGoal;
                heading_index_ = 0;
                last_heading_index_ = 0;
                pid_.reset();
                odom_anchor_active_ = false; // 阶段切换：重新按新阶段判据评估末端定位切换
                RCLCPP_INFO(get_logger(),
                            "omni-trans phase2 done (reached, err_x=%.4f m) -> phase3 align->goal.",
                            error.delta_x);
                logArrivalPose("omni-trans phase2->pos",
                               pose,
                               pos.points.back().x, pos.points.back().y,
                               target_theta);
            }
            else
            {
                // 到点即停，无末段旋转（无 target_yaw）
                route_finished_ = true;
                {
                    std::lock_guard<std::mutex> lock(route_mutex_);
                    route_.reset();
                }
                pid_.reset();
                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED,
                                      "omni-trans phase2 done, no target_yaw, skip phase3");
                RCLCPP_INFO(get_logger(),
                            "omni-trans phase2 done (reached, no target_yaw) -> stop, skip phase3.");
            }
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "omni-trans[phase2 follow] idx=%zu/%zu s=%.3f err=(x%.3f,y%.3f,th%.3f) "
            "ref=(v%.2f) traj=(%.2f,%.3f,%.3f) out=(%.3f,%.3f,%.3f)",
            index, last, pos.points[safe_idx].current_s,
            error.delta_x, error.delta_y, error.delta_theta,
            ref_v, traj_v.vx, traj_v.vy, traj_v.wz,
            out.vx, out.vy, out.wz);
        return out;
    }

    // ---------------------------------------------------------------------------
    // diff：三阶段状态机
    // ---------------------------------------------------------------------------
    void PathFollowerNode::updateDiff(
        const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
        const SegmentProfile& segment,
        const std::chrono::steady_clock::time_point& /*now_steady*/)
    {
        Velocity out;
        switch (track_phase_)
        {
        case TrackPhase::kAlignToPath:
            out = diffPhaseAlignToPath(pose, route, segment);
            break;
        case TrackPhase::kFollowTrack:
            out = diffPhaseFollowTrack(pose, vel, route, segment);
            break;
        case TrackPhase::kAlignToGoal:
            out = diffPhaseAlignToGoal(pose, route, segment);
            break;
        }
        applyMinCommandFloor(out);
        publishCmdVel(out);
    }

    // 阶段一：原地旋转对准轨迹起始切向。
    // 反馈误差锚定在 headingProjection 的插值目标 target_theta（当前位置投影朝向 - 车实际朝向），
    // heading_align 提供梯形前馈；完成判定单独锚定固定目标 path_start_heading。
    // （完成判定不能用投影 target：因 hp[0].theta=start_theta≈当前朝向，h_idx=0 时误差≈0，
    //  会在第一个周期误判完成——车根本没转。）
    Velocity PathFollowerNode::diffPhaseAlignToPath(
        const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& segment)
    {
        // 全流程偏差检测：阶段一原地旋转期间车应仍在轨迹起点附近，偏差超限则中止（同阶段二）。
        if (checkTrackDeviation(pose, route.position, 0))
        {
            return Velocity{};
        }

        const HeadingRoute& hed = route.heading_align;
        const double goal_theta = segment.enable_reverse
            ? angles::normalize_angle(route.path_start_heading + M_PI)
            : route.path_start_heading;
        double ref_heading_omega = 0.0;
        double target_theta = goal_theta;
        if (!hed.empty())
        {
            const std::size_t h_index = findHeadingIndex(pose.theta, last_heading_index_, hed);
            last_heading_index_ = h_index;
            heading_index_ = h_index;
            headingProjection(pose.theta, h_index, hed, ref_heading_omega, target_theta);
        }

        // 阶段一完成判定：锚定固定目标 goal_theta（容差复用 reach_angle_）。
        const double goal_err = angles::normalize_angle(goal_theta - pose.theta);
        // 越冲纠偏判断仅在朝向轨迹已推进到末尾附近时才生效：goal_theta-pose.theta
        // 在旋转刚起步、且起止朝向接近对跖（相差近 π，如本例 start≈-1.57 goal≈1.57）时，
        // 会落在 normalize_angle 的 ±π 裂缝两侧，符号对起始位姿噪声极敏感，不能当作
        // "已转过头"的证据；只有车确已转到规划终点附近（heading_index_ 到达 hed 末尾），
        // goal_err 才真正反映"是否被惯性/抖动带过了目标"，此时符号才可信。
        const bool near_plan_end = !hed.empty() && (heading_index_ + 1 >= hed.size());
        if (near_plan_end && std::fabs(goal_err) > reach_angle_ && ref_heading_omega * goal_err < 0.0)
        {
            ref_heading_omega = -ref_heading_omega;
        }
        // 障碍减速/停止/恢复：原地旋转阶段同样受控（以前靠 STOP 重规划 hed 实现，现由模型逐帧递推）。
        ref_heading_omega = stepTrackedAngularVelocity(ref_heading_omega);

        // 反馈误差：投影目标朝向 - 车实际朝向（归一化）。
        const double err_theta = angles::normalize_angle(target_theta - pose.theta);
        // double wz = ref_heading_omega + kp_heading_ * err_theta;
        double wz = ref_heading_omega;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        if (std::fabs(goal_err) <= reach_angle_)
        {
            track_phase_ = TrackPhase::kFollowTrack;
            // 进入跟随：重置位置索引，从车头正对方向开始跟踪。
            last_index_ = 0;
            nearest_index_ = 0;
            track_index_ = 0;
            pid_.reset();
            odom_anchor_active_ = false; // 阶段切换：重新按新阶段判据评估末端定位切换
            RCLCPP_INFO(get_logger(),
                        "diff phase1 done (goal=%.4f goal_err=%.4f rad) -> phase2 follow track.",
                        goal_theta, goal_err);
            wz = 0.0; // 阶段切换瞬间停旋，避免带入跟随阶段
        }

        Velocity out;
        out.vx = 0.0;
        out.vy = 0.0;
        out.wz = wz;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "diff[phase1 align->path] h_idx=%zu/%zu goal=%.4f target=%.4f cur=%.4f err=%.4f "
            "goal_err=%.4f hw=%.3f wz=%.3f",
            heading_index_, hed.size(), goal_theta, target_theta, pose.theta, err_theta,
            goal_err, ref_heading_omega, wz);
        return out;
    }

    // 阶段二：沿轨迹跟随，仅输出 vx（vy≡0）；航向跟随位置轨迹切向。
    Velocity PathFollowerNode::diffPhaseFollowTrack(
        const Pose2D& pose, const Velocity& vel, const PlannedRoute& route,
        const SegmentProfile& segment)
    {
        const Route& pos = route.position;

        // 1) 找最近点（禁后退）
        std::size_t index = findNearestIndex(pose, last_index_);
        if (index < last_index_)
        {
            index = last_index_;
        }
        last_index_ = index;
        nearest_index_ = index;

        // 跟踪偏差检测：车到最近点距离超限 -> 发零速、中止轨迹。
        if (checkTrackDeviation(pose, pos, index))
        {
            return Velocity{};
        }

        // 2) 前视点
        const double dt = 1.0 / control_rate_;
        std::size_t t_index = offsetPointIndex(vel, dt, tracking_num_, pos, index);
        track_index_ = t_index;

        // 3) 参考线速度 + 曲率角速度前馈
        double ref_v = 0.0, ref_w = 0.0;
        velProjection(pose, index, pos, ref_v, ref_w);
        if (segment.enable_reverse) { ref_v = -ref_v; }
        // 段限速：NaN=默认上限，不限制。
        if (!std::isnan(segment.max_speed) && std::fabs(ref_v) > segment.max_speed)
        {
            ref_v = std::copysign(segment.max_speed, ref_v);
        }
        // 障碍减速/停止/恢复：两阶段步进模型逐帧递推（不再重规划轨迹）。
        // ref_w 是曲率前馈（w = v·curvature），随 ref_v 等比缩放以保持转弯半径。
        {
            const double route_ref_v = ref_v;
            ref_v = stepTrackedLinearVelocity(route_ref_v);
            ref_w = (std::fabs(route_ref_v) > kEpsilon) ? ref_w * (ref_v / route_ref_v) : 0.0;
        }

        // 4) 位置误差
        const std::size_t last = pos.points.size() - 1;
        const std::size_t safe_idx = std::max<std::size_t>(0, index);
        Pose2D start_pose{pos.points[safe_idx].x, pos.points[safe_idx].y};
        Pose2D aim_pose{pos.points[t_index].x, pos.points[t_index].y};
        if (t_index >= last)
        {
            aim_pose.x = pos.points.back().x + end_extended_distance_ * std::cos(pos.points.back().heading_angle);
            aim_pose.y = pos.points.back().y + end_extended_distance_ * std::sin(pos.points.back().heading_angle);
        }
        TrackingError error = calcDelta(pose, start_pose, aim_pose);

        // 5) 航向误差 = 切向朝向 - 当前航向（差速阶段二航向跟随轨迹切向）
        //    倒车模式：航向对准切线反向（+π），vx 取负
        const double raw_track_heading = pos.points[index].heading_angle;
        const double track_heading = segment.enable_reverse
            ? angles::normalize_angle(raw_track_heading + M_PI)
            : raw_track_heading;
        error.delta_theta = angles::normalize_angle(track_heading - pose.theta);

        // 6) 到点判定：纵向剩余 <= reach_distance_（xy ±1cm）
        // pos_overshoot：越过终点超出容差（定位抖动/惯性导致），反向修正拉回而非判定到点，
        // 避免抖动把车"判定为已到点"后放弃纵向纠偏、任由前馈正速度把车推得更远。
        const bool pos_overshoot = (error.delta_x < -reach_distance_);
        const bool pos_done = !pos_overshoot && (error.delta_x <= reach_distance_);
        if (pos_overshoot)
        {
            ref_v = -ref_v;
        }
        else if (pos_done)
        {
            ref_v = 0.0;
        }

        // 7) wz = 曲率前馈 + 航向比例反馈 + 横向误差折叠（Stanley 式）
        const double level = pid_.selectLevel(std::hypot(vel.vx, vel.vy));
        const double fold = followPath(error, ref_v,
                                       y_gains_.kp[level], y_gains_.kp[level]);
        double wz = ref_w + 2 * kp_heading_ * error.delta_theta + fold;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        // 8) 进入阶段三：到点后原地旋转到目标朝向
        if (pos_done)
        {
            if (segment.has_target_yaw)
            {
                track_phase_ = TrackPhase::kAlignToGoal;
            heading_index_ = 0;
            last_heading_index_ = 0;
            pid_.reset();
            odom_anchor_active_ = false; // 阶段切换：重新按新阶段判据评估末端定位切换
            RCLCPP_INFO(get_logger(),
                        "diff phase2 done (reached, err_x=%.4f m) -> phase3 align->goal.",
                        error.delta_x);
            logArrivalPose("phase2->pos",
                           pose,
                           pos.points.back().x, pos.points.back().y,
                           route.goal_theta);
                ref_v = 0.0;
                wz = 0.0;
            }
            else
            {
                // 到点即停，不做末段旋转（无 target_yaw）
                route_finished_ = true;
                {
                    std::lock_guard<std::mutex> lock(route_mutex_);
                    route_.reset();
                }
                pid_.reset();
                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED,
                                      "diff phase2 done, no target_yaw, skip phase3");
                RCLCPP_INFO(get_logger(),
                            "diff phase2 done (reached, no target_yaw) -> stop, skip phase3.");
                ref_v = 0.0;
                wz = 0.0;
            }
        }

        Velocity traj_v;
        traj_v.vx = ref_v;
        traj_v.vy = 0.0;
        traj_v.wz = wz;

        // 9) 轨迹坐标系 -> 车体坐标系（差速 vy 强制 0，旋转仅影响 vx 分解）
        Velocity out = trajVelToAgvVel(traj_v, track_heading, pose);
        out.vy = 0.0;

        // STOP 悬停兜底：参考速度已经是 0 且当前处于障碍 STOP 状态，直接清零最终输出。
        // 不管前面航向比例纠偏/折叠算出了什么残留量，STOP 时车必须真正静止。
        if (vel_track_mode_ == VelocityTrackMode::kStop && std::fabs(ref_v) <= kEpsilon)
        {
            out = Velocity{};
        }

        // RCLCPP_INFO(get_logger(),
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "diff[phase2 follow] idx=%zu/%zu s=%.3f err=(x%.3f,y%.3f,th%.3f) "
            "ref=(v%.2f,w%.2f) fold=%.3f out=(%.3f,0,%.3f)",
            index, last, pos.points[safe_idx].current_s,
            error.delta_x, error.delta_y, error.delta_theta,
            ref_v, ref_w, fold, out.vx, out.wz);
        return out;
    }

    // 阶段三：到点后原地旋转到目标朝向。
    // 同阶段一：反馈误差锚定 headingProjection 的插值目标 target_theta，heading_final 提供梯形前馈；
    // 完成判定单独锚定固定目标 goal_theta。
    Velocity PathFollowerNode::diffPhaseAlignToGoal(
        const Pose2D& pose, const PlannedRoute& route, const SegmentProfile& /*segment*/)
    {
        // 全流程偏差检测：阶段三原地旋转期间车应仍在轨迹终点附近，偏差超限则中止（同阶段二）。
        const std::size_t goal_idx =
            route.position.points.empty() ? 0 : route.position.points.size() - 1;
        if (checkTrackDeviation(pose, route.position, goal_idx))
        {
            return Velocity{};
        }

        const HeadingRoute& hed = route.heading_final;
        const double goal_theta = route.goal_theta;
        double ref_heading_omega = 0.0;
        double target_theta = goal_theta;
        if (!hed.empty())
        {
            const std::size_t h_index = findHeadingIndex(pose.theta, last_heading_index_, hed);
            last_heading_index_ = h_index;
            heading_index_ = h_index;
            headingProjection(pose.theta, h_index, hed, ref_heading_omega, target_theta);
        }

        // 阶段三完成判定：锚定固定目标 goal_theta（容差 reach_angle_ = 0.015 rad）。
        const double goal_err = angles::normalize_angle(goal_theta - pose.theta);
        // 越冲纠偏判断仅在朝向轨迹已推进到末尾附近时才生效：goal_theta-pose.theta
        // 在旋转刚起步、且起止朝向接近对跖（相差近 π）时，会落在 normalize_angle 的
        // ±π 裂缝两侧，符号对起始位姿噪声极敏感，不能当作"已转过头"的证据；只有
        // 车确已转到规划终点附近（heading_index_ 到达 hed 末尾），goal_err 才真正
        // 反映"是否被惯性/抖动带过了目标"，此时符号才可信。
        const bool near_plan_end = !hed.empty() && (heading_index_ + 1 >= hed.size());
        if (near_plan_end && std::fabs(goal_err) > reach_angle_ && ref_heading_omega * goal_err < 0.0)
        {
            ref_heading_omega = -ref_heading_omega;
        }
        // 前馈耗尽兜底：hed 是规划期离线生成的（起点用规划的 path_end_heading / hold_direction，
        // 而不是本阶段真实进入时的车体朝向），前馈转角与实际残差天然对不齐：
        //   - 规划起末朝向相同时 planHeading 走退化分支，两点 heading_omega 全 0，压根没前馈；
        //   - 正常梯形前馈推进到轨迹末端后也会衰减到 0。
        // 而完成判定锚定的是真实 goal_err，两者一旦不一致（阶段二遗留的朝向残差、近终点
        // map->odom 位姿源切换注入的阶跃等），就会 wz 恒 0 却又判不了到点，永久卡在本阶段。
        // 此时按 goal_err 符号补一个最小角速度，把最后这点残差磨掉。
        if (std::fabs(ref_heading_omega) < kEpsilon && std::fabs(goal_err) > reach_angle_)
        {
            ref_heading_omega = std::copysign(planner_params_.min_angular_velocity, goal_err);
        }
        // 障碍减速/停止/恢复：原地旋转阶段同样受控（以前靠 STOP 重规划 hed 实现，现由模型逐帧递推）。
        // 注意：上面的兜底必须注入在本行之前——本函数既是障碍 STOP 通道，末尾又有
        // |w_new| <= |route_ref_w| 的硬上限；放在其后既绕过避障，又会被硬上限重新削回 0。
        ref_heading_omega = stepTrackedAngularVelocity(ref_heading_omega);

        // 反馈误差：投影目标朝向 - 车实际朝向（归一化）。
        const double err_theta = angles::normalize_angle(target_theta - pose.theta);
        // double wz = ref_heading_omega + kp_heading_ * err_theta;
        double wz = ref_heading_omega;
        wz = std::clamp(wz, -planner_params_.max_angular_velocity, planner_params_.max_angular_velocity);

        if (std::fabs(goal_err) <= reach_angle_)
        {
            route_finished_ = true;
            {
                std::lock_guard<std::mutex> lock(route_mutex_);
                route_.reset();
            }
            pid_.reset();
            RCLCPP_INFO(get_logger(),
                        "diff phase3 done (goal=%.4f goal_err=%.4f rad). Trajectory reached.\n",
                        goal_theta, goal_err);
            logArrivalPose("phase3->goal",
                           pose,
                           route.position.points.back().x, route.position.points.back().y,
                           goal_theta);
            wz = 0.0;
            {
                char buf[128];
                std::snprintf(buf, sizeof(buf),
                              "diff phase3 done, goal=%.4f goal_err=%.4f rad",
                              goal_theta, goal_err);
                publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED, buf);
            }

            // 到点时打印剩余的 HeadingPoints（theta / heading_omega），观察末段梯形残量。
            {
                std::string remain;
                remain.reserve(256);
                for (std::size_t i = heading_index_; i < hed.size(); ++i)
                {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "[%zu]th=%.4f w=%.3f ",
                                  i, hed.points[i].theta, hed.points[i].heading_omega);
                    remain += buf;
                }
                RCLCPP_INFO(get_logger(),
                            "diff[phase3] remaining %zu heading points: %s",
                            hed.size() - heading_index_, remain.c_str());
            }
        }

        Velocity out;
        out.vx = 0.0;
        out.vy = 0.0;
        out.wz = wz;

        // RCLCPP_INFO(get_logger(),
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "diff[phase3 align->goal] h_idx=%zu/%zu goal=%.4f target=%.4f cur=%.4f err=%.4f "
            "goal_err=%.4f hw=%.3f wz=%.3f",
            heading_index_, hed.size(), goal_theta, target_theta, pose.theta, err_theta,
            goal_err, ref_heading_omega, wz);
        return out;
    }

} // namespace path_follower

// rclcpp_components 注册
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (path_follower::PathFollowerNode)
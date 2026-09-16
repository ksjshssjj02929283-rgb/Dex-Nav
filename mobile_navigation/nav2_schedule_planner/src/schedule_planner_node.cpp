#include "schedule_planner/schedule_planner_node.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

namespace schedule_planner
{
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    using std::placeholders::_2;

    SchedulePlannerNode::SchedulePlannerNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("schedule_planner_node", options)
    {
        // ---------------- 参数 ----------------
        action_name_ = declare_parameter<std::string>("action_name", "navigate_through_graph_points");
        tracking_path_topic_ = declare_parameter<std::string>("tracking_path_topic", "/tracking_path");
        navigation_status_topic_ = declare_parameter<std::string>("navigation_status_topic", "/navigation_status");
        stop_service_ = declare_parameter<std::string>("stop_service", "/stop");
        resolve_service_ = declare_parameter<std::string>("resolve_service",
                                                          "/map_asset_manager/resolve_nav_points");
        cancel_service_name_ = declare_parameter<std::string>(
            "cancel_service_name", "/schedule_planner/force_cancel");
        padding_service_ = declare_parameter<std::string>(
            "padding_service", "/collision_monitor/set_polygon_padding");
        stop_polygon_name_ = declare_parameter<std::string>("stop_polygon_name", "PolygonStop");
        padding_timeout_ = declare_parameter<double>("padding_timeout", 2.0);
        require_padding_service_ = declare_parameter<bool>("require_padding_service", true);
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        resolution_ = declare_parameter<double>("resolution", 0.005);
        resolve_timeout_ = declare_parameter<double>("resolve_timeout", 5.0);
        segment_timeout_ = declare_parameter<double>("segment_timeout", 300.0);
        status_message_timeout_ = declare_parameter<double>("status_message_timeout", 0.3);
        bag_trigger_topic_ = declare_parameter<std::string>(
            "bag_trigger_topic", "/mobile_bag_recorder_node/trigger_event");
        // default_mode：PathSegment 未显式指定 mode 时的默认跟踪方式。1=DIFF, 3=OMNI_TRANS_ROTATION。
        default_mode_ = static_cast<uint8_t>(declare_parameter<int>("default_mode", 1));

        // move_mode：底盘运动能力（对应 path_follower 的 base_type），决定允许下发的跟踪方式。
        // 与 default_mode 不同——后者只在段没写 mode 时兜底，前者无条件覆盖。默认 3 = 不约束，
        // 保持引入该参数之前的行为；具体车型在 yaml 里显式指定。
        {
            const int mm = declare_parameter<int>("move_mode", 3);
            if (mm == nav_interfaces::msg::TrackingPath::MODE_DIFF ||
                mm == nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION ||
                mm == nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION)
            {
                move_mode_ = static_cast<uint8_t>(mm);
            }
            else
            {
                move_mode_ = nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION;
                RCLCPP_WARN(get_logger(),
                            "Invalid move_mode=%d (expect 1/2/3), falling back to 3 (no constraint).",
                            mm);
            }
            // 取 1/2 时段 mode 会被归一成具体值，resolveMode 不再走兜底分支，default_mode 失效。
            if (move_mode_ != nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION &&
                default_mode_ != move_mode_)
            {
                RCLCPP_WARN(get_logger(),
                            "move_mode=%u overrides every segment mode, so default_mode=%u "
                            "will never take effect.",
                            move_mode_, default_mode_);
            }
        }

        // 相邻段合并平滑：属性一致、长度够的连续 LINE 段并成一条带拐角倒角的轨迹一次下发，
        // 省掉组内每个途经点的「停车 + 原地旋转」。长度门槛由 anchor_spacing 推导，不单独配。
        merge_params_.enable = declare_parameter<bool>("merge_enable", true);
        merge_params_.bezier_degree = declare_parameter<int>("merge_bezier_degree", 4);
        merge_params_.anchor_spacing = declare_parameter<double>("merge_anchor_spacing", 0.25);
        merge_params_.end_margin = declare_parameter<double>("merge_end_margin", 0.1);
        merge_params_.max_turn_angle_deg =
            declare_parameter<double>("merge_max_turn_angle_deg", 120.0);

        // mode：节点启动时固定二选一，不支持运行时热切换。
        const std::string mode = declare_parameter<std::string>("mode", "independence");
        if (mode == "controller")
        {
            controller_mode_ = true;
        }
        else
        {
            controller_mode_ = false;
            if (mode != "independence")
            {
                RCLCPP_WARN(get_logger(),
                            "Unknown mode '%s', falling back to 'independence'.", mode.c_str());
            }
        }
        follow_path_action_name_ = declare_parameter<std::string>(
            "follow_path_action_name", "/follow_path");
        controller_id_ = declare_parameter<std::string>("controller_id", "");
        goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "");
        if (controller_mode_)
        {
            // controller 模式专用离散分辨率：5mm 对 controller_server 每 tick 全路径扫描
            // （SpeedPlanner Phase A/B、距终点反馈）偏密，默认放宽到 5cm。
            resolution_ = declare_parameter<double>("controller_resolution", 0.05);
        }

        // ---------------- ROS 接口 ----------------
        path_pub_ = create_publisher<nav_interfaces::msg::TrackingPath>(tracking_path_topic_, rclcpp::QoS(1));

        status_sub_ = create_subscription<nav_interfaces::msg::NavigationStatus>(
            navigation_status_topic_, rclcpp::QoS(10),
            std::bind(&SchedulePlannerNode::onNavigationStatus, this, _1));

        resolve_client_ = create_client<ResolveNavPoints>(resolve_service_);
        stop_client_ = create_client<std_srvs::srv::Trigger>(stop_service_);
        padding_client_ = create_client<SetPolygonPadding>(padding_service_);

        cancel_service_ = create_service<std_srvs::srv::Trigger>(
            cancel_service_name_,
            std::bind(&SchedulePlannerNode::onCancelService, this, _1, _2));

        if (!bag_trigger_topic_.empty())
        {
            // 在构造期就建好：publisher 建好到 discovery 完成之间有几十到几百 ms，
            // 等真失败了再现建，第一条上报大概率丢在发现窗口里。
            bag_trigger_pub_ = create_publisher<std_msgs::msg::String>(
                bag_trigger_topic_, rclcpp::QoS(10));
        }

        if (controller_mode_)
        {
            follow_path_client_ = rclcpp_action::create_client<FollowPath>(
                this, follow_path_action_name_);
        }

        action_server_ = rclcpp_action::create_server<NavigateThroughGraphPoints>(
            this, action_name_,
            std::bind(&SchedulePlannerNode::handleGoal, this, _1, _2),
            std::bind(&SchedulePlannerNode::handleCancel, this, _1),
            std::bind(&SchedulePlannerNode::handleAccepted, this, _1));

        std::string controller_suffix;
        if (controller_mode_)
        {
            controller_suffix = " follow_path=" + follow_path_action_name_ +
                                 " controller_id='" + controller_id_ +
                                 "' goal_checker_id='" + goal_checker_id_ + "'";
        }
        RCLCPP_INFO(get_logger(),
                    "schedule_planner ready: action=%s mode=%s tracking_path=%s status=%s stop=%s "
                    "cancel=%s resolve=%s frame=%s resolution=%.4fm seg_timeout=%.0fs default_mode=%d "
                    "move_mode=%d bag_trigger=%s%s",
                    action_name_.c_str(), controller_mode_ ? "controller" : "independence",
                    tracking_path_topic_.c_str(), navigation_status_topic_.c_str(),
                    stop_service_.c_str(), cancel_service_name_.c_str(), resolve_service_.c_str(),
                    frame_id_.c_str(), resolution_, segment_timeout_, static_cast<int>(default_mode_),
                    static_cast<int>(move_mode_),
                    bag_trigger_pub_ ? bag_trigger_topic_.c_str() : "(disabled)",
                    controller_suffix.c_str());
    }

    SchedulePlannerNode::~SchedulePlannerNode()
    {
        shutting_down_.store(true);
        state_cv_.notify_all();
        // 不持 current_goal_mutex_ join，避免与 execute() 末尾 finishGoal() 抢锁死锁
        std::thread t;
        {
            std::lock_guard<std::mutex> lk(current_goal_mutex_);
            t = std::move(exec_thread_);
        }
        if (t.joinable())
        {
            t.join();
        }
    }

    // ---------------------------------------------------------------------------
    // action 回调
    // ---------------------------------------------------------------------------
    rclcpp_action::GoalResponse SchedulePlannerNode::handleGoal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const NavigateThroughGraphPoints::Goal> goal)
    {
        if (goal->nav_point_names.empty())
        {
            RCLCPP_WARN(get_logger(), "Reject goal: need >=1 nav_point_names, got 0.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        std::lock_guard<std::mutex> lk(current_goal_mutex_);
        if (current_goal_ && current_goal_->is_active())
        {
            RCLCPP_WARN(get_logger(), "Reject goal: another goal is active.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        std::string names_str;
        for (std::size_t i = 0; i < goal->nav_point_names.size(); ++i)
        {
            if (i) names_str += " -> ";
            names_str += goal->nav_point_names[i];
        }
        RCLCPP_INFO(get_logger(), "Accept goal with %zu nav_point_names: %s",
                    goal->nav_point_names.size(), names_str.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SchedulePlannerNode::handleCancel(
        const std::shared_ptr<GoalHandleT> goal_handle)
    {
        std::lock_guard<std::mutex> lk(current_goal_mutex_);
        if (current_goal_ == goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Cancel requested for active goal.");
            state_cv_.notify_all(); // 唤醒 runSegment 等待，让其检查 is_canceling()
            controller_state_cv_.notify_all(); // 唤醒 runSegmentViaController 等待
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        return rclcpp_action::CancelResponse::REJECT;
    }

    void SchedulePlannerNode::handleAccepted(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        std::lock_guard<std::mutex> lk(current_goal_mutex_);
        // 回收上一条已结束的执行线程（handleGoal 已拒绝并发，此处线程应已结束）
        if (exec_thread_.joinable())
        {
            exec_thread_.join();
        }
        current_goal_ = goal_handle;
        exec_thread_ = std::thread(&SchedulePlannerNode::execute, this, goal_handle);
    }

    void SchedulePlannerNode::finishGoal()
    {
        std::lock_guard<std::mutex> lk(current_goal_mutex_);
        current_goal_.reset();
    }

    // ---------------------------------------------------------------------------
    // 主执行
    // ---------------------------------------------------------------------------
    void SchedulePlannerNode::execute(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        // 线程边界上的异常兜底：executeGoal() 里一路调的 rclcpp/rclcpp_action 接口都可能抛
        // （典型如对已终态 goal 调 async_cancel_goal 抛 UnknownGoalHandleError），而线程函数
        // 抛出异常 = std::terminate = 整个节点带着 SIGABRT 退出。这里接住，把 goal 收成
        // aborted 让上层拿到明确失败，节点继续服务下一个 goal。
        try
        {
            executeGoal(goal_handle);
            return;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Unhandled exception in execute(): %s", e.what());
            abortGoalOnException(goal_handle, e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Unhandled non-standard exception in execute().");
            abortGoalOnException(goal_handle, "unknown exception");
        }
    }

    void SchedulePlannerNode::abortGoalOnException(
        const std::shared_ptr<GoalHandleT>& goal_handle, const std::string& what)
    {
        // 收尾本身也不许抛：goal 可能已经是终态（异常正是在 abort/succeed 之后发生的），
        // 那时再调 abort() 会因非法状态转换抛出，等于在异常处理里再炸一次。
        try
        {
            if (goal_handle && goal_handle->is_active())
            {
                auto res = std::make_shared<NavigateThroughGraphPoints::Result>();
                res->success = false;
                res->message = "internal exception: " + what;
                goal_handle->abort(res);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to abort goal after exception: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Failed to abort goal after exception.");
        }
        finishGoal();
    }

    void SchedulePlannerNode::executeGoal(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        // 每个新 goal 开始时复位，避免上一个 goal 遗留的取消请求误伤这一个。
        force_cancel_requested_.store(false);

        const auto goal = goal_handle->get_goal();

        // 1) 解析 nav_point_names -> PathSegment[]
        std::vector<PathSegment> segments;
        std::string err;
        if (!resolveSegments(goal->nav_point_names, segments, err))
        {
            auto res = std::make_shared<NavigateThroughGraphPoints::Result>();
            res->success = false;
            res->message = "resolve failed: " + err;
            RCLCPP_ERROR(get_logger(), "Goal aborted: %s", res->message.c_str());
            goal_handle->abort(res);
            finishGoal();
            return;
        }
        if (segments.empty())
        {
            auto res = std::make_shared<NavigateThroughGraphPoints::Result>();
            res->success = false;
            res->message = "no segments resolved";
            goal_handle->abort(res);
            finishGoal();
            return;
        }

        // 2) 分组：属性一致、长度够的连续 LINE 段合并成一条带倒角的轨迹一次下发。
        const std::size_t total = segments.size();
        // map_asset 在车离起点太远时会在最前面插一条归位边，此时段数等于点数
        // （正常是点数-1，见 route_graph_manager.cpp 的 "prepended homing edge"）。
        // 归位段必须单独跑完再开始合并。单点请求也满足这个等式，但它只有一段，无所谓。
        const std::size_t first_mergeable =
            (goal->nav_point_names.size() >= 2 && total == goal->nav_point_names.size()) ? 1u : 0u;
        MergeParams mp = merge_params_;
        // controller 模式走的是 FollowPath + controller_server，离散分辨率与跟踪语义都不同，
        // 合并暂不覆盖该模式。
        mp.enable = mp.enable && !controller_mode_;
        const auto groups = planMergeGroups(segments, first_mergeable, default_mode_, mp);
        if (mp.enable)
        {
            RCLCPP_INFO(get_logger(),
                        "Merge planning: %zu segments -> %zu groups (first_mergeable=%zu, "
                        "d=%.2fm, min_end=%.2fm, min_interior=%.2fm, max_turn=%.0fdeg cos>%.3f)",
                        total, groups.size(), first_mergeable, mp.retreat(),
                        mp.minEndLength(), mp.minInteriorLength(), mp.max_turn_angle_deg,
                        std::cos(mp.max_turn_angle_deg * M_PI / 180.0));
        }

        // 3) 逐组执行
        bool ok = true;
        bool canceled = false;
        std::string fail_msg;
        for (std::size_t gi = 0; gi < groups.size(); ++gi)
        {
            const auto& g = groups[gi];
            const std::size_t i = g.begin; // 进度/日志仍按原始段索引报，上层看到的编号不变
            if (shutting_down_.load())
            {
                fail_msg = "shutdown";
                ok = false;
                break;
            }
            publishFeedback(goal_handle,
                            "executing segment " + std::to_string(i) + "/" + std::to_string(total),
                            i, total);
            RCLCPP_INFO(get_logger(),
                        "Group %zu/%zu = segments [%zu,%zu): type=%u cps=%zu target_yaw=%.3f "
                        "backward=%d max_speed=%.2f",
                        gi + 1, groups.size(), g.begin, g.end, segments[i].type,
                        segments[i].control_points.size(), segments[i].target_yaw,
                        segments[i].backward, segments[i].max_speed);
            std::string seg_msg;
            int r = 0;
            if (controller_mode_)
            {
                r = runSegmentViaController(segments[i], i, total, goal_handle, seg_msg);
            }
            else
            {
                auto tp = (g.size() > 1)
                    ? sampler_.discretizeGroup(segments, g.begin, g.end, resolution_,
                                               mp.anchor_spacing, mp.bezier_degree,
                                               frame_id_, default_mode_)
                    : sampler_.discretize(segments[i], resolution_, frame_id_, default_mode_);
                // 只有最后一组的末段保留 target_yaw，其余置 NaN（到点不做末段旋转）。
                if (gi + 1 < groups.size())
                {
                    tp.target_yaw = std::numeric_limits<double>::quiet_NaN();
                }
                r = runSegment(std::move(tp), i, total, g.size(), goal_handle, seg_msg);
            }
            if (r == 2)
            {
                fail_msg = "canceled at segment " + std::to_string(i) + ": " + seg_msg;
                ok = false;
                canceled = true;
                break;
            }
            if (r == 4)
            {
                fail_msg = "shutdown at segment " + std::to_string(i) + ": " + seg_msg;
                ok = false;
                break;
            }
            if (r != 0)
            {
                fail_msg = "segment " + std::to_string(i) +
                           (r == 1 ? " failed: " : " timeout: ") + seg_msg;
                ok = false;
                // 段级失败统一在这里出一条 WARN：runSegment 的部分失败分支（如 path_follower
                // 报 FAILED）自身不打日志，只把原因塞进 out_msg，不补这条的话中途什么都看不到，
                // 只剩最后 goal 级的 "Goal aborted"，排查时定位不到是哪一段、因为什么断的。
                RCLCPP_WARN(get_logger(), "Segment %zu/%zu %s: %s", i, total,
                            r == 1 ? "failed" : "timeout", seg_msg.c_str());
                // 段级失败是四种失败（FollowPath 拒绝/接受超时/controller abort/段超时，
                // independence 模式下是 path_follower 报 FAILED/超时）唯一的汇合点，
                // 在这里上报一次即可覆盖全部，不必在各 return 点分别埋。
                // 放在 break 之前：越早发，录包节点的 post 窗口越贴近失败时刻。
                triggerBagRecord(std::string("schedule_seg") + std::to_string(i) +
                                 (r == 1 ? "_failed" : "_timeout"));
                break;
            }
            RCLCPP_INFO(get_logger(), "Segment %zu/%zu finished: %s", i, total, seg_msg.c_str());
        }

        // 3) 结果
        auto res = std::make_shared<NavigateThroughGraphPoints::Result>();
        if (ok)
        {
            publishFeedback(goal_handle, "all segments done", total, total);
            res->success = true;
            res->message = "success";
            goal_handle->succeed(res);
            RCLCPP_INFO(get_logger(), "Goal succeeded: %zu segments done.", total);
        }
        else if (canceled)
        {
            res->success = false;
            res->message = fail_msg;
            if (goal_handle->is_canceling())
            {
                // 真正的 action 级 cancel 请求：goal_handle 已经过 CANCELING，转 CANCELED 合法。
                goal_handle->canceled(res);
                RCLCPP_INFO(get_logger(), "Goal canceled: %s", fail_msg.c_str());
            }
            else
            {
                // force_cancel_requested_ 触发，没有经过真正的 action cancel 流程，
                // goal_handle 仍在 EXECUTING：直接调 canceled() 是非法状态转换会崩溃
                // （EXECUTING 必须先经 CANCELING 才能到 CANCELED），改用合法的 EXECUTING→ABORTED。
                goal_handle->abort(res);
                RCLCPP_INFO(get_logger(), "Goal force-stopped (aborted): %s", fail_msg.c_str());
            }
        }
        else
        {
            res->success = false;
            res->message = fail_msg;
            goal_handle->abort(res);
            RCLCPP_ERROR(get_logger(), "Goal aborted: %s", fail_msg.c_str());
        }

        // 暂不启用
        // 目前 goal 结束后 PolygonStop 会停留在最后一段的 padding 上。
        // goal 结束（成功/取消/失败都算），把停障膨胀交还给 collision_monitor 的默认值。
        // {
        //     std::string padding_err;
        //     if (!callSetStopPadding(std::numeric_limits<float>::quiet_NaN(), padding_err))
        //     {
        //         RCLCPP_WARN(get_logger(),
        //                     "restore default stop padding failed: %s", padding_err.c_str());
        //     }
        //     else
        //     {
        //         RCLCPP_INFO(get_logger(), "Stop padding restored to default.");
        //     }
        // }
        finishGoal();
    }

    int SchedulePlannerNode::runSegment(
        nav_interfaces::msg::TrackingPath tp, std::size_t idx, std::size_t total,
        std::size_t seg_count,
        const std::shared_ptr<GoalHandleT>& goal_handle, std::string& out_msg)
    {
        if (tp.poses.empty())
        {
            out_msg = "segment discretized to empty path";
            RCLCPP_ERROR(get_logger(), "Segment discretized to empty path, aborting segment.");
            return 1;
        }
        tp.header.stamp = now();

        // 本段的停障膨胀先落到 collision_monitor，再发路径：
        // 必须等服务返回，否则 path_follower 已经起步而 padding 还是上一段的值。
        {
            std::string padding_err;
            if (!callSetStopPadding(tp.padding, padding_err))
            {
                if (require_padding_service_)
                {
                    out_msg = "set stop padding failed: " + padding_err;
                    RCLCPP_ERROR(get_logger(), "%s, aborting segment.", out_msg.c_str());
                    return 1;
                }
                // 放行时明确告警：此时停障距离仍是上一段的值，不是本段期望的
                RCLCPP_WARN(get_logger(),
                            "set stop padding failed: %s, continuing with previous padding.",
                            padding_err.c_str());
            }
            else
            {
                const std::string padding_str = std::isnan(tp.padding)
                                                    ? std::string("default")
                                                    : std::to_string(tp.padding);
                RCLCPP_INFO(get_logger(), "Segment %zu/%zu stop padding: %s",
                            idx + 1, total, padding_str.c_str());
            }
        }

        // 复位跟踪状态，避免上一段残留 FINISHED 立即满足等待
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            track_state_ = TrackState::kIdle;
            last_status_message_.clear();
        }
        path_pub_->publish(tp);

        // 合并后一条轨迹可能由多段拼成，超时按段数等比放宽，否则长路径会被误判超时。
        const double group_timeout = segment_timeout_ * static_cast<double>(std::max<std::size_t>(1, seg_count));
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(group_timeout));
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling() || force_cancel_requested_.load())
            {
                callStop();
                out_msg = goal_handle->is_canceling() ? "canceled by action client" : "force-cancel requested";
                return 2;
            }
            if (shutting_down_.load())
            {
                callStop();
                out_msg = "node shutting down";
                return 4;
            }
            std::unique_lock<std::mutex> lk(state_mutex_);
            state_cv_.wait_for(lk, 200ms,
                [this] {
                    return track_state_ == TrackState::kFinished ||
                           track_state_ == TrackState::kFailed;
                });
            if (track_state_ == TrackState::kFinished)
            {
                out_msg = last_status_message_;
                return 0;
            }
            if (track_state_ == TrackState::kFailed)
            {
                out_msg = last_status_message_.empty() ?
                    "path_follower reported FAILED (no detail)" : last_status_message_;
                return 1;
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                RCLCPP_ERROR(get_logger(), "Segment timeout after %.1fs.", segment_timeout_);
                callStop();
                out_msg = "segment timeout after " + std::to_string(segment_timeout_) + "s";
                return 3;
            }
        }
        // rclcpp 退出
        callStop();
        out_msg = "rclcpp shutdown";
        return 4;
    }

    std::string SchedulePlannerNode::waitFailureMessage()
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(status_message_timeout_));
        std::unique_lock<std::mutex> lk(state_mutex_);
        state_cv_.wait_until(lk, deadline,
            [this] { return track_state_ == TrackState::kFailed; });
        if (track_state_ != TrackState::kFailed)
        {
            return {};
        }
        return last_status_message_;
    }

    nav_msgs::msg::Path SchedulePlannerNode::buildNav2Path(
        const nav_interfaces::msg::TrackingPath& tp) const
    {
        nav_msgs::msg::Path path;
        path.header = tp.header;
        const std::size_t n = tp.poses.size();
        path.poses.reserve(n);

        double last_yaw = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = tp.header;
            ps.pose.position = tp.poses[i].position;

            double yaw = last_yaw;
            if (i + 1 < n)
            {
                const double dx = tp.poses[i + 1].position.x - tp.poses[i].position.x;
                const double dy = tp.poses[i + 1].position.y - tp.poses[i].position.y;
                if (std::fabs(dx) > 1e-9 || std::fabs(dy) > 1e-9)
                {
                    yaw = std::atan2(dy, dx);
                }
            }
            else if (!std::isnan(tp.target_yaw))
            {
                // 末点：优先用段级 target_yaw 表达期望的最终朝向（喂给 goal_checker 的偏航判定）。
                yaw = tp.target_yaw;
            }
            // 纯 yaw 旋转的四元数，避免为此引入 tf2 依赖。
            ps.pose.orientation.z = std::sin(yaw * 0.5);
            ps.pose.orientation.w = std::cos(yaw * 0.5);

            path.poses.push_back(ps);
            last_yaw = yaw;
        }
        return path;
    }

    void SchedulePlannerNode::cancelControllerGoal()
    {
        GoalHandleFollowPath::SharedPtr gh;
        {
            std::lock_guard<std::mutex> lk(controller_state_mutex_);
            // 只有还在执行中的 goal 才可能被 rclcpp_action 客户端认得：结果一旦回来
            // （result_callback 里置终态的同时，客户端内部已 goal_handles_.erase 掉它），
            // 再拿这个句柄去 async_cancel_goal 就会抛 UnknownGoalHandleError。
            if (controller_goal_state_ != ControllerGoalState::kExecuting)
            {
                return;
            }
            gh = controller_goal_handle_;
        }
        if (!gh || !follow_path_client_)
        {
            return;
        }
        // 上面的状态判断挡不住竞态：判断与发送之间结果可能刚好到达并被 erase。
        // 这里是兜底——本函数在节点 executor 线程（onCancelService）和 exec_thread_ 上
        // 都会被调用，任一处异常逃逸都会 std::terminate 掉整个节点。
        try
        {
            follow_path_client_->async_cancel_goal(gh);
        }
        catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
        {
            RCLCPP_DEBUG(get_logger(),
                         "FollowPath goal already reached a terminal state; cancel skipped.");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Cancel FollowPath goal failed: %s", e.what());
        }
    }

    void SchedulePlannerNode::triggerBagRecord(const std::string& reason)
    {
        if (!bag_trigger_pub_)
        {
            return;
        }
        std_msgs::msg::String msg;
        msg.data = reason;
        bag_trigger_pub_->publish(msg);
        // 上报了不等于录到了：录包节点可能没起、event 关着、或正处于冷却/上一次事件录制中。
        // 那边命中与否都有日志，这边只记发出这一步，不然现场分不清是没发还是被拒了。
        RCLCPP_INFO(get_logger(), "Requested event bag recording: reason=%s topic=%s",
                    reason.c_str(), bag_trigger_topic_.c_str());
    }

    void SchedulePlannerNode::onCancelService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        bool had_active_goal = false;
        {
            std::lock_guard<std::mutex> lk(current_goal_mutex_);
            had_active_goal = current_goal_ && current_goal_->is_active();
        }

        // 有 active goal 时置位让等待循环走正常的"取消"返回路径（return 2，
        // execute() 里统一处理成 goal_handle->canceled()）。
        force_cancel_requested_.store(true);
        state_cv_.notify_all();
        controller_state_cv_.notify_all();

        // 无论有没有 active goal，都直接兜底停一次车：没有 goal 时上面的置位没人会看，
        // 只能在这里自己发停车指令。按模式选择对应的停车通道，不两个都调
        // （independence 模式下 follow_path_client_ 为空，cancelControllerGoal 本身是空操作，
        // 但没必要两条通道都走一遍）。
        // 本回调运行在节点 executor 线程上，callStopAsync 必须用非阻塞版，否则会自锁死。
        if (controller_mode_)
        {
            cancelControllerGoal();
        }
        else
        {
            callStopAsync();
        }

        res->success = true;
        res->message = had_active_goal ?
            "Canceled active goal and issued stop." :
            "No active goal; issued stop anyway.";
    }

    int SchedulePlannerNode::runSegmentViaController(
        const PathSegment& seg, std::size_t idx, std::size_t total,
        const std::shared_ptr<GoalHandleT>& goal_handle, std::string& out_msg)
    {
        auto tp = sampler_.discretize(seg, resolution_, frame_id_, default_mode_);
        if (tp.poses.empty())
        {
            out_msg = "segment discretized to empty path";
            RCLCPP_ERROR(get_logger(), "Segment discretized to empty path, aborting segment.");
            return 1;
        }
        if (idx < total - 1)
        {
            tp.target_yaw = std::numeric_limits<double>::quiet_NaN(); // 非末段不指定 target_yaw
        }
        tp.header.stamp = now();

        const nav_msgs::msg::Path path = buildNav2Path(tp);

        const auto server_wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(resolve_timeout_));
        if (!follow_path_client_->wait_for_action_server(server_wait))
        {
            out_msg = "FollowPath action server '" + follow_path_action_name_ + "' unavailable";
            RCLCPP_ERROR(get_logger(), "FollowPath action server '%s' unavailable.",
                         follow_path_action_name_.c_str());
            return 1;
        }

        FollowPath::Goal fp_goal;
        fp_goal.path = path;
        fp_goal.controller_id = controller_id_;
        fp_goal.goal_checker_id = goal_checker_id_;

        {
            std::lock_guard<std::mutex> lk(controller_state_mutex_);
            controller_goal_state_ = ControllerGoalState::kAwaitingAcceptance;
            controller_goal_handle_.reset();
        }

        // 出这段时（无论走哪条 return）都把句柄清掉。留到下一段开头才 reset 的话，
        // 段与段之间、以及最后一段结束后，controller_goal_handle_ 都还挂着一个已终态的
        // 陈旧句柄，此期间任何一次取消（force_cancel 服务最典型：上层收到 goal aborted
        // 后习惯再兜底停一次车）都会拿它去 async_cancel_goal 而抛异常。
        struct HandleReleaser
        {
            SchedulePlannerNode* self;
            ~HandleReleaser()
            {
                std::lock_guard<std::mutex> lk(self->controller_state_mutex_);
                self->controller_goal_handle_.reset();
            }
        } handle_releaser{this};

        // 复位跟踪状态：本段失败时要靠 navigation_status 的 message 给失败原因，
        // 不复位的话会把上一段残留的 FAILED 文本当成本段的原因报上去。
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            track_state_ = TrackState::kIdle;
            last_status_message_.clear();
        }

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](GoalHandleFollowPath::SharedPtr gh)
            {
                std::lock_guard<std::mutex> lk(controller_state_mutex_);
                if (!gh)
                {
                    controller_goal_state_ = ControllerGoalState::kRejected;
                }
                else
                {
                    controller_goal_handle_ = gh;
                    controller_goal_state_ = ControllerGoalState::kExecuting;
                }
                controller_state_cv_.notify_all();
            };
        send_goal_options.result_callback =
            [this](const GoalHandleFollowPath::WrappedResult& result)
            {
                std::lock_guard<std::mutex> lk(controller_state_mutex_);
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    controller_goal_state_ = ControllerGoalState::kSucceeded;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    controller_goal_state_ = ControllerGoalState::kCanceled;
                    break;
                default:
                    controller_goal_state_ = ControllerGoalState::kAborted;
                    break;
                }
                controller_state_cv_.notify_all();
            };

        follow_path_client_->async_send_goal(fp_goal, send_goal_options);

        // 1) 等 goal 被接受/拒绝（复用 resolve_timeout_ 作为等待上限）。
        const auto accept_deadline = std::chrono::steady_clock::now() +
                                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<double>(resolve_timeout_));
        {
            std::unique_lock<std::mutex> lk(controller_state_mutex_);
            controller_state_cv_.wait_until(lk, accept_deadline,
                [this]
                {
                    return controller_goal_state_ != ControllerGoalState::kAwaitingAcceptance;
                });
            if (controller_goal_state_ == ControllerGoalState::kAwaitingAcceptance)
            {
                out_msg = "FollowPath goal acceptance timeout";
                RCLCPP_ERROR(get_logger(), "FollowPath goal acceptance timeout.");
                return 1;
            }
            if (controller_goal_state_ == ControllerGoalState::kRejected)
            {
                lk.unlock();
                const std::string detail = waitFailureMessage();
                out_msg = "FollowPath goal rejected by controller_server";
                if (!detail.empty())
                {
                    out_msg += ": " + detail;
                }
                RCLCPP_ERROR(get_logger(), "FollowPath goal rejected by controller_server: %s",
                             detail.empty() ? "(no detail on navigation_status)" : detail.c_str());
                return 1;
            }
        }

        // 2) 等终态，期间响应取消/关闭/超时（返回码语义与 runSegment 一致）。
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(segment_timeout_));
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling() || force_cancel_requested_.load())
            {
                cancelControllerGoal();
                out_msg = goal_handle->is_canceling() ? "canceled by action client" : "force-cancel requested";
                return 2;
            }
            if (shutting_down_.load())
            {
                cancelControllerGoal();
                out_msg = "node shutting down";
                return 4;
            }
            std::unique_lock<std::mutex> lk(controller_state_mutex_);
            controller_state_cv_.wait_for(lk, 200ms,
                [this]
                {
                    return controller_goal_state_ == ControllerGoalState::kSucceeded ||
                           controller_goal_state_ == ControllerGoalState::kAborted ||
                           controller_goal_state_ == ControllerGoalState::kCanceled;
                });
            if (controller_goal_state_ == ControllerGoalState::kSucceeded)
            {
                return 0;
            }
            if (controller_goal_state_ == ControllerGoalState::kAborted)
            {
                // FollowPath 结果不带失败原因，去 navigation_status 上取 controller 报的详情。
                lk.unlock();
                const std::string detail = waitFailureMessage();
                out_msg = "controller_server aborted FollowPath goal";
                if (!detail.empty())
                {
                    out_msg += ": " + detail;
                }
                RCLCPP_ERROR(get_logger(), "Segment %zu/%zu aborted by controller_server: %s",
                             idx + 1, total,
                             detail.empty() ? "(no detail on navigation_status)" : detail.c_str());
                return 1;
            }
            if (controller_goal_state_ == ControllerGoalState::kCanceled)
            {
                out_msg = "FollowPath goal canceled";
                return 2;
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                RCLCPP_ERROR(get_logger(), "Segment timeout after %.1fs (controller mode).",
                             segment_timeout_);
                lk.unlock();
                cancelControllerGoal();
                out_msg = "segment timeout after " + std::to_string(segment_timeout_) + "s (controller mode)";
                // 超时本身不是 controller 报的错，但若它同时已报了 FAILED，一并带上更好定位。
                const std::string detail = waitFailureMessage();
                if (!detail.empty())
                {
                    out_msg += ": " + detail;
                }
                return 3;
            }
        }
        // rclcpp 退出
        cancelControllerGoal();
        out_msg = "rclcpp shutdown";
        return 4;
    }

    // ---------------------------------------------------------------------------
    // 外部交互
    // ---------------------------------------------------------------------------
    bool SchedulePlannerNode::resolveSegments(
        const std::vector<std::string>& nav_point_names,
        std::vector<PathSegment>& segments, std::string& err)
    {
        if (!resolve_client_->wait_for_service(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(resolve_timeout_))))
        {
            err = "resolve service '" + resolve_service_ + "' unavailable";
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            return false;
        }
        auto req = std::make_shared<ResolveNavPoints::Request>();
        req->nav_point_names = nav_point_names;
        auto fut = resolve_client_->async_send_request(req);
        const auto wait_deadline = std::chrono::steady_clock::now() +
                                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(resolve_timeout_));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < wait_deadline)
        {
            if (fut.wait_for(100ms) == std::future_status::ready)
            {
                break;
            }
        }
        if (fut.wait_for(0s) != std::future_status::ready)
        {
            err = "resolve service timeout";
            return false;
        }
        auto resp = fut.get();
        if (!resp->success)
        {
            err = resp->message.empty() ? "resolve returned failure" : resp->message;
            return false;
        }
        segments = std::move(resp->segments);
        // 这里是 segments 的唯一入口，后续 planMergeGroups / discretize / discretizeGroup /
        // runSegmentViaController 全部取自它，所以底盘能力约束在这里归一一次就够。
        applyMoveModeConstraint(segments);
        return true;
    }

    void SchedulePlannerNode::applyMoveModeConstraint(std::vector<PathSegment>& segments)
    {
        using TP = nav_interfaces::msg::TrackingPath;
        using PS = nav_interfaces::msg::PathSegment;

        if (move_mode_ == TP::MODE_OMNI_TRANS_ROTATION)
        {
            return; // 3 = 全能力，三种模式都允许，按段自身的 mode 走（等同于引入该参数之前）
        }

        const bool to_diff = (move_mode_ == TP::MODE_DIFF);
        for (auto& s : segments)
        {
            const uint8_t before = s.mode;
            if (to_diff)
            {
                s.mode = PS::MODE_DIFF;
                // 清成 NaN 而不是留着不管：sampler 会把它原样透传进 TrackingPath.hold_direction，
                // 而该字段在 MODE_DIFF 下无意义，留着只会让下游排查时误判成"还想保向平移"。
                // backward 保留——差速车支持倒车，那是另一个维度。
                s.hold_direction = std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
                s.mode = PS::MODE_OMNI_TRANSLATION;
                if (std::isnan(s.hold_direction))
                {
                    // 全向平移必须有保持朝向，缺了只能由 resolveMode 降级为 DIFF。这不违背
                    // move_mode=2 的前提：全向底盘本就能做差速运动（差速是全向的子集）；而
                    // move_mode=1 的差速车不能平移，那才是物理上的硬约束，所以上面那支是强制的。
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "move_mode=2 but segment hold_direction is NaN: this segment will be "
                        "downgraded to MODE_DIFF by resolveMode.");
                }
            }
            // before==0 是"段未显式指定 mode"，被归一属于预期，不值得告警。
            if (before != 0 && before != s.mode)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "move_mode=%u overrides segment mode %u -> %u.",
                                     move_mode_, before, s.mode);
            }
        }
    }

    void SchedulePlannerNode::callStop()
    {
        if (!stop_client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(), "stop service '%s' not ready, cannot stop path_follower.",
                        stop_service_.c_str());
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto fut = stop_client_->async_send_request(req);
        if (fut.wait_for(500ms) != std::future_status::ready)
        {
            RCLCPP_WARN(get_logger(), "stop service call timeout.");
            return;
        }
        auto resp = fut.get();
        if (!resp->success)
        {
            RCLCPP_WARN(get_logger(), "stop service returned failure: %s", resp->message.c_str());
        }
    }

    bool SchedulePlannerNode::callSetStopPadding(float padding, std::string& err)
    {
        if (!padding_client_->wait_for_service(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(padding_timeout_))))
        {
            err = "padding service '" + padding_service_ + "' unavailable";
            return false;
        }
        auto req = std::make_shared<SetPolygonPadding::Request>();
        req->polygon_name = stop_polygon_name_;
        req->padding = padding;  // NaN 原样透传：collision_monitor 侧即恢复 default_padding
        auto fut = padding_client_->async_send_request(req);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(padding_timeout_));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            if (fut.wait_for(50ms) == std::future_status::ready)
            {
                break;
            }
        }
        if (fut.wait_for(0s) != std::future_status::ready)
        {
            err = "padding service timeout";
            return false;
        }
        auto resp = fut.get();
        if (!resp->success)
        {
            err = resp->message.empty() ? "padding service returned failure" : resp->message;
            return false;
        }
        return true;
    }

    void SchedulePlannerNode::callStopAsync()
    {
        if (!stop_client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(), "stop service '%s' not ready, cannot stop path_follower.",
                        stop_service_.c_str());
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        stop_client_->async_send_request(
            req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                auto resp = future.get();
                if (!resp->success)
                {
                    RCLCPP_WARN(get_logger(), "stop service returned failure: %s",
                                resp->message.c_str());
                }
            });
    }

    void SchedulePlannerNode::onNavigationStatus(
        const nav_interfaces::msg::NavigationStatus::SharedPtr msg)
    {
        TrackState s = TrackState::kIdle;
        switch (msg->status)
        {
        case nav_interfaces::msg::NavigationStatus::NAVIGATING:
            s = TrackState::kNavigating;
            break;
        case nav_interfaces::msg::NavigationStatus::FINISHED:
            s = TrackState::kFinished;
            break;
        case nav_interfaces::msg::NavigationStatus::FAILED:
            s = TrackState::kFailed;
            break;
        default:
            s = TrackState::kIdle;
            break;
        }
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            track_state_ = s;
            last_status_message_ = msg->message;
        }
        state_cv_.notify_all();
    }

    void SchedulePlannerNode::publishFeedback(
        const std::shared_ptr<GoalHandleT>& gh,
        const std::string& status, std::size_t idx, std::size_t total)
    {
        auto fb = std::make_shared<NavigateThroughGraphPoints::Feedback>();
        fb->current_status = status;
        fb->current_point_index = static_cast<uint32_t>(idx);
        fb->total_points = static_cast<uint32_t>(total);
        gh->publish_feedback(fb);
    }
} // namespace schedule_planner

// rclcpp_components 注册
RCLCPP_COMPONENTS_REGISTER_NODE(schedule_planner::SchedulePlannerNode)

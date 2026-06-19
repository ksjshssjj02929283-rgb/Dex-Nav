#include "map_asset_manager/point_manager/nav_point_manager.hpp"

#include <chrono>
#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace map_asset_manager {

NavPointManager::NavPointManager(rclcpp::Node* node,
                                       std::shared_ptr<PointStore> point_store,
                                       std::shared_ptr<DataEventBus> event_bus,
                                       std::function<void()> save_fn)
    : node_(node),
      point_store_(point_store),
      event_bus_(event_bus),
      save_fn_(std::move(save_fn)) {

    // ── TF ────────────────────────────────────────────────────────
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_);

    // ── publishers ────────────────────────────────────────────────
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/nav_points_marker", rclcpp::QoS(10).transient_local());

    // ── services ─────────────────────────────────────────────────
    add_point_srv_ = node_->create_service<nav_interfaces::srv::AddNavPoint>(
        "~/add_point",
        std::bind(&NavPointManager::addPointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    sample_point_srv_ = node_->create_service<nav_interfaces::srv::SampleNavPoint>(
        "~/sample_point",
        std::bind(&NavPointManager::samplePointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    remove_point_srv_ = node_->create_service<nav_interfaces::srv::RemoveNavPoint>(
        "~/remove_point",
        std::bind(&NavPointManager::removePointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    update_point_srv_ = node_->create_service<nav_interfaces::srv::UpdateNavPoint>(
        "~/update_point",
        std::bind(&NavPointManager::updatePointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_point_srv_ = node_->create_service<nav_interfaces::srv::GetNavPoint>(
        "~/get_point",
        std::bind(&NavPointManager::getPointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_all_points_srv_ = node_->create_service<nav_interfaces::srv::GetAllNavPoints>(
        "~/get_all_nav_points",
        std::bind(&NavPointManager::getAllPointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_status_srv_ = node_->create_service<nav_interfaces::srv::GetStatus>(
        "~/get_status",
        std::bind(&NavPointManager::getStatusCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "NavPointManager: services registered");
}

void NavPointManager::initializeActionServers(rclcpp::Node::SharedPtr node,
                                                 std::shared_ptr<NavigationExecutor> nav_executor) {
    nav_executor_ = nav_executor;

    navigate_to_point_action_ = rclcpp_action::create_server<NavigateToPointAction>(
        node, "~/navigate_to_point",
        std::bind(&NavPointManager::handleGoal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavPointManager::handleCancel, this, std::placeholders::_1),
        std::bind(&NavPointManager::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "NavPointManager: action server registered");
}

// ──────────────────────────────────────────────────────────────────────
// Service 回调
// ──────────────────────────────────────────────────────────────────────

void NavPointManager::addPointCallback(
    const std::shared_ptr<nav_interfaces::srv::AddNavPoint::Request> req,
    std::shared_ptr<nav_interfaces::srv::AddNavPoint::Response> res) {
    NavPoint pt;
    pt.name      = req->point.name;
    pt.type      = req->point.type;
    pt.x         = req->point.x;
    pt.y         = req->point.y;
    pt.yaw       = req->point.theta;
    pt.tolerance = req->point.tolerance;

    int result = point_store_->addPoint(pt);
    if (result > 0) {
        res->success = true;
        res->message = (result == 1 ? "Point added: " : "Point overwritten: ") + pt.name;
        saveToFile();
        publishMarkers();
        event_bus_->publish({AssetEventType::POINT_ADDED, pt.name, ""});
    } else {
        res->success = false;
        res->message = "Failed to add point: " + pt.name;
    }
}

std::string NavPointManager::pointTypeToString(uint32_t type) {
    using Req = nav_interfaces::srv::SampleNavPoint::Request;
    switch (type) {
        case Req::STATION:  return "STATION";
        case Req::CHARGING: return "CHARGING";
        case Req::PARKING:  return "PARKING";
        case Req::DOORWAIT: return "DOORWAIT";
        case Req::LIFT:     return "LIFT";
        default:            return "";
    }
}

void NavPointManager::samplePointCallback(
    const std::shared_ptr<nav_interfaces::srv::SampleNavPoint::Request> req,
    std::shared_ptr<nav_interfaces::srv::SampleNavPoint::Response> res) {
    // name 为空 = 仅采样返回坐标，不保存（UI 预填模式）
    const bool save_mode = !req->name.empty();

    if (save_mode) {
        using Req = nav_interfaces::srv::SampleNavPoint::Request;
        if (req->point_type == Req::FREE_GOAL) {
            RCLCPP_ERROR(node_->get_logger(), "sample_point: FREE_GOAL is not supported");
            res->success = false;
            return;
        }
        std::string type_str = pointTypeToString(req->point_type);
        if (type_str.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "sample_point: invalid point_type %u", req->point_type);
            res->success = false;
            return;
        }
    }

    // 采样 11 次 base_link → map 变换，取平均
    constexpr int kSamples = 11;
    double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0;

    for (int i = 0; i < kSamples; ++i) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "sample_point: TF lookup failed: %s", ex.what());
            res->success = false;
            return;
        }
        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);

        sum_x     += tf.transform.translation.x;
        sum_y     += tf.transform.translation.y;
        sum_theta += tf2::getYaw(q) + M_PI * 2.0;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const double x   = sum_x     / kSamples;
    const double y   = sum_y     / kSamples;
    const double yaw = sum_theta / kSamples - M_PI * 2.0;

    res->x     = static_cast<float>(x);
    res->y     = static_cast<float>(y);
    res->theta = static_cast<float>(yaw);

    if (!save_mode) {
        // 仅采样，不保存
        res->success = true;
        RCLCPP_INFO(node_->get_logger(), "sample_point (preview): (%.3f, %.3f, %.3f°)", x, y, yaw * 180.0 / M_PI);
        return;
    }

    std::string type_str = pointTypeToString(req->point_type);
    NavPoint pt;
    pt.name  = req->name;
    pt.type  = type_str;
    pt.x     = x;
    pt.y     = y;
    pt.yaw   = yaw;

    int result = point_store_->addPoint(pt);
    if (result > 0) {
        res->success = true;
        saveToFile();
        publishMarkers();
        event_bus_->publish({AssetEventType::POINT_ADDED, pt.name, ""});
        RCLCPP_INFO(node_->get_logger(), "sample_point: added '%s' (%.3f, %.3f, %.3f°)",
                    pt.name.c_str(), pt.x, pt.y, pt.yaw * 180.0 / M_PI);
    } else {
        res->success = false;
        RCLCPP_ERROR(node_->get_logger(), "sample_point: failed to add '%s'", pt.name.c_str());
    }
}

void NavPointManager::removePointCallback(
    const std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Request> req,
    std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Response> res) {
    if (point_store_->removePoint(req->name)) {
        res->success = true;
        res->message = "Point removed: " + req->name;
        saveToFile();
        publishMarkers();
        // 通知 RouteManager 联动更新
        event_bus_->publish({AssetEventType::POINT_REMOVED, req->name, ""});
    } else {
        res->success = false;
        res->message = "Point not found: " + req->name;
    }
}

void NavPointManager::updatePointCallback(
    const std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Request> req,
    std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Response> res) {
    NavPoint pt;
    pt.name      = req->point.name;
    pt.type      = req->point.type;
    pt.x         = req->point.x;
    pt.y         = req->point.y;
    pt.yaw       = req->point.theta;
    pt.tolerance = req->point.tolerance;

    // 判断是否为重命名（通过 request 中 old_name 字段，若接口没有则留空）
    // 目前 nav_interfaces::srv::UpdateNavPoint 按现有接口处理（无 old_name 字段）
    if (point_store_->updatePoint(pt)) {
        res->success = true;
        res->message = "Point updated: " + pt.name;
        saveToFile();
        publishMarkers();
        event_bus_->publish({AssetEventType::POINT_UPDATED, pt.name, ""});
    } else {
        res->success = false;
        res->message = "Point not found: " + pt.name;
    }
}

void NavPointManager::getPointCallback(
    const std::shared_ptr<nav_interfaces::srv::GetNavPoint::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetNavPoint::Response> res) {
    auto opt = point_store_->getPoint(req->name);
    if (opt) {
        res->success       = true;
        res->point.name    = opt->name;
        res->point.type    = opt->type;
        res->point.x       = opt->x;
        res->point.y       = opt->y;
        res->point.theta   = opt->yaw;
        res->point.tolerance = opt->tolerance;
    } else {
        res->success = false;
        RCLCPP_WARN(node_->get_logger(), "Point not found: %s", req->name.c_str());
    }
}

void NavPointManager::getAllPointsCallback(
    const std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Request>,
    std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Response> res) {
    auto points = point_store_->getAllPoints();
    res->success = true;
    for (const auto& pt : points) {
        nav_interfaces::msg::NavPoint msg;
        msg.name      = pt.name;
        msg.type      = pt.type;
        msg.x         = pt.x;
        msg.y         = pt.y;
        msg.theta     = pt.yaw;
        msg.tolerance = pt.tolerance;
        res->points.push_back(msg);
    }
}

void NavPointManager::getStatusCallback(
    const std::shared_ptr<nav_interfaces::srv::GetStatus::Request>,
    std::shared_ptr<nav_interfaces::srv::GetStatus::Response> res) {
    if (!nav_executor_) {
        res->success = false;
        return;
    }
    auto state  = nav_executor_->getCurrentState();
    auto target = nav_executor_->getCurrentTarget();
    res->success = true;
    res->name    = target;
    switch (state) {
        case NavState::IDLE:       res->status = "IDLE";       break;
        case NavState::NAVIGATING: res->status = "NAVIGATING"; break;
        case NavState::SUCCESS:    res->status = "SUCCESS";    break;
        case NavState::FAILED:     res->status = "FAILED";     break;
        case NavState::CANCELED:   res->status = "CANCELED";   break;
    }
}

// ──────────────────────────────────────────────────────────────────────
// Action 回调
// ──────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse NavPointManager::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const NavigateToPointAction::Goal> goal) {
    if (!goal->point_name.empty() && !point_store_->hasPoint(goal->point_name)) {
        RCLCPP_WARN(node_->get_logger(), "navigate_to_point: point not found: %s",
                    goal->point_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavPointManager::handleCancel(
    const std::shared_ptr<GoalHandleNavigateToPoint>) {
    nav_executor_->cancelNavigation();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPointManager::handleAccepted(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    std::thread{std::bind(&NavPointManager::executeNavigateToPoint, this, goal_handle)}.detach();
}

void NavPointManager::executeNavigateToPoint(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    const auto goal    = goal_handle->get_goal();
    auto feedback      = std::make_shared<NavigateToPointAction::Feedback>();
    auto result        = std::make_shared<NavigateToPointAction::Result>();

    bool nav_started = false;
    if (!goal->point_name.empty()) {
        auto opt = point_store_->getPoint(goal->point_name);
        if (!opt) {
            result->success = false;
            result->message = "Point not found: " + goal->point_name;
            goal_handle->abort(result);
            return;
        }
        auto pose = nav_executor_->createPoseStamped(opt->x, opt->y, opt->yaw);
        double tol = opt->tolerance > 0.0 ? opt->tolerance : 0.05;
        nav_started = nav_executor_->navigateToPose(goal->point_name, pose, tol);
    } else if (goal->use_pose) {
        nav_started = nav_executor_->navigateToPose("custom_pose", goal->pose, goal->tolerance);
    }

    if (!nav_started) {
        result->success = false;
        result->message = "Failed to start navigation";
        goal_handle->abort(result);
        return;
    }

    rclcpp::Rate rate(2);
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            nav_executor_->cancelNavigation();
            result->success = false;
            result->message = "Navigation canceled";
            goal_handle->canceled(result);
            return;
        }

        auto state = nav_executor_->getCurrentState();
        feedback->current_status = [state]() -> std::string {
            switch (state) {
                case NavState::NAVIGATING: return "NAVIGATING";
                case NavState::SUCCESS:    return "SUCCESS";
                case NavState::FAILED:     return "FAILED";
                case NavState::CANCELED:   return "CANCELED";
                default:                   return "IDLE";
            }
        }();
        goal_handle->publish_feedback(feedback);

        if (state == NavState::SUCCESS) {
            result->success = true;
            result->message = "Navigation succeeded";
            goal_handle->succeed(result);
            return;
        } else if (state == NavState::FAILED) {
            result->success = false;
            result->message = "Navigation failed";
            goal_handle->abort(result);
            return;
        } else if (state == NavState::CANCELED) {
            result->success = false;
            result->message = "Navigation canceled";
            goal_handle->canceled(result);
            return;
        }
        rate.sleep();
    }
}

// ──────────────────────────────────────────────────────────────────────
// 辅助
// ──────────────────────────────────────────────────────────────────────

void NavPointManager::saveToFile() {
    if (save_fn_) save_fn_();
}

void NavPointManager::publishMarkers() {
    auto points = point_store_->getAllPoints();
    visualization_msgs::msg::MarkerArray arr;

    // 先删除超出当前数量的旧 marker（防止删除点后残留）
    const int new_count = static_cast<int>(points.size()) * 3;  // 每点 3 个 marker
    for (int i = new_count; i < prev_marker_count_; ++i) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp    = node_->now();
        del.ns              = "nav_points";
        del.id              = i;
        del.action          = visualization_msgs::msg::Marker::DELETE;
        arr.markers.push_back(del);
    }
    prev_marker_count_ = new_count;

    int id = 0;
    for (const auto& pt : points) {
        // ① 圆柱体（点位本体）
        auto m = createPointMarker(pt, id++);
        arr.markers.push_back(m);

        // ② 朝向箭头
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp    = node_->now();
        arrow.ns              = "nav_points";
        arrow.id              = id++;
        arrow.type            = visualization_msgs::msg::Marker::ARROW;
        arrow.action          = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = pt.x;
        arrow.pose.position.y = pt.y;
        arrow.pose.position.z = 0.15;
        {
            double half_yaw = pt.yaw * 0.5;
            arrow.pose.orientation.z = std::sin(half_yaw);
            arrow.pose.orientation.w = std::cos(half_yaw);
        }
        arrow.scale.x = 0.5;   // 箭头长度
        arrow.scale.y = 0.08;  // 箭头宽度
        arrow.scale.z = 0.08;  // 箭头高度
        arrow.color   = getColorForType(pt.type);
        arrow.color.a = 1.0f;
        arrow.lifetime = rclcpp::Duration::from_seconds(0);
        arr.markers.push_back(arrow);

        // ③ 名称文本标签
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "map";
        text.header.stamp    = node_->now();
        text.ns              = "nav_points";
        text.id              = id++;
        text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action          = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = pt.x;
        text.pose.position.y = pt.y;
        text.pose.position.z = 0.6;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.25;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text    = pt.name;
        text.lifetime = rclcpp::Duration::from_seconds(0);
        arr.markers.push_back(text);
    }
    marker_pub_->publish(arr);
}

visualization_msgs::msg::Marker NavPointManager::createPointMarker(const NavPoint& pt, int id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = node_->now();
    m.ns              = "nav_points";
    m.id              = id;
    m.type            = visualization_msgs::msg::Marker::CYLINDER;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pt.x;
    m.pose.position.y = pt.y;
    m.pose.position.z = 0.15;

    double half_yaw = pt.yaw * 0.5;
    m.pose.orientation.z = std::sin(half_yaw);
    m.pose.orientation.w = std::cos(half_yaw);

    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.color   = getColorForType(pt.type);
    m.lifetime = rclcpp::Duration::from_seconds(0);
    return m;
}

std_msgs::msg::ColorRGBA NavPointManager::getColorForType(const std::string& type) {
    std_msgs::msg::ColorRGBA c;
    c.a = 1.0;
    if (type == "CHARGING") {
        c.r = 0.0; c.g = 1.0; c.b = 0.0;
    } else if (type == "STATION") {
        c.r = 0.0; c.g = 0.5; c.b = 1.0;
    } else if (type == "DOORWAIT") {
        c.r = 1.0; c.g = 0.5; c.b = 0.0;
    } else if (type == "PARKING") {
        c.r = 0.5; c.g = 0.5; c.b = 0.5;
    } else {
        c.r = 1.0; c.g = 1.0; c.b = 1.0;
    }
    return c;
}

}  // namespace map_asset_manager

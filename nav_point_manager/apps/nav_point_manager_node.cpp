#include "nav_point_manager/nav_point_manager_node.hpp"

#include <cmath>
#include <thread>

namespace nav_point_manager {

NavPointManagerNode::NavPointManagerNode() : Node("nav_point_manager_node") {
    // 声明参数
    this->declare_parameter<std::string>("data_base_path", "/home/dexforce/.dexe/data/maps");
    this->declare_parameter<std::string>("default_map_folder", "map_2_standard");
    this->declare_parameter<std::string>("points_file", "");
    this->declare_parameter<std::string>("routes_file", "");
    this->declare_parameter<bool>("auto_save", true);

    data_base_path_ = this->get_parameter("data_base_path").as_string();
    default_map_folder_ = this->get_parameter("default_map_folder").as_string();
    points_file_ = this->get_parameter("points_file").as_string();
    routes_file_ = this->get_parameter("routes_file").as_string();
    auto_save_ = this->get_parameter("auto_save").as_bool();

    // 如果points_file和routes_file为空，使用默认路径
    if (points_file_.empty()) {
        points_file_ = data_base_path_ + "/" + default_map_folder_ + "/points.json";
    }
    if (routes_file_.empty()) {
        routes_file_ = data_base_path_ + "/" + default_map_folder_ + "/routes.json";
    }

    RCLCPP_INFO(this->get_logger(), "Nav Point Manager Node initializing...");
    RCLCPP_INFO(this->get_logger(), "  Data base path: %s", data_base_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Default map folder: %s", default_map_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Points file: %s", points_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Routes file: %s", routes_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Auto save: %s", auto_save_ ? "true" : "false");

    initialize();
    loadData();

    RCLCPP_INFO(this->get_logger(), "Nav Point Manager Node initialized successfully");
}

void NavPointManagerNode::initialize() {
    // 创建核心组件
    file_manager_ = std::make_shared<FileManager>(this->get_logger());
    database_ = std::make_shared<PointDatabase>(this->get_logger());
    // nav_executor_ 将在 initializeActionServers() 中创建

    // 创建发布器
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/nav_points_marker", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("~/navigation_status", 10);
    route_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/route_markers", 10);

    // 创建服务 - 点位管理
    add_point_srv_ = this->create_service<nav_interfaces::srv::AddNavPoint>(
        "~/add_point",
        std::bind(&NavPointManagerNode::addPointCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_point_srv_ = this->create_service<nav_interfaces::srv::RemoveNavPoint>(
        "~/remove_point",
        std::bind(&NavPointManagerNode::removePointCallback, this, std::placeholders::_1, std::placeholders::_2));

    update_point_srv_ = this->create_service<nav_interfaces::srv::UpdateNavPoint>(
        "~/update_point",
        std::bind(&NavPointManagerNode::updatePointCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_point_srv_ = this->create_service<nav_interfaces::srv::GetNavPoint>(
        "~/get_point",
        std::bind(&NavPointManagerNode::getPointCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_all_points_srv_ = this->create_service<nav_interfaces::srv::GetAllNavPoints>(
        "~/get_all_points",
        std::bind(&NavPointManagerNode::getAllPointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务 - 路网管理
    add_route_srv_ = this->create_service<nav_interfaces::srv::AddRoute>(
        "~/add_route",
        std::bind(&NavPointManagerNode::addRouteCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_route_srv_ = this->create_service<nav_interfaces::srv::RemoveRoute>(
        "~/remove_route",
        std::bind(&NavPointManagerNode::removeRouteCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_route_srv_ = this->create_service<nav_interfaces::srv::GetRoute>(
        "~/get_route",
        std::bind(&NavPointManagerNode::getRouteCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务 - 其他
    reload_points_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reload_points",
        std::bind(&NavPointManagerNode::reloadPointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_status_srv_ = this->create_service<nav_interfaces::srv::GetStatus>(
        "~/get_status",
        std::bind(&NavPointManagerNode::getStatusCallback, this, std::placeholders::_1, std::placeholders::_2));

    load_point_srv_ = this->create_service<nav_interfaces::srv::LoadPoint>(
        "~/load_point",
        std::bind(&NavPointManagerNode::loadPointCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建定时器
    marker_timer_ =
        this->create_wall_timer(std::chrono::seconds(5), std::bind(&NavPointManagerNode::publishMarkers, this));

    status_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                            std::bind(&NavPointManagerNode::statusTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "All services, actions and publishers created");
}

void NavPointManagerNode::loadData() {
    if (points_file_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points file specified");
        return;
    }

    // 加载点位
    std::unordered_map<std::string, NavPoint> points_map;
    if (file_manager_->loadPointsFromFile(points_file_, points_map)) {
        database_->setPoints(points_map);
        RCLCPP_INFO(this->get_logger(), "Loaded %zu points", points_map.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load points from file");
    }

    // 加载路网
    if (!routes_file_.empty()) {
        std::unordered_map<std::string, Route> routes_map;
        if (file_manager_->loadRoutesFromFile(routes_file_, routes_map)) {
            database_->setRoutes(routes_map);
            RCLCPP_INFO(this->get_logger(), "Loaded %zu routes", routes_map.size());
        }
    }

    // 发布初始标记
    publishMarkers();
}

void NavPointManagerNode::initializeActionServers() {
    // 创建导航执行器（需要 shared_from_this）
    nav_executor_ = std::make_shared<NavigationExecutor>(this->shared_from_this(), database_);

    // 创建 Action 服务器（必须在构造函数完成后调用）
    navigate_to_point_action_ = rclcpp_action::create_server<NavigateToPointAction>(
        this->shared_from_this(), "~/navigate_to_point",
        std::bind(&NavPointManagerNode::handleNavigateToPointGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavPointManagerNode::handleNavigateToPointCancel, this, std::placeholders::_1),
        std::bind(&NavPointManagerNode::handleNavigateToPointAccepted, this, std::placeholders::_1));

    navigate_route_action_ = rclcpp_action::create_server<NavigateRouteAction>(
        this->shared_from_this(), "~/navigate_route",
        std::bind(&NavPointManagerNode::handleNavigateRouteGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavPointManagerNode::handleNavigateRouteCancel, this, std::placeholders::_1),
        std::bind(&NavPointManagerNode::handleNavigateRouteAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action servers initialized");
}

void NavPointManagerNode::savePointsToFile() {
    if (auto_save_ && !points_file_.empty()) {
        auto points_map = database_->getPointsMap();
        file_manager_->savePointsToFile(points_file_, points_map);
    }
}

void NavPointManagerNode::saveRoutesToFile() {
    if (auto_save_ && !routes_file_.empty()) {
        auto routes_map = database_->getRoutesMap();
        file_manager_->saveRoutesToFile(routes_file_, routes_map);
    }
}

// ============================================================
// 点位管理服务回调
// ============================================================

void NavPointManagerNode::addPointCallback(const std::shared_ptr<nav_interfaces::srv::AddNavPoint::Request> request,
                                           std::shared_ptr<nav_interfaces::srv::AddNavPoint::Response> response) {
    NavPoint point;
    point.name = request->point.name;
    point.type = request->point.type;
    point.x = request->point.x;
    point.y = request->point.y;
    point.yaw = request->point.theta;
    point.tolerance = request->point.tolerance;

    int result = database_->addPoint(point);
    if (result == 1) {
        // 新增
        response->success = true;
        response->message = "Point added successfully: " + point.name;
        savePointsToFile();
        publishMarkers();
        RCLCPP_INFO(this->get_logger(), "Added point: %s", point.name.c_str());
    } else if (result == 2) {
        // 覆盖
        response->success = true;
        response->message = "Point overwritten (already existed): " + point.name;
        savePointsToFile();
        publishMarkers();
        RCLCPP_INFO(this->get_logger(), "Overwritten point: %s", point.name.c_str());
    } else {
        response->success = false;
        response->message = "Failed to add point: " + point.name;
        RCLCPP_WARN(this->get_logger(), "Failed to add point: %s", point.name.c_str());
    }
}

void NavPointManagerNode::removePointCallback(
    const std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Request> request,
    std::shared_ptr<nav_interfaces::srv::RemoveNavPoint::Response> response) {
    if (database_->removePoint(request->name)) {
        response->success = true;
        response->message = "Point removed successfully: " + request->name;
        savePointsToFile();
        publishMarkers();
        RCLCPP_INFO(this->get_logger(), "Removed point: %s", request->name.c_str());
    } else {
        response->success = false;
        response->message = "Failed to remove point (not found): " + request->name;
        RCLCPP_WARN(this->get_logger(), "Failed to remove point: %s", request->name.c_str());
    }
}

void NavPointManagerNode::updatePointCallback(
    const std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Request> request,
    std::shared_ptr<nav_interfaces::srv::UpdateNavPoint::Response> response) {
    NavPoint point;
    point.name = request->point.name;
    point.type = request->point.type;
    point.x = request->point.x;
    point.y = request->point.y;
    point.yaw = request->point.theta;
    point.tolerance = request->point.tolerance;

    if (database_->updatePoint(point)) {
        response->success = true;
        response->message = "Point updated successfully: " + point.name;
        savePointsToFile();
        publishMarkers();
        RCLCPP_INFO(this->get_logger(), "Updated point: %s", point.name.c_str());
    } else {
        response->success = false;
        response->message = "Failed to update point (not found): " + point.name;
        RCLCPP_WARN(this->get_logger(), "Failed to update point: %s", point.name.c_str());
    }
}

void NavPointManagerNode::getPointCallback(const std::shared_ptr<nav_interfaces::srv::GetNavPoint::Request> request,
                                           std::shared_ptr<nav_interfaces::srv::GetNavPoint::Response> response) {
    auto point_opt = database_->getPoint(request->name);
    if (point_opt.has_value()) {
        const auto& point = point_opt.value();
        response->success = true;
        response->point.name = point.name;
        response->point.type = point.type;
        response->point.x = point.x;
        response->point.y = point.y;
        response->point.theta = point.yaw;
        response->point.tolerance = point.tolerance;
    } else {
        response->success = false;
        RCLCPP_WARN(this->get_logger(), "Point not found: %s", request->name.c_str());
    }
}

void NavPointManagerNode::getAllPointsCallback(
    const std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Request> request,
    std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Response> response) {
    (void)request;

    auto points = database_->getAllPoints();
    response->success = true;

    for (const auto& point : points) {
        nav_interfaces::msg::NavPoint point_msg;
        point_msg.name = point.name;
        point_msg.type = point.type;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.theta = point.yaw;
        point_msg.tolerance = point.tolerance;
        response->points.push_back(point_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Returned %zu points", points.size());
}

// ============================================================
// 路网管理服务回调
// ============================================================

void NavPointManagerNode::addRouteCallback(const std::shared_ptr<nav_interfaces::srv::AddRoute::Request> request,
                                           std::shared_ptr<nav_interfaces::srv::AddRoute::Response> response) {
    Route route;
    route.name = request->name;
    route.point_names = request->point_names;

    if (database_->addRoute(route)) {
        response->success = true;
        response->message = "Route added successfully: " + route.name;
        saveRoutesToFile();
        RCLCPP_INFO(this->get_logger(), "Added route: %s with %zu points", route.name.c_str(),
                    route.point_names.size());
    } else {
        response->success = false;
        response->message = "Failed to add route (may already exist): " + route.name;
        RCLCPP_WARN(this->get_logger(), "Failed to add route: %s", route.name.c_str());
    }
}

void NavPointManagerNode::removeRouteCallback(const std::shared_ptr<nav_interfaces::srv::RemoveRoute::Request> request,
                                              std::shared_ptr<nav_interfaces::srv::RemoveRoute::Response> response) {
    if (database_->removeRoute(request->name)) {
        response->success = true;
        response->message = "Route removed successfully: " + request->name;
        saveRoutesToFile();
        RCLCPP_INFO(this->get_logger(), "Removed route: %s", request->name.c_str());
    } else {
        response->success = false;
        response->message = "Failed to remove route (not found): " + request->name;
        RCLCPP_WARN(this->get_logger(), "Failed to remove route: %s", request->name.c_str());
    }
}

void NavPointManagerNode::getRouteCallback(const std::shared_ptr<nav_interfaces::srv::GetRoute::Request> request,
                                           std::shared_ptr<nav_interfaces::srv::GetRoute::Response> response) {
    auto route_opt = database_->getRoute(request->name);
    if (route_opt.has_value()) {
        const auto& route = route_opt.value();
        response->success = true;
        response->name = route.name;
        response->point_names = route.point_names;
    } else {
        response->success = false;
        RCLCPP_WARN(this->get_logger(), "Route not found: %s", request->name.c_str());
    }
}

// ============================================================
// 其他服务回调
// ============================================================

void NavPointManagerNode::reloadPointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    RCLCPP_INFO(this->get_logger(), "Reloading points and routes...");
    loadData();

    response->success = true;
    response->message = "Points and routes reloaded successfully";
}

void NavPointManagerNode::getStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> request,
                                            std::shared_ptr<nav_interfaces::srv::GetStatus::Response> response) {
    (void)request;

    auto state = nav_executor_->getCurrentState();
    auto target = nav_executor_->getCurrentTarget();

    response->success = true;
    response->name = target;

    switch (state) {
        case NavState::IDLE:
            response->status = "IDLE";
            break;
        case NavState::NAVIGATING:
            response->status = "NAVIGATING";
            break;
        case NavState::SUCCESS:
            response->status = "SUCCESS";
            break;
        case NavState::FAILED:
            response->status = "FAILED";
            break;
        case NavState::CANCELED:
            response->status = "CANCELED";
            break;
    }
}

void NavPointManagerNode::loadPointCallback(const std::shared_ptr<nav_interfaces::srv::LoadPoint::Request> request,
                                            std::shared_ptr<nav_interfaces::srv::LoadPoint::Response> response) {
    std::string folder_name = request->folder_name;

    if (folder_name.empty()) {
        response->success = false;
        response->message = "folder_name cannot be empty";
        RCLCPP_WARN(this->get_logger(), "LoadPoint: folder_name is empty");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading map folder: %s", folder_name.c_str());

    // 使用FileManager获取文件路径
    std::string new_points_file;
    std::string new_routes_file;
    if (!file_manager_->loadMapFolder(data_base_path_, folder_name, new_points_file, new_routes_file)) {
        response->success = false;
        response->message = "Failed to load map folder: " + folder_name;
        RCLCPP_ERROR(this->get_logger(), "Failed to load map folder: %s", folder_name.c_str());
        return;
    }

    // 更新当前文件路径
    points_file_ = new_points_file;
    routes_file_ = new_routes_file;

    // 重新加载数据
    loadData();

    response->success = true;
    response->message = "Loaded map folder: " + folder_name;
    RCLCPP_INFO(this->get_logger(), "Successfully loaded map folder: %s", folder_name.c_str());
}

// ============================================================
// Action 回调 - 导航到点位
// ============================================================

rclcpp_action::GoalResponse NavPointManagerNode::handleNavigateToPointGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateToPointAction::Goal> goal) {
    (void)uuid;

    RCLCPP_INFO(this->get_logger(), "Received navigate_to_point goal: %s", goal->point_name.c_str());

    // 检查点位是否存在
    if (!goal->point_name.empty() && !database_->hasPoint(goal->point_name)) {
        RCLCPP_WARN(this->get_logger(), "Point not found: %s", goal->point_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavPointManagerNode::handleNavigateToPointCancel(
    const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Received cancel request for navigate_to_point");
    nav_executor_->cancelNavigation();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPointManagerNode::handleNavigateToPointAccepted(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    // 在新线程中执行导航
    std::thread{std::bind(&NavPointManagerNode::executeNavigateToPoint, this, goal_handle)}.detach();
}

void NavPointManagerNode::executeNavigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPointAction::Feedback>();
    auto result = std::make_shared<NavigateToPointAction::Result>();

    RCLCPP_INFO(this->get_logger(), "Executing navigate_to_point: %s", goal->point_name.c_str());

    // 开始导航
    bool nav_started = false;
    if (!goal->point_name.empty()) {
        // 使用点位名称导航
        nav_started = nav_executor_->navigateToPoint(goal->point_name);
    } else if (goal->use_pose) {
        // 使用自定义姿态导航
        nav_started = nav_executor_->navigateToPose(goal->pose, goal->tolerance);
    }

    if (!nav_started) {
        result->success = false;
        result->message = "Failed to start navigation";
        goal_handle->abort(result);
        return;
    }

    // 等待导航完成，定期发布反馈
    rclcpp::Rate rate(2);  // 2 Hz
    while (rclcpp::ok()) {
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            nav_executor_->cancelNavigation();
            result->success = false;
            result->message = "Navigation canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Navigation canceled");
            return;
        }

        // 获取当前状态
        auto state = nav_executor_->getCurrentState();

        // 发布反馈
        feedback->current_status = [state]() {
            switch (state) {
                case NavState::IDLE:
                    return "IDLE";
                case NavState::NAVIGATING:
                    return "NAVIGATING";
                case NavState::SUCCESS:
                    return "SUCCESS";
                case NavState::FAILED:
                    return "FAILED";
                case NavState::CANCELED:
                    return "CANCELED";
                default:
                    return "UNKNOWN";
            }
        }();
        goal_handle->publish_feedback(feedback);

        // 检查是否完成
        if (state == NavState::SUCCESS) {
            result->success = true;
            result->message = "Navigation succeeded";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            return;
        } else if (state == NavState::FAILED) {
            result->success = false;
            result->message = "Navigation failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            return;
        } else if (state == NavState::CANCELED) {
            result->success = false;
            result->message = "Navigation canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Navigation canceled");
            return;
        }

        rate.sleep();
    }
}

// ============================================================
// Action 回调 - 路网导航
// ============================================================

rclcpp_action::GoalResponse NavPointManagerNode::handleNavigateRouteGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateRouteAction::Goal> goal) {
    (void)uuid;

    RCLCPP_INFO(this->get_logger(), "Received navigate_route goal: %s", goal->route_name.c_str());

    // 检查路网是否存在
    if (!database_->hasRoute(goal->route_name)) {
        RCLCPP_WARN(this->get_logger(), "Route not found: %s", goal->route_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavPointManagerNode::handleNavigateRouteCancel(
    const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Received cancel request for navigate_route");
    nav_executor_->cancelNavigation();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPointManagerNode::handleNavigateRouteAccepted(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    // 在新线程中执行路网导航
    std::thread{std::bind(&NavPointManagerNode::executeNavigateRoute, this, goal_handle)}.detach();
}

void NavPointManagerNode::executeNavigateRoute(const std::shared_ptr<GoalHandleNavigateRoute> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateRouteAction::Feedback>();
    auto result = std::make_shared<NavigateRouteAction::Result>();

    RCLCPP_INFO(this->get_logger(), "Executing navigate_route: %s", goal->route_name.c_str());

    // 获取路网信息并发布路径可视化
    auto route_opt = database_->getRoute(goal->route_name);
    if (route_opt.has_value()) {
        const auto& route = route_opt.value();
        std::vector<NavPoint> route_points;

        // 获取路网中所有点位
        for (const auto& point_name : route.point_names) {
            auto point_opt = database_->getPoint(point_name);
            if (point_opt.has_value()) {
                route_points.push_back(point_opt.value());
            }
        }

        // 发布路径可视化
        if (!route_points.empty()) {
            publishRoutePath(route_points);
        }
    }

    // 开始路网导航
    bool nav_started = nav_executor_->navigateRoute(goal->route_name);

    if (!nav_started) {
        result->success = false;
        result->message = "Failed to start route navigation";
        goal_handle->abort(result);
        return;
    }

    // 等待导航完成，定期发布反馈
    rclcpp::Rate rate(2);  // 2 Hz
    while (rclcpp::ok()) {
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            nav_executor_->cancelNavigation();
            result->success = false;
            result->message = "Route navigation canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Route navigation canceled");
            return;
        }

        // 获取当前状态
        auto state = nav_executor_->getCurrentState();

        // 发布反馈
        feedback->current_status = [state]() {
            switch (state) {
                case NavState::IDLE:
                    return "IDLE";
                case NavState::NAVIGATING:
                    return "NAVIGATING";
                case NavState::SUCCESS:
                    return "SUCCESS";
                case NavState::FAILED:
                    return "FAILED";
                case NavState::CANCELED:
                    return "CANCELED";
                default:
                    return "UNKNOWN";
            }
        }();
        goal_handle->publish_feedback(feedback);

        // 检查是否完成
        if (state == NavState::SUCCESS) {
            result->success = true;
            result->message = "Route navigation succeeded";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Route navigation succeeded");
            return;
        } else if (state == NavState::FAILED) {
            result->success = false;
            result->message = "Route navigation failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Route navigation failed");
            return;
        } else if (state == NavState::CANCELED) {
            result->success = false;
            result->message = "Route navigation canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Route navigation canceled");
            return;
        }

        rate.sleep();
    }
}

// ============================================================
// 辅助函数
// ============================================================

void NavPointManagerNode::publishMarkers() {
    auto points = database_->getAllPoints();
    if (points.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto& point : points) {
        auto marker = createPointMarker(point, id++);
        marker_array.markers.push_back(marker);

        // 添加文本标签
        auto text_marker = marker;
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.text = point.name;
        text_marker.pose.position.z += 0.5;
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        marker_array.markers.push_back(text_marker);
    }

    marker_pub_->publish(marker_array);
}

void NavPointManagerNode::publishRoutePath(const std::vector<NavPoint>& route_points) {
    if (route_points.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (size_t i = 0; i < route_points.size(); ++i) {
        const auto& point = route_points[i];

        // 1. 创建箭头标记（显示姿态）
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "route_arrows";
        arrow_marker.id = id++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置和姿态
        arrow_marker.pose.position.x = point.x;
        arrow_marker.pose.position.y = point.y;
        arrow_marker.pose.position.z = 0.0;

        // 使用 yaw 角度计算四元数（不依赖 tf2）
        double half_yaw = point.yaw * 0.5;
        arrow_marker.pose.orientation.x = 0.0;
        arrow_marker.pose.orientation.y = 0.0;
        arrow_marker.pose.orientation.z = std::sin(half_yaw);
        arrow_marker.pose.orientation.w = std::cos(half_yaw);

        // 设置箭头大小
        arrow_marker.scale.x = 0.5;  // 箭头长度
        arrow_marker.scale.y = 0.1;  // 箭头宽度
        arrow_marker.scale.z = 0.1;  // 箭头高度

        // 设置颜色（蓝色，带透明度）
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.5;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.8;

        arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 永久显示
        marker_array.markers.push_back(arrow_marker);

        // 2. 创建球体标记（路径点）
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header = arrow_marker.header;
        sphere_marker.ns = "route_points";
        sphere_marker.id = id++;
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;

        sphere_marker.pose.position.x = point.x;
        sphere_marker.pose.position.y = point.y;
        sphere_marker.pose.position.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;

        // 球体大小
        sphere_marker.scale.x = 0.3;
        sphere_marker.scale.y = 0.3;
        sphere_marker.scale.z = 0.3;

        // 颜色（黄色）
        sphere_marker.color.r = 1.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 0.0;
        sphere_marker.color.a = 1.0;

        sphere_marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(sphere_marker);

        // 3. 创建文本标记（点位名称和序号）
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = arrow_marker.header;
        text_marker.ns = "route_labels";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = point.x;
        text_marker.pose.position.y = point.y;
        text_marker.pose.position.z = 0.6;  // 文字显示在上方
        text_marker.pose.orientation.w = 1.0;

        // 文本内容：序号 + 点位名称
        text_marker.text = std::to_string(i + 1) + ". " + point.name;

        // 文字大小
        text_marker.scale.z = 0.3;

        // 文字颜色（白色）
        text_marker.color.r = 0.1;
        text_marker.color.g = 0.1;
        text_marker.color.b = 0.1;
        text_marker.color.a = 1.0;

        text_marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(text_marker);

        // 4. 如果不是最后一个点，添加连接线
        if (i < route_points.size() - 1) {
            const auto& next_point = route_points[i + 1];

            visualization_msgs::msg::Marker line_marker;
            line_marker.header = arrow_marker.header;
            line_marker.ns = "route_lines";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;

            line_marker.pose.orientation.w = 1.0;

            // 添加起点和终点
            geometry_msgs::msg::Point p1, p2;
            p1.x = point.x;
            p1.y = point.y;
            p1.z = 0.05;
            p2.x = next_point.x;
            p2.y = next_point.y;
            p2.z = 0.05;

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);

            // 线条宽度
            line_marker.scale.x = 0.05;

            // 线条颜色（青色）
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 0.6;

            line_marker.lifetime = rclcpp::Duration::from_seconds(0);
            marker_array.markers.push_back(line_marker);
        }
    }

    route_marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published route visualization with %zu points", route_points.size());
}

void NavPointManagerNode::statusTimerCallback() {
    auto state = nav_executor_->getCurrentState();
    auto target = nav_executor_->getCurrentTarget();

    std_msgs::msg::String status_msg;

    switch (state) {
        case NavState::IDLE:
            status_msg.data = "IDLE";
            break;
        case NavState::NAVIGATING:
            status_msg.data = "NAVIGATING to " + target;
            break;
        case NavState::SUCCESS:
            status_msg.data = "SUCCESS";
            break;
        case NavState::FAILED:
            status_msg.data = "FAILED";
            break;
        case NavState::CANCELED:
            status_msg.data = "CANCELED";
            break;
    }

    status_pub_->publish(status_msg);
}

visualization_msgs::msg::Marker NavPointManagerNode::createPointMarker(const NavPoint& point, int id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "nav_points";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;

    marker.color = getColorForType(point.type);
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 永久

    return marker;
}

std_msgs::msg::ColorRGBA NavPointManagerNode::getColorForType(const std::string& type) {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.8;

    if (type == "CHARGING") {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;  // 红色
    } else if (type == "STATION") {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;  // 蓝色
    } else if (type == "PICKUP") {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;  // 绿色
    } else if (type == "PARKING") {
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;  // 黄色
    } else {
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;  // 灰色（默认）
    }

    return color;
}

}  // namespace nav_point_manager

#include "map_asset_manager/map_asset_manager_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;

namespace map_asset_manager {

MapAssetManagerNode::MapAssetManagerNode()
    : rclcpp::Node("map_asset_manager_node") {
    declare_parameter("auto_save", true);
    auto_save_ = get_parameter("auto_save").as_bool();

    initialize();

    RCLCPP_INFO(get_logger(), "MapAssetManagerNode constructed, waiting for map_manager/map_folder and map_name...");
}

// ──────────────────────────────────────────────────────────────────────
// initialize
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::initialize() {
    // ── 共享组件 ───────────────────────────────────────────────────
    event_bus_     = std::make_shared<DataEventBus>();
    file_manager_  = std::make_shared<FileManager>(get_logger());
    point_store_   = std::make_shared<PointStore>(get_logger());
    route_store_   = std::make_shared<RouteStore>(get_logger());
    area_store_    = std::make_shared<AreaStore>(get_logger());
    room_store_    = std::make_shared<RoomStore>(get_logger());
    junction_store_= std::make_shared<JunctionStore>(get_logger());
    edge_store_    = std::make_shared<RouteEdgeStore>(get_logger());

    // save_fn：三个子管理器共用同一个保存回调
    auto save_fn = [this]() { saveAllAssets(); };

    // ── 子管理器 ───────────────────────────────────────────────────
    point_mgr_ = std::make_shared<NavPointManager>(
        this, point_store_, event_bus_, save_fn);

    route_mgr_ = std::make_shared<RouteManager>(
        this, route_store_, point_store_, event_bus_, save_fn);

    area_mgr_ = std::make_shared<AreaManager>(
        this, area_store_, event_bus_, save_fn);

    room_mgr_ = std::make_shared<RoomManager>(
        this, room_store_, event_bus_, save_fn);

    route_graph_mgr_ = std::make_shared<RouteGraphManager>(
        this, junction_store_, edge_store_, point_store_, event_bus_, save_fn);

    // ── 空间归属索引引擎（事件驱动自动维护 Area/Room 内的 NavPoint+Junction）──
    spatial_indexer_ = std::make_shared<SpatialIndexer>(
        this,
        point_store_,
        junction_store_,
        area_store_,
        room_store_,
        event_bus_,
        /*on_areas_changed=*/ [this]() {
            if (area_mgr_) {
                area_mgr_->publishMarkers();
                area_mgr_->publishAreas();
            }
            // 持久化 related_* 字段（SpatialIndexer 重算结果写回 JSON）
            saveAllAssets();
        },
        /*on_rooms_changed=*/ [this]() {
            if (room_mgr_) {
                room_mgr_->publishMarkers();
                room_mgr_->publishRooms();
            }
            saveAllAssets();
        });

    // ── 通用服务 & 发布器 ──────────────────────────────────────────
    reload_data_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reload_data",
        std::bind(&MapAssetManagerNode::reloadDataCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    save_assets_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/save_assets",
        std::bind(&MapAssetManagerNode::saveAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    transform_assets_srv_ = create_service<nav_interfaces::srv::TransformAssets>(
        "~/transform_assets",
        std::bind(&MapAssetManagerNode::transformAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // status 用 transient_local：后启动订阅者自动收到最近一次状态（latched）
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "~/status", rclcpp::QoS(1).transient_local());

    // 注：markers 与 status 均改为事件驱动（数据变化时由 saveAllAssets / loadData 触发），
    // 不再周期重发；配合各 publisher 的 transient_local QoS，rviz 重启也能收到最新。

    // ── 订阅 map_manager/map_folder 和 map_name（transient_local）──
    auto qos = rclcpp::QoS(1).transient_local().reliable();

    map_folder_sub_ = create_subscription<std_msgs::msg::String>(
        "map_manager/map_folder", qos,
        std::bind(&MapAssetManagerNode::onMapFolderChanged, this, std::placeholders::_1));

    map_name_sub_ = create_subscription<std_msgs::msg::String>(
        "map_manager/map_name", qos,
        std::bind(&MapAssetManagerNode::onMapNameChanged, this, std::placeholders::_1));

    // ── EventBus 订阅 ──────────────────────────────────────────────
    registerEventHandlers();
}

// ──────────────────────────────────────────────────────────────────────
// onMapFolderChanged / onMapNameChanged
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::onMapFolderChanged(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& new_folder = msg->data;
    if (new_folder.empty() || new_folder == map_base_folder_) return;

    map_base_folder_ = new_folder;
    RCLCPP_INFO(get_logger(), "map_base_folder: '%s'", map_base_folder_.c_str());

    // 若 map_name 已经到达，立即加载
    if (!current_map_name_.empty()) {
        switchToMap(current_map_name_);
    }
}

void MapAssetManagerNode::onMapNameChanged(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& new_name = msg->data;
    if (new_name.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty map_name, ignoring");
        return;
    }
    if (new_name == current_map_name_) return;

    RCLCPP_INFO(get_logger(), "Map name changed: '%s' -> '%s'",
                current_map_name_.c_str(), new_name.c_str());
    current_map_name_ = new_name;

    if (map_base_folder_.empty()) {
        RCLCPP_WARN(get_logger(), "map_folder not yet received, will load when it arrives");
        return;
    }

    switchToMap(new_name);
}

// ──────────────────────────────────────────────────────────────────────
// switchToMap
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::switchToMap(const std::string& map_name) {
    assets_file_ = map_base_folder_ + "/" + map_name + "/map_assets.json";

    point_store_->clear();
    route_store_->clear();
    area_store_->clear();
    room_store_->clear();
    junction_store_->clear();
    edge_store_->clear();

    loadData();

    event_bus_->publish({AssetEventType::DATA_RELOADED, "", ""});
}

// ──────────────────────────────────────────────────────────────────────
// loadData
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::loadData() {
    if (assets_file_.empty()) {
        RCLCPP_WARN(get_logger(), "assets_file_ is empty, skip loading");
        return;
    }

    // 确保目录存在
    fs::create_directories(fs::path(assets_file_).parent_path());

    std::unordered_map<std::string, NavPoint> points;
    std::unordered_map<std::string, Route>    routes;
    std::unordered_map<std::string, Area>     areas;
    std::vector<RouteJunction>                junctions;
    std::vector<RouteEdge>                    edges;
    std::unordered_map<std::string, Room>     rooms;

    file_manager_->loadAllAssetsFromFile(assets_file_, points, routes, areas, junctions, edges, rooms);

    for (auto& [name, pt] : points) {
        point_store_->addPoint(pt);
    }
    for (auto& [name, rt] : routes) {
        route_store_->addRoute(rt);
    }
    for (auto& [name, area] : areas) {
        area_store_->addArea(area);
    }
    for (auto& [name, room] : rooms) {
        room_store_->addRoom(room);
    }
    junction_store_->set(junctions);
    edge_store_->set(edges);

    RCLCPP_INFO(get_logger(),
                "Loaded from %s: %zu points, %zu routes, %zu areas, %zu rooms, %zu junctions, %zu edges",
                assets_file_.c_str(), points.size(), routes.size(), areas.size(), rooms.size(),
                junctions.size(), edges.size());

    point_mgr_->publishMarkers();
    area_mgr_->publishMarkers();
    area_mgr_->publishAreas();
    room_mgr_->publishMarkers();
    room_mgr_->publishRooms();
    route_mgr_->publishAllRouteMarkers();
    route_graph_mgr_->publishMarkers();

    publishStatus();
}

// ──────────────────────────────────────────────────────────────────────
// saveAllAssets
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::saveAllAssets() {
    // status 反映 store 当前状态：即使 auto_save=false 或 assets_file_ 未就绪，
    // 数据已变化也要刷新一次（事件驱动 status，无周期 timer）
    publishStatus();

    if (!auto_save_) return;
    if (assets_file_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "saveAllAssets: assets_file_ not set yet (map_manager not ready and data_dir not configured)");
        return;
    }
    file_manager_->saveAllAssetsToFile(
        assets_file_,
        point_store_->getPointsMap(),
        route_store_->getRoutesMap(),
        area_store_->getAreasMap(),
        junction_store_->getList(),
        edge_store_->getAll(),
        room_store_->getRoomsMap());
}

// ──────────────────────────────────────────────────────────────────────
// initializeActionServers（构造完成后调用）
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::initializeActionServers() {
    auto self = shared_from_this();

    // NavigationExecutor 需要 shared_from_this()，在此创建
    nav_executor_ = std::make_shared<NavigationExecutor>(self);

    point_mgr_->initializeActionServers(self, nav_executor_);
    route_mgr_->initializeActionServers(self, nav_executor_);
    RCLCPP_INFO(get_logger(), "MapAssetManagerNode: action servers initialized");
}

// ──────────────────────────────────────────────────────────────────────
// EventBus 订阅
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::registerEventHandlers() {
    // 导航点被删除 → 路线级联清理 + 路网边级联清理
    event_bus_->subscribe(AssetEventType::POINT_REMOVED,
        [this](const AssetEvent& e) {
            route_mgr_->onPointRemoved(e.name);
            route_graph_mgr_->onNavPointRemoved(e.name);
        });

    // 导航点被更新（重命名）→ 路线级联更新
    event_bus_->subscribe(AssetEventType::POINT_UPDATED,
        [this](const AssetEvent& e) {
            if (!e.old_name.empty() && e.old_name != e.name) {
                route_mgr_->onPointRenamed(e.old_name, e.name);
            }
        });
}

// ──────────────────────────────────────────────────────────────────────
// 服务 / 定时器回调
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::reloadDataCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
        if (assets_file_.empty()) {
            res->success = false;
            res->message = "No map loaded yet (waiting for map_manager/map_name)";
            return;
        }

        // 清空各 store
        point_store_->clear();
        route_store_->clear();
        area_store_->clear();
        room_store_->clear();
        junction_store_->clear();
        edge_store_->clear();

        // 重新加载
        loadData();

        // 通知所有观察者
        event_bus_->publish({AssetEventType::DATA_RELOADED, "", ""});

        res->success = true;
        res->message = "Data reloaded from " + assets_file_;
        RCLCPP_INFO(get_logger(), "Data reloaded via service");
    } catch (const std::exception& ex) {
        res->success = false;
        res->message = std::string("Reload failed: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "reloadData: %s", ex.what());
    }
}

void MapAssetManagerNode::publishStatus() {
    std_msgs::msg::String msg;
    msg.data =
        "map=" + current_map_name_ +
        " points=" + std::to_string(point_store_->getPointsMap().size()) +
        " routes=" + std::to_string(route_store_->getRoutesMap().size()) +
        " areas="  + std::to_string(area_store_->getAreasMap().size()) +
        " rooms="  + std::to_string(room_store_->getRoomsMap().size());
    status_pub_->publish(msg);
}

void MapAssetManagerNode::saveAssetsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
        if (assets_file_.empty()) {
            res->success = false;
            res->message = "No map loaded yet (waiting for map_manager/map_name)";
            return;
        }
        saveAllAssets();
        res->success = true;
        res->message = "Assets saved to " + assets_file_;
        RCLCPP_INFO(get_logger(), "Assets saved via service to %s", assets_file_.c_str());
    } catch (const std::exception& ex) {
        res->success = false;
        res->message = std::string("Save failed: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "saveAssets: %s", ex.what());
    }
}

// ──────────────────────────────────────────────────────────────────────
// transformAssetsCallback — 对所有资产坐标施加 2D 刚体变换
//
// map_name 为空 或等于 current_map_name_：
//   直接操作内存 store → 持久化 → 重新发布 markers（当前地图，在线变换）
// map_name 为其他非空字符串：
//   从文件加载 → 变换 → 写回文件（非当前地图，离线变换，不触碰内存）
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::transformAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::TransformAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::TransformAssets::Response> res)
{
    const bool is_current = req->map_name.empty() || req->map_name == current_map_name_;

    if (is_current && assets_file_.empty()) {
        res->success = false;
        res->message = "No map loaded (waiting for map_manager/map_name)";
        return;
    }
    if (!is_current && map_base_folder_.empty()) {
        res->success = false;
        res->message = "map_base_folder not set, cannot locate map '" + req->map_name + "'";
        return;
    }

    const double cos_yaw = std::cos(req->yaw);
    const double sin_yaw = std::sin(req->yaw);
    const double ox = req->x;
    const double oy = req->y;

    // 2D 点变换：T_inv * (x, y)
    auto transform_xy = [&](double x, double y, double& nx, double& ny) {
        const double dx = x - ox;
        const double dy = y - oy;
        nx =  cos_yaw * dx + sin_yaw * dy;
        ny = -sin_yaw * dx + cos_yaw * dy;
    };

    // 偏航角变换 + 归一化到 (-π, π]
    auto transform_yaw = [&](double yaw) -> double {
        double ny = yaw - req->yaw;
        while (ny >  M_PI) ny -= 2.0 * M_PI;
        while (ny <= -M_PI) ny += 2.0 * M_PI;
        return ny;
    };

    int n_points = 0, n_areas = 0, n_junctions = 0;

    try {
        if (is_current) {
            // ── 当前地图：直接操作内存 store ─────────────────────────

            // 1. 导航点
            {
                auto pts = point_store_->getAllPoints();
                for (auto& pt : pts) {
                    double nx, ny;
                    transform_xy(pt.x, pt.y, nx, ny);
                    pt.x   = nx;
                    pt.y   = ny;
                    pt.yaw = transform_yaw(pt.yaw);
                    point_store_->updatePoint(pt);
                }
                n_points = static_cast<int>(pts.size());
            }

            // 2. 区域（多边形顶点）
            {
                auto areas = area_store_->getAllAreas();
                for (auto& area : areas) {
                    for (auto& pt : area.polygon) {
                        double nx, ny;
                        transform_xy(pt.x, pt.y, nx, ny);
                        pt.x = nx;
                        pt.y = ny;
                    }
                    area_store_->updateArea(area);
                }
                n_areas = static_cast<int>(areas.size());
            }

            // 3. 路网 Junction
            {
                auto jcts = junction_store_->getAll();
                for (auto& j : jcts) {
                    double nx, ny;
                    transform_xy(j.x, j.y, nx, ny);
                    j.x   = static_cast<float>(nx);
                    j.y   = static_cast<float>(ny);
                    j.yaw = static_cast<float>(transform_yaw(j.yaw));
                    junction_store_->update(j);
                }
                n_junctions = static_cast<int>(jcts.size());
            }

            // 4. 路网 Edge（BEZIER 曲线控制点）
            {
                auto edges = edge_store_->getAll();
                for (auto& e : edges) {
                    if (e.edge_type != "BEZIER" || e.control_points.empty()) continue;
                    for (auto& cp : e.control_points) {
                        double nx, ny;
                        transform_xy(cp.first, cp.second, nx, ny);
                        cp.first  = static_cast<float>(nx);
                        cp.second = static_cast<float>(ny);
                    }
                    edge_store_->update(e);
                }
            }

            // 5. 持久化
            saveAllAssets();

            // 6. 重新发布 markers
            point_mgr_->publishMarkers();
            area_mgr_->publishMarkers();
            area_mgr_->publishAreas();
            route_graph_mgr_->publishMarkers();

            RCLCPP_INFO(get_logger(),
                        "transform_assets [current map '%s']: x=%.3f y=%.3f yaw=%.4f",
                        current_map_name_.c_str(), req->x, req->y, req->yaw);

        } else {
            // ── 非当前地图：从文件加载 → 变换 → 写回 ─────────────────

            const std::string file_path =
                map_base_folder_ + "/" + req->map_name + "/map_assets.json";

            std::unordered_map<std::string, NavPoint> points;
            std::unordered_map<std::string, Route>    routes;
            std::unordered_map<std::string, Area>     areas;
            std::vector<RouteJunction>                junctions;
            std::vector<RouteEdge>                    edges;
            std::unordered_map<std::string, Room>     rooms;

            file_manager_->loadAllAssetsFromFile(
                file_path, points, routes, areas, junctions, edges, rooms);

            // 变换导航点
            for (auto& [name, pt] : points) {
                double nx, ny;
                transform_xy(pt.x, pt.y, nx, ny);
                pt.x   = nx;
                pt.y   = ny;
                pt.yaw = transform_yaw(pt.yaw);
            }

            // 变换区域多边形顶点
            for (auto& [name, area] : areas) {
                for (auto& pt : area.polygon) {
                    double nx, ny;
                    transform_xy(pt.x, pt.y, nx, ny);
                    pt.x = nx;
                    pt.y = ny;
                }
            }

            // 变换路网 Junction
            for (auto& j : junctions) {
                double nx, ny;
                transform_xy(j.x, j.y, nx, ny);
                j.x   = static_cast<float>(nx);
                j.y   = static_cast<float>(ny);
                j.yaw = static_cast<float>(transform_yaw(j.yaw));
            }

            // 变换路网 Edge（BEZIER 曲线控制点）
            for (auto& e : edges) {
                if (e.edge_type != "BEZIER" || e.control_points.empty()) continue;
                for (auto& cp : e.control_points) {
                    double nx, ny;
                    transform_xy(cp.first, cp.second, nx, ny);
                    cp.first  = static_cast<float>(nx);
                    cp.second = static_cast<float>(ny);
                }
            }

            // 变换房间（polygon + pose.position + bounding_box 重算）
            for (auto& [name, r] : rooms) {
                for (auto& pt : r.polygon) {
                    double nx, ny;
                    transform_xy(pt.x, pt.y, nx, ny);
                    pt.x = nx;
                    pt.y = ny;
                }
                double nx, ny;
                transform_xy(r.pose.position.x, r.pose.position.y, nx, ny);
                r.pose.position.x = nx;
                r.pose.position.y = ny;
                // pose.orientation 中 yaw 也应做对应旋转；简化起见此处不动（pose 多用作 centroid）
                // bounding_box：由 polygon 重算
                if (!r.polygon.empty()) {
                    double minx = r.polygon[0].x, miny = r.polygon[0].y;
                    double maxx = r.polygon[0].x, maxy = r.polygon[0].y;
                    for (const auto& pt : r.polygon) {
                        minx = std::min(minx, pt.x); miny = std::min(miny, pt.y);
                        maxx = std::max(maxx, pt.x); maxy = std::max(maxy, pt.y);
                    }
                    r.bounding_box[0] = minx; r.bounding_box[1] = miny; r.bounding_box[2] = 0.0;
                    r.bounding_box[3] = maxx; r.bounding_box[4] = maxy; r.bounding_box[5] = 0.0;
                }
            }

            // 写回文件（不触碰内存 store，不发布 markers）
            file_manager_->saveAllAssetsToFile(
                file_path, points, routes, areas, junctions, edges, rooms);

            n_points    = static_cast<int>(points.size());
            n_areas     = static_cast<int>(areas.size());
            n_junctions = static_cast<int>(junctions.size());

            RCLCPP_INFO(get_logger(),
                        "transform_assets [offline map '%s']: x=%.3f y=%.3f yaw=%.4f",
                        req->map_name.c_str(), req->x, req->y, req->yaw);
        }

        res->success = true;
        res->transformed_points    = n_points;
        res->transformed_areas     = n_areas;
        res->transformed_junctions = n_junctions;
        res->message = std::string(is_current ? "current map" : ("map='" + req->map_name + "'")) +
                       " transformed: points=" + std::to_string(n_points) +
                       " areas=" + std::to_string(n_areas) +
                       " junctions=" + std::to_string(n_junctions);

    } catch (const std::exception& ex) {
        res->success = false;
        res->message = std::string("Transform failed: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "transform_assets: %s", ex.what());
    }
}

}  // namespace map_asset_manager

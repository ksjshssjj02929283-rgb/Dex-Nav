#include "map_asset_manager/map_asset_manager_node.hpp"
#include "map_asset_manager/asset_codec.hpp"
#include "map_asset_manager/asset_editor.hpp"
#include "map_asset_manager/asset_transform.hpp"
#include <unordered_set>

#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace map_asset_manager {


MapAssetManagerNode::MapAssetManagerNode()
    // 节点名 = map_asset_manager（与 launch name= 及所有消费方 /map_asset_manager/ 一致；
    // 裸 ros2 run 也能对上服务名，避免 /map_asset_manager_node/ 与消费方 mismatch）
    : rclcpp::Node("map_asset_manager") {
    declare_parameter("auto_save", true);
    auto_save_ = get_parameter("auto_save").as_bool();
    // 资产到障碍 / 未知 / 禁行区 / 地图边界的最小距离（m），取机器人半宽
    geometry_params_.clearance_m = declare_parameter("asset_clearance_m", 0.35);

    initialize();

    // 同名实例保护：后台残留的旧进程会和新进程同时响应同一个服务，一次写入被执行多遍、
    // 各自内存不一致。启动 2 s 后查一次图，已有同名节点则本进程退出（退出的总是后来者）
    single_instance_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        single_instance_timer_->cancel();
        const std::string me = get_fully_qualified_name();
        const auto names = get_node_names();
        const auto count = std::count(names.begin(), names.end(), me);
        if (count > 1) {
            RCLCPP_FATAL(get_logger(),
                         "Another instance of node '%s' is already running (%ld found); exiting so that "
                         "requests are not executed twice. Stop the old instance first.",
                         me.c_str(), static_cast<long>(count));
            rclcpp::shutdown();
        }
    });

    RCLCPP_INFO(get_logger(),
                "MapAssetManagerNode constructed, waiting for map_manager/map_state "
                "(fallback: legacy map_manager/map_folder + map_manager/map_name)...");
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
            }
            // 持久化 related_* 字段（SpatialIndexer 重算结果写回 JSON）
            saveAllAssets();
        },
        /*on_rooms_changed=*/ [this]() {
            if (room_mgr_) {
                room_mgr_->publishMarkers();
            }
            saveAllAssets();
        });

    // ── 通用服务 & 发布器 ──────────────────────────────────────────
    reload_data_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reload_data",
        std::bind(&MapAssetManagerNode::reloadDataCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    save_assets_srv_ = create_service<nav_interfaces::srv::SaveAssets>(
        "~/save_assets",
        std::bind(&MapAssetManagerNode::saveAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // X-Wiz 批量接口（M4-3 §3.3）：整体读 / 整体提交
    get_assets_srv_ = create_service<nav_interfaces::srv::GetAssets>(
        "~/get_assets",
        std::bind(&MapAssetManagerNode::getAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    set_assets_srv_ = create_service<nav_interfaces::srv::SetAssets>(
        "~/set_assets",
        std::bind(&MapAssetManagerNode::setAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    rollback_assets_srv_ = create_service<nav_interfaces::srv::RollbackAssets>(
        "~/rollback_assets",
        std::bind(&MapAssetManagerNode::rollbackAssetsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    area_filter_ = std::make_unique<AreaFilterPublisher>(this);
    // 变更事件：事件语义，不 latched；晚到的订阅者先 get_assets 取快照即可
    changes_pub_ = create_publisher<nav_interfaces::msg::AssetChange>("~/asset_changes", rclcpp::QoS(10).reliable());
    // state / assets 均 transient_local：后启动订阅者自动收到最近一次（latched）
    assets_pub_ = create_publisher<nav_interfaces::msg::AssetSnapshot>(
        "~/assets", rclcpp::QoS(1).transient_local());

    // 注：markers 与 status 均改为事件驱动（数据变化时由 saveAllAssets / loadData 触发），
    // 不再周期重发；配合各 publisher 的 transient_local QoS，rviz 重启也能收到最新。

    // ── 订阅 map_manager/map_state（latched，原子 folder+name+epoch）──
    auto qos = rclcpp::QoS(1).transient_local().reliable();

    map_state_sub_ = create_subscription<nav_interfaces::msg::MapState>(
        "map_manager/map_state", qos,
        std::bind(&MapAssetManagerNode::onMapState, this, std::placeholders::_1));

    // ── EventBus 订阅 ──────────────────────────────────────────────
    registerEventHandlers();
    // 启动即发一次 state（loaded=false）：latched 语义下晚订阅者总能拿到状态，不必等首次加载
    publishStatus();
}

// ──────────────────────────────────────────────────────────────────────
// onMapState — 按 epoch 比对触发重载（不依赖名字）
//   epoch 变即重载：切换地图 / 同名重载 / 回退 都被确定性感知，根治同名漏检。
//   folder+name 随消息原子到达，消除旧 folder/name 两 topic 的时序竞争。
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::onMapState(const nav_interfaces::msg::MapState::SharedPtr msg) {
    if (msg->map_folder.empty() || msg->map_name.empty()) {
        RCLCPP_WARN(get_logger(), "Received MapState with empty folder/name, ignoring");
        return;
    }
    // epoch 未变 → 无需重载（首条 epoch>=1，与初始 last_epoch_=0 不同即触发首次加载）
    if (msg->epoch == last_epoch_) return;

    RCLCPP_INFO(get_logger(),
                "MapState: epoch %lu -> %lu, map '%s' -> '%s' (folder '%s'), reloading assets",
                static_cast<unsigned long>(last_epoch_),
                static_cast<unsigned long>(msg->epoch),
                current_map_name_.c_str(), msg->map_name.c_str(), msg->map_folder.c_str());

    last_epoch_ = msg->epoch;
    map_base_folder_ = msg->map_folder;
    current_map_name_ = msg->map_name;

    switchToMap(current_map_name_);
}


// ──────────────────────────────────────────────────────────────────────
// resolveAssetsFile — 布局自适应
//   新机器：新 map_manager 在 LoadMap 前把地图迁到 <map>/slam + <map>/assets（DEC-007）
//   旧机器：老 map_manager 无迁移，底图与资产都在 <map>/ 根下（扁平）
// ──────────────────────────────────────────────────────────────────────

std::string MapAssetManagerNode::resolveAssetsFile(const std::string& map_name) const {
    return ResolveAssetsFile((fs::path(map_base_folder_) / map_name).string());
}

// ──────────────────────────────────────────────────────────────────────
// switchToMap
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::switchToMap(const std::string& map_name) {
    // 分层布局（DEC-007）：资产在 <map>/assets/ 子目录（与底图 <map>/slam/ 物理隔离）；
    // 旧机器扁平布局自动退回 <map>/map_assets.json
    assets_file_ = resolveAssetsFile(map_name);

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
    // 记下文件上的内容指纹：之后自动落盘只在内存与它不同时才写
    file_revision_ = ComputeAssetsRevision(currentSet());

    RCLCPP_INFO(get_logger(),
                "Loaded from %s: %zu points, %zu routes, %zu areas, %zu rooms, %zu junctions, %zu edges",
                assets_file_.c_str(), points.size(), routes.size(), areas.size(), rooms.size(),
                junctions.size(), edges.size());

    point_mgr_->publishMarkers();
    area_mgr_->publishMarkers();
    room_mgr_->publishMarkers();
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
    // 内容与文件上的一致就不写：加载地图后的空间归属重算、重复的保存触发都不会改写文件，
    // 资产的 last_edited_at 只在资产真的变了时才推进
    const AssetSet set = currentSet();
    if (ComputeAssetsRevision(set) == file_revision_) return;
    persistCurrent();
}

bool MapAssetManagerNode::persistCurrent() {
    const AssetSet set = currentSet();
    if (!SaveAssetSet(*file_manager_, assets_file_, set)) return false;
    file_revision_ = ComputeAssetsRevision(set);
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// initializeActionServers（构造完成后调用）
// ──────────────────────────────────────────────────────────────────────

void MapAssetManagerNode::initializeActionServers() {
    // 导航执行已归 navigation 门面（DEC-041）：资产管理器不再提供 navigate_to_point / navigate_route / get_status。
    RCLCPP_INFO(get_logger(), "MapAssetManagerNode: navigation actions are served by the navigation facade (/navigation/*)");
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
            res->message = "No map loaded yet (waiting for map_manager/map_state)";
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
    // 单一话题：资产快照即状态（loaded / error_code / 六类数组），latched；
    // 任何一次资产变更（含 RViz 侧单条编辑）经 saveAllAssets → 这里推送
    auto snap = buildSnapshot();
    snap.loaded     = !current_map_name_.empty();
    snap.error_code = last_error_code_;
    if (snap.loaded) {
        const AssetSet set = currentSet();
        snap.issues = issuesFor((fs::path(map_base_folder_) / current_map_name_).string(), set);
        // 禁行区 / 虚拟墙 / 减速区掩膜：内容不变时内部去重，不会重复触发导航的 filter 更新
        std::vector<Area> areas;
        for (const auto& kv : set.areas) areas.push_back(kv.second);
        if (area_filter_) area_filter_->setAreas(areas);
    }
    assets_pub_->publish(snap);
}

nav_interfaces::msg::AssetSnapshot MapAssetManagerNode::buildSnapshot() const {
    nav_interfaces::msg::AssetSnapshot snap;
    snap.header.stamp = now();
    snap.map_name     = current_map_name_;
    snap.map_epoch    = last_epoch_;
    for (const auto& kv : point_store_->getPointsMap()) snap.points.push_back(toMsg(kv.second));
    for (const auto& kv : area_store_->getAreasMap())   snap.areas.push_back(AreaManager::toMsg(kv.second));
    for (const auto& kv : room_store_->getRoomsMap())   snap.rooms.push_back(RoomManager::toMsg(kv.second));
    for (const auto& kv : route_store_->getRoutesMap()) snap.routes.push_back(toMsg(kv.second));
    for (const auto& j  : junction_store_->getList())   snap.junctions.push_back(toMsg(j));
    for (const auto& e  : edge_store_->getAll())        snap.edges.push_back(toMsg(e));
    return snap;
}

void MapAssetManagerNode::refreshAllOutputs() {
    point_mgr_->publishMarkers();
    area_mgr_->publishMarkers();
    room_mgr_->publishMarkers();
    route_mgr_->publishAllRouteMarkers();
    route_graph_mgr_->publishMarkers();
    publishStatus();
}

// ──────────────────────────────────────────────────────────────────────
// 按图寻址：map_name 空或等于当前图 → 当前地图（内存工作集，发话题）；
// 其它 → 非当前地图（请求内从文件装载、改完原子写回，不碰内存、不发话题）
// ──────────────────────────────────────────────────────────────────────

bool MapAssetManagerNode::resolveTarget(const std::string& map_name, Target& t, int32_t& code,
                                        std::string& msg) const {
    using ST = nav_interfaces::msg::AssetSnapshot;
    t = Target{};
    t.is_current = map_name.empty() || map_name == current_map_name_;
    if (t.is_current) {
        if (current_map_name_.empty() || assets_file_.empty()) {
            code = ST::E_ASSET_NO_MAP;
            msg = "no map loaded yet (waiting for map_manager/map_state)";
            return false;
        }
        t.map_name = current_map_name_;
        t.map_dir = (fs::path(map_base_folder_) / current_map_name_).string();
        t.assets_file = assets_file_;
        return true;
    }
    if (map_name.find('/') != std::string::npos || map_name == "." || map_name == ".." || map_name[0] == '.') {
        code = ST::E_ASSET_MAP_NOT_FOUND;
        msg = "invalid map name '" + map_name + "'";
        return false;
    }
    std::string root = map_base_folder_;
    if (root.empty()) {
        const char* home = std::getenv("HOME");
        if (home != nullptr) root = (fs::path(home) / ".dexe" / "data" / "maps").string();
    }
    std::error_code ec;
    if (root.empty() || !fs::is_directory(fs::path(root) / map_name, ec)) {
        code = ST::E_ASSET_MAP_NOT_FOUND;
        msg = "map '" + map_name + "' not found";
        return false;
    }
    t.map_name = map_name;
    t.map_dir = (fs::path(root) / map_name).string();
    t.assets_file = ResolveAssetsFile(t.map_dir);
    return true;
}

AssetSet MapAssetManagerNode::currentSet() const {
    AssetSet s;
    s.points = point_store_->getPointsMap();
    s.routes = route_store_->getRoutesMap();
    s.areas = area_store_->getAreasMap();
    s.rooms = room_store_->getRoomsMap();
    for (const auto& j : junction_store_->getList()) s.junctions[j.id] = j;
    for (const auto& e : edge_store_->getAll()) s.edges[e.id] = e;
    return s;
}

std::shared_ptr<const GridMap> MapAssetManagerNode::gridFor(const std::string& map_dir) {
    std::string err;
    auto g = grid_cache_.get(map_dir, err);
    if (!g) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "no base map for geometry checks in %s (%s); only restricted areas are checked",
                             map_dir.c_str(), err.c_str());
    }
    return g;
}

std::vector<nav_interfaces::msg::AssetIssue> MapAssetManagerNode::issuesFor(const std::string& map_dir,
                                                                           const AssetSet& set) {
    const GeometryChecker checker(gridFor(map_dir), set, geometry_params_);
    return checker.computeIssues(set);
}

std::vector<nav_interfaces::msg::MapVersion> MapAssetManagerNode::versionsFor(const std::string& assets_file) const {
    std::vector<nav_interfaces::msg::MapVersion> out;
    for (const auto& v : ListAssetVersions(assets_file)) {
        nav_interfaces::msg::MapVersion m;
        m.version_ts = v.version_ts;
        m.saved_at.sec = static_cast<int32_t>(v.saved_at_sec);
        m.source = v.source;
        m.complete = true;
        out.push_back(std::move(m));
    }
    return out;
}

void MapAssetManagerNode::applyToStores(const AssetSet& set) {
    point_store_->setPoints(set.points);
    route_store_->setRoutes(set.routes);
    area_store_->setAreas(set.areas);
    room_store_->setRooms(set.rooms);
    junction_store_->set(set.junctionList());
    edge_store_->set(set.edgeList());
}

void MapAssetManagerNode::reloadCurrentFromFile() {
    point_store_->clear();
    route_store_->clear();
    area_store_->clear();
    room_store_->clear();
    junction_store_->clear();
    edge_store_->clear();
    loadData();
    event_bus_->publish({AssetEventType::DATA_RELOADED, "", ""});
}

// 资产变更事件：任意地图的资产变了就广播一条（只带地图名与新指纹），前端据此决定是否重新 get_assets
void MapAssetManagerNode::publishChange(const std::string& map_name, bool is_current, const std::string& op,
                                        const std::string& version_ts, std::string revision) {
    if (!changes_pub_) return;
    if (revision.empty()) {
        AssetSet set;
        Target t;
        int32_t code = 0;
        std::string msg;
        if (is_current) set = currentSet();
        else if (resolveTarget(map_name, t, code, msg)) LoadAssetSet(*file_manager_, t.assets_file, set);
        revision = ComputeAssetsRevision(set);
    }
    nav_interfaces::msg::AssetChange m;
    m.stamp = now();
    m.map_name = map_name;
    m.is_current = is_current;
    m.revision = revision;
    m.op = op;
    m.version_ts = version_ts;
    changes_pub_->publish(m);
}

void MapAssetManagerNode::getAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::GetAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetAssets::Response> res) {
    using ST = nav_interfaces::msg::AssetSnapshot;
    using GA = nav_interfaces::srv::GetAssets;
    Target t;
    if (!resolveTarget(req->map_name, t, res->error_code, res->message)) {
        res->success = false;
        return;
    }
    AssetSet set;
    if (t.is_current) {
        set = currentSet();
    } else if (!LoadAssetSet(*file_manager_, t.assets_file, set)) {
        res->success = false;
        res->error_code = ST::E_ASSET_SAVE_FAILED;
        res->message = "failed to read assets file " + t.assets_file;
        return;
    }
    // kinds 位掩码：0 视为全部；只装配被请求的类型，避免运行时消费方为查一类而拉全量
    const uint8_t k = (req->kinds == 0) ? GA::Request::KIND_ALL : req->kinds;
    res->map_name = t.map_name;
    res->is_current = t.is_current;
    res->map_epoch = t.is_current ? last_epoch_ : 0;
    res->revision = ComputeAssetsRevision(set);
    res->last_edited_at.sec = static_cast<int32_t>(AssetsLastEditedAt(t.assets_file));
    res->last_edited_at.nanosec = 0;
    res->issues = issuesFor(t.map_dir, set);
    res->versions = versionsFor(t.assets_file);
    if (k & GA::Request::KIND_POINTS)    for (const auto& kv : set.points) res->points.push_back(toMsg(kv.second));
    if (k & GA::Request::KIND_AREAS)     for (const auto& kv : set.areas)  res->areas.push_back(AreaManager::toMsg(kv.second));
    if (k & GA::Request::KIND_ROOMS)     for (const auto& kv : set.rooms)  res->rooms.push_back(RoomManager::toMsg(kv.second));
    if (k & GA::Request::KIND_ROUTES)    for (const auto& kv : set.routes) res->routes.push_back(toMsg(kv.second));
    if (k & GA::Request::KIND_JUNCTIONS) for (const auto& j : set.junctionList()) res->junctions.push_back(toMsg(j));
    if (k & GA::Request::KIND_EDGES)     for (const auto& e : set.edgeList())     res->edges.push_back(toMsg(e));
    res->success = true;
    res->error_code = ST::E_NONE;
    res->message = "ok (kinds=" + std::to_string(k) + (t.is_current ? ", current map)" : ", map '" + t.map_name + "')");
}

void MapAssetManagerNode::setAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::SetAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::SetAssets::Response> res) {
    using ST = nav_interfaces::msg::AssetSnapshot;
    Target t;
    auto fail = [&](int32_t code, const std::string& msg) {
        res->success = false;
        res->error_code = code;
        res->message = msg;
        if (t.is_current) {
            last_error_code_ = code;
            publishStatus();
        }
        RCLCPP_ERROR(get_logger(), "set_assets rejected (%d): %s%s", code, msg.c_str(),
                     res->rejected.empty() ? "" : (" [" + std::to_string(res->rejected.size()) + " items]").c_str());
    };
    {
        int32_t code = 0;
        std::string msg;
        if (!resolveTarget(req->map_name, t, code, msg)) { fail(code, msg); return; }
    }

    // 非当前图：读改写全程持锁，写前用文件内容比对指纹
    std::unique_ptr<AssetFileLock> lock;
    AssetSet set;
    if (t.is_current) {
        // expected_epoch = 0 表示不校验（本地编辑器用）
        if (req->expected_epoch != 0 && req->expected_epoch != last_epoch_) {
            fail(ST::E_ASSET_EPOCH_MISMATCH, "expected_epoch " + std::to_string(req->expected_epoch) +
                 " != current map epoch " + std::to_string(last_epoch_) + "; map changed during editing, re-fetch get_assets");
            return;
        }
        set = currentSet();
    } else {
        lock = std::make_unique<AssetFileLock>(t.assets_file);
        if (!lock->locked()) { fail(ST::E_ASSET_SAVE_FAILED, "failed to lock " + t.assets_file); return; }
        if (!LoadAssetSet(*file_manager_, t.assets_file, set)) { fail(ST::E_ASSET_SAVE_FAILED, "failed to read " + t.assets_file); return; }
    }
    if (!req->expected_revision.empty()) {
        const std::string rev = ComputeAssetsRevision(set);
        if (rev != req->expected_revision) {
            fail(ST::E_ASSET_REVISION_MISMATCH, "expected_revision " + req->expected_revision + " != " + rev +
                 "; assets changed by another writer, re-fetch get_assets");
            return;
        }
    }

    ApplyOutcome r = ApplySetAssets(*req, set, now(), &geometry_params_, gridFor(t.map_dir));
    res->rejected = r.rejected;
    if (!r.ok) { fail(r.error_code, r.message); return; }

    res->points_count    = static_cast<uint32_t>(set.points.size());
    res->areas_count     = static_cast<uint32_t>(set.areas.size());
    res->rooms_count     = static_cast<uint32_t>(set.rooms.size());
    res->routes_count    = static_cast<uint32_t>(set.routes.size());
    res->junctions_count = static_cast<uint32_t>(set.junctions.size());
    res->edges_count     = static_cast<uint32_t>(set.edges.size());

    std::string saved;
    if (t.is_current) {
        applyToStores(set);
        // 批量路径不经事件总线：空间归属显式重算，经 on_*_changed 钩子发布并按 auto_save 落盘
        if (spatial_indexer_) spatial_indexer_->rebuildAll();
        last_error_code_ = ST::E_NONE;
        if (req->save) {
            // 从 store 取（含索引重算后的 related_*）
            if (!persistCurrent()) {
                last_error_code_ = ST::E_ASSET_SAVE_FAILED;
                refreshAllOutputs();
                res->success = false;
                res->error_code = last_error_code_;
                res->message = "applied in memory but save failed: " + assets_file_;
                return;
            }
            const std::string ts = CreateAssetVersion(assets_file_, get_logger(), AssetVersionSource::SAVE);
            saved = " saved" + (ts.empty() ? std::string("") : " (version " + ts + ")");
        }
        refreshAllOutputs();
        set = currentSet();
    } else {
        // 非当前图没有内存副本：每次都落盘；save=true 再存一份版本
        if (!SaveAssetSet(*file_manager_, t.assets_file, set)) {
            fail(ST::E_ASSET_SAVE_FAILED, "failed to write " + t.assets_file + "; nothing applied");
            return;
        }
        saved = " written";
        if (req->save) {
            const std::string ts = CreateAssetVersion(t.assets_file, get_logger(), AssetVersionSource::SAVE);
            saved += ts.empty() ? std::string("") : " (version " + ts + ")";
        }
    }
    res->revision = ComputeAssetsRevision(set);
    res->issues = issuesFor(t.map_dir, set);
    res->success = true;
    res->error_code = ST::E_NONE;
    res->message = r.message + (t.is_current ? "" : " [map '" + t.map_name + "']") + saved;
    publishChange(t.map_name, t.is_current, "set", "", res->revision);
    RCLCPP_INFO(get_logger(), "set_assets %s", res->message.c_str());
}

void MapAssetManagerNode::saveAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::SaveAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::SaveAssets::Response> res) {
    Target t;
    int32_t code = 0;
    if (!resolveTarget(req->map_name, t, code, res->message)) {
        res->success = false;
        return;
    }
    try {
        if (t.is_current) {
            // 显式保存不受 auto_save 开关影响
            if (!persistCurrent()) {
                res->success = false;
                res->message = "Save failed: " + assets_file_;
                return;
            }
            publishStatus();
        }
        const AssetFileLock lock(t.assets_file);
        std::error_code ec;
        if (!fs::exists(t.assets_file, ec)) {
            res->success = true;
            res->assets_path = t.assets_file;
            res->message = "map '" + t.map_name + "' has no assets file; nothing to save";
            return;
        }
        // 显式保存产一份资产版本，供版本列表与回滚
        res->backup_ts = CreateAssetVersion(t.assets_file, get_logger(), AssetVersionSource::SAVE);
        res->success = true;
        res->assets_path = t.assets_file;
        res->message = "Assets saved" + (res->backup_ts.empty() ? std::string("") : " (version " + res->backup_ts + ")");
        RCLCPP_INFO(get_logger(), "Assets saved via service: %s%s", t.assets_file.c_str(),
                    res->backup_ts.empty() ? "" : (" [version " + res->backup_ts + "]").c_str());
        publishChange(t.map_name, t.is_current, "save", res->backup_ts);
    } catch (const std::exception& ex) {
        res->success = false;
        res->message = std::string("Save failed: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "saveAssets: %s", ex.what());
    }
}

// rollback_assets：任意地图的资产回滚到某个版本。回滚前把当前资产存成一份 rollback 版本（可撤销）；
// 当前地图同时重载内存、重新发布话题与禁行区掩膜，不需要重新加载地图
void MapAssetManagerNode::rollbackAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::RollbackAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::RollbackAssets::Response> res) {
    using ST = nav_interfaces::msg::AssetSnapshot;
    Target t;
    auto fail = [&](int32_t code, const std::string& msg) {
        res->success = false;
        res->error_code = code;
        res->message = msg;
        RCLCPP_ERROR(get_logger(), "rollback_assets failed (%d): %s", code, msg.c_str());
    };
    {
        int32_t code = 0;
        std::string msg;
        if (!resolveTarget(req->map_name, t, code, msg)) { fail(code, msg); return; }
    }
    res->is_current = t.is_current;
    if (t.is_current && !persistCurrent()) {
        fail(ST::E_ASSET_SAVE_FAILED, "failed to persist current assets before rollback");
        return;
    }
    const AssetFileLock lock(t.assets_file);
    if (!lock.locked()) { fail(ST::E_ASSET_SAVE_FAILED, "failed to lock " + t.assets_file); return; }

    std::string ts = req->version_ts;
    if (ts.empty()) {
        const auto versions = ListAssetVersions(t.assets_file);
        if (versions.empty()) { fail(ST::E_ASSET_VERSION_NOT_FOUND, "map '" + t.map_name + "' has no asset versions"); return; }
        ts = versions.front().version_ts;
    }
    const std::string src = AssetVersionFile(t.assets_file, ts);
    std::error_code ec;
    if (src.empty() || !fs::is_regular_file(src, ec)) {
        fail(ST::E_ASSET_VERSION_NOT_FOUND, "asset version '" + ts + "' not found for map '" + t.map_name + "'");
        return;
    }
    AssetSet probe;
    if (!LoadAssetSet(*file_manager_, src, probe)) { fail(ST::E_ASSET_VERSION_NOT_FOUND, "asset version '" + ts + "' is unreadable"); return; }
    if (!req->expected_revision.empty()) {
        AssetSet cur;
        if (!LoadAssetSet(*file_manager_, t.assets_file, cur)) { fail(ST::E_ASSET_SAVE_FAILED, "failed to read " + t.assets_file); return; }
        if (ComputeAssetsRevision(cur) != req->expected_revision) {
            fail(ST::E_ASSET_REVISION_MISMATCH, "assets changed by another writer, re-fetch get_assets");
            return;
        }
    }

    res->backup_ts = CreateAssetVersion(t.assets_file, get_logger(), AssetVersionSource::ROLLBACK,
                                        kAssetVersionKeepCount, ts);
    // 按内容写回（不整文件拷贝）：回滚也是一次资产编辑，last_edited_at 记为现在而不是版本里的旧值
    if (!SaveAssetSet(*file_manager_, t.assets_file, probe)) {
        fail(ST::E_ASSET_SAVE_FAILED, "failed to restore " + t.assets_file);
        return;
    }
    if (t.is_current) reloadCurrentFromFile();

    AssetSet after;
    if (t.is_current) after = currentSet();
    else LoadAssetSet(*file_manager_, t.assets_file, after);
    res->revision = ComputeAssetsRevision(after);
    res->version_ts = ts;
    res->success = true;
    res->error_code = ST::E_NONE;
    res->message = "assets of map '" + t.map_name + "' rolled back to " + ts +
                   (res->backup_ts.empty() ? std::string("") : "; previous state saved as " + res->backup_ts) +
                   (t.is_current ? "; reloaded" : "");
    RCLCPP_INFO(get_logger(), "rollback_assets: %s", res->message.c_str());
    publishChange(t.map_name, t.is_current, "rollback", ts, res->revision);
}

}  // namespace map_asset_manager

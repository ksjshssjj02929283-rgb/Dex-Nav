#include "map_asset_manager/map_asset_manager_node.hpp"
#include "map_asset_manager/asset_codec.hpp"
#include <unordered_set>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace map_asset_manager {

namespace {
// 资产 ring 版本备份（DEC-006/007 Model C）：把当前资产文件复制成带时间戳的版本，
// 轮换保留最近 keep 个。返回版本时间戳（失败返回空）。
// 供 command_board 统一版本 manifest 引用 / 完整回滚（B 阶段消费）。
// 仅显式 save_assets（手动 / command_board 触发）调用；auto_save（编辑触发）不产版本，避免 churn。
std::string MakeAssetRingBackup(const std::string& src_file,
                                const rclcpp::Logger& logger, int keep = 50) {
  std::error_code ec;
  if (!fs::exists(src_file, ec)) return "";
  const fs::path backups_dir = fs::path(src_file).parent_path() / "backups";
  fs::create_directories(backups_dir, ec);

  std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm_buf;
  localtime_r(&t, &tm_buf);
  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d-%H%M%S", &tm_buf);

  const fs::path dst = backups_dir / (std::string("map_assets.") + ts + ".json");
  fs::copy_file(src_file, dst, fs::copy_options::overwrite_existing, ec);
  if (ec) {
    RCLCPP_WARN(logger, "asset ring backup failed: %s", ec.message().c_str());
    return "";
  }

  // 轮换：文件名含时间戳，字典序即时间序，保留最近 keep 个
  std::vector<fs::path> versions;
  for (const auto& e : fs::directory_iterator(backups_dir, ec)) {
    const std::string name = e.path().filename().string();
    if (name.rfind("map_assets.", 0) == 0 && e.path().extension() == ".json")
      versions.push_back(e.path());
  }
  std::sort(versions.begin(), versions.end());
  for (size_t i = 0; i + static_cast<size_t>(keep) < versions.size(); ++i)
    fs::remove(versions[i], ec);

  return ts;
}
}  // namespace

MapAssetManagerNode::MapAssetManagerNode()
    // 节点名 = map_asset_manager（与 launch name= 及所有消费方 /map_asset_manager/ 一致；
    // 裸 ros2 run 也能对上服务名，避免 /map_asset_manager_node/ 与消费方 mismatch）
    : rclcpp::Node("map_asset_manager") {
    declare_parameter("auto_save", true);
    auto_save_ = get_parameter("auto_save").as_bool();

    initialize();

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

    transform_assets_srv_ = create_service<nav_interfaces::srv::TransformAssets>(
        "~/transform_assets",
        std::bind(&MapAssetManagerNode::transformAssetsCallback, this,
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
    const fs::path map_dir = fs::path(map_base_folder_) / map_name;
    std::error_code ec;

    const bool legacy_layout = !fs::exists(map_dir / "slam", ec) &&
                               (fs::exists(map_dir / "map_assets.json", ec) ||
                                fs::exists(map_dir / "map.yaml", ec));

    return legacy_layout ? (map_dir / "map_assets.json").string()
                         : (map_dir / "assets" / "map_assets.json").string();
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

void MapAssetManagerNode::getAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::GetAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetAssets::Response> res) {
    using ST = nav_interfaces::msg::AssetSnapshot;
    if (current_map_name_.empty()) {
        res->success = false;
        res->error_code = ST::E_ASSET_NO_MAP;
        res->message = "no map loaded yet (waiting for map_manager/map_state)";
        return;
    }
    using GA = nav_interfaces::srv::GetAssets;
    // kinds 位掩码：0 视为全部；只装配被请求的类型，避免运行时消费方为查一类而拉全量
    const uint8_t k = (req->kinds == 0) ? GA::Request::KIND_ALL : req->kinds;
    res->map_name  = current_map_name_;
    res->map_epoch = last_epoch_;
    if (k & GA::Request::KIND_POINTS)    for (const auto& kv : point_store_->getPointsMap()) res->points.push_back(toMsg(kv.second));
    if (k & GA::Request::KIND_AREAS)     for (const auto& kv : area_store_->getAreasMap())   res->areas.push_back(AreaManager::toMsg(kv.second));
    if (k & GA::Request::KIND_ROOMS)     for (const auto& kv : room_store_->getRoomsMap())   res->rooms.push_back(RoomManager::toMsg(kv.second));
    if (k & GA::Request::KIND_ROUTES)    for (const auto& kv : route_store_->getRoutesMap()) res->routes.push_back(toMsg(kv.second));
    if (k & GA::Request::KIND_JUNCTIONS) for (const auto& j  : junction_store_->getList())   res->junctions.push_back(toMsg(j));
    if (k & GA::Request::KIND_EDGES)     for (const auto& e  : edge_store_->getAll())        res->edges.push_back(toMsg(e));
    res->success = true;
    res->error_code = ST::E_NONE;
    res->message = "ok (kinds=" + std::to_string(k) + ")";
}

void MapAssetManagerNode::setAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::SetAssets::Request> req,
    std::shared_ptr<nav_interfaces::srv::SetAssets::Response> res) {
    using SA = nav_interfaces::srv::SetAssets;
    using ST = nav_interfaces::msg::AssetSnapshot;
    auto fail = [&](int32_t code, const std::string& msg) {
        res->success = false; res->error_code = code; res->message = msg;
        last_error_code_ = code;
        RCLCPP_ERROR(get_logger(), "set_assets rejected (%d): %s%s", code, msg.c_str(),
                     res->rejected.empty() ? "" : (" [" + std::to_string(res->rejected.size()) + " items]").c_str());
        publishStatus();
    };
    // ── 前置校验 ──
    if (req->mode > SA::Request::MODE_REMOVE) {
        fail(ST::E_ASSET_MODE_INVALID, "invalid mode " + std::to_string(req->mode) + "; expected 0 REPLACE / 1 MERGE / 2 ADD_ONLY / 3 UPDATE_ONLY / 4 REMOVE"); return;
    }
    if (current_map_name_.empty()) { fail(ST::E_ASSET_NO_MAP, "no map loaded yet"); return; }
    // expected_epoch = 0 表示不校验（本地编辑器用）；X-Wiz 必须传 get_assets 返回的 map_epoch
    if (req->expected_epoch != 0 && req->expected_epoch != last_epoch_) {
        fail(ST::E_ASSET_EPOCH_MISMATCH, "expected_epoch " + std::to_string(req->expected_epoch) +
             " != current map epoch " + std::to_string(last_epoch_) + "; map changed during editing, re-fetch get_assets");
        return;
    }
    // ── 组装目标集合 ──
    // REPLACE：kinds 列出的类型从空开始（请求缺席即删），未列出的类型保持当前不变；kinds=0 视为全部。
    //          与 GetAssets.kinds 对称——按类型读、编辑、按同类型 REPLACE 提交，不会误清其它类型。
    // MERGE / ADD_ONLY / UPDATE_ONLY / REMOVE：六类都从当前开始，按 name/id 逐项处理（原六个单条 set_* 接口的语义）。
    const bool replace     = (req->mode == SA::Request::MODE_REPLACE);
    const bool add_only    = (req->mode == SA::Request::MODE_ADD_ONLY);
    const bool update_only = (req->mode == SA::Request::MODE_UPDATE_ONLY);
    const bool remove      = (req->mode == SA::Request::MODE_REMOVE);
    const uint8_t k = (req->kinds == 0) ? SA::Request::KIND_ALL : req->kinds;
    auto keep = [&](uint8_t bit) { return !replace || !(k & bit); };
    std::unordered_map<std::string, NavPoint> points = keep(SA::Request::KIND_POINTS) ? point_store_->getPointsMap() : std::unordered_map<std::string, NavPoint>{};
    std::unordered_map<std::string, Route>    routes = keep(SA::Request::KIND_ROUTES) ? route_store_->getRoutesMap() : std::unordered_map<std::string, Route>{};
    std::unordered_map<std::string, Area>     areas  = keep(SA::Request::KIND_AREAS)  ? area_store_->getAreasMap()   : std::unordered_map<std::string, Area>{};
    std::unordered_map<std::string, Room>     rooms  = keep(SA::Request::KIND_ROOMS)  ? room_store_->getRoomsMap()   : std::unordered_map<std::string, Room>{};
    std::unordered_map<std::string, RouteJunction> jcts;
    std::unordered_map<std::string, RouteEdge>     edges;
    if (keep(SA::Request::KIND_JUNCTIONS)) for (const auto& j : junction_store_->getList()) jcts[j.id] = j;
    if (keep(SA::Request::KIND_EDGES))     for (const auto& e : edge_store_->getAll())      edges[e.id] = e;
    std::vector<std::string>& rej = res->rejected;
    bool guard_hit = false;
    // 单条模式守卫：ADD_ONLY 已存在 → 1508；UPDATE_ONLY / REMOVE 不存在 → 1507（按当前集合查主键）
    auto guard = [&](const std::string& kind, const std::string& key, bool exists) -> bool {
        if (add_only && exists)                  { rej.push_back(kind + ":" + key + ":exists");    guard_hit = true; return false; }
        if ((update_only || remove) && !exists)  { rej.push_back(kind + ":" + key + ":not_found"); guard_hit = true; return false; }
        return true;
    };
    if (remove) {
        std::unordered_set<std::string> rm_pts, rm_jcts;
        for (const auto& m : req->points)    { if (m.name.empty()) { rej.push_back("point::name is empty"); continue; } if (guard("point", m.name, points.count(m.name) > 0))    { points.erase(m.name); rm_pts.insert(m.name); } }
        for (const auto& m : req->areas)     { if (m.name.empty()) { rej.push_back("area::name is empty"); continue; }  if (guard("area", m.name, areas.count(m.name) > 0))      areas.erase(m.name); }
        for (const auto& m : req->rooms)     { if (m.name.empty()) { rej.push_back("room::name is empty"); continue; }  if (guard("room", m.name, rooms.count(m.name) > 0))      rooms.erase(m.name); }
        for (const auto& m : req->routes)    { if (m.name.empty()) { rej.push_back("route::name is empty"); continue; } if (guard("route", m.name, routes.count(m.name) > 0))    routes.erase(m.name); }
        for (const auto& m : req->junctions) { if (m.id.empty())   { rej.push_back("junction::id is empty"); continue; } if (guard("junction", m.id, jcts.count(m.id) > 0))    { jcts.erase(m.id); rm_jcts.insert(m.id); } }
        for (const auto& m : req->edges)     { if (m.id.empty())   { rej.push_back("edge::id is empty"); continue; }   if (guard("edge", m.id, edges.count(m.id) > 0))         edges.erase(m.id); }
        if (!rej.empty()) { fail(guard_hit ? ST::E_ASSET_NOT_FOUND : ST::E_ASSET_VALIDATION, "remove rejected; nothing applied"); return; }
        // 级联（与原单条路径一致）：删点 → 路线剔点、清引用它的边、区域/房间关联剔除；删节点 → 清引用它的边
        auto ep_removed = [&](const RouteEndpoint& ep) { return (ep.type == "nav" && rm_pts.count(ep.id) > 0) || (ep.type == "junction" && rm_jcts.count(ep.id) > 0); };
        for (auto it = edges.begin(); it != edges.end();) { if (ep_removed(it->second.from) || ep_removed(it->second.to)) it = edges.erase(it); else ++it; }
        auto strip = [](std::vector<std::string>& v, const std::unordered_set<std::string>& gone) {
            v.erase(std::remove_if(v.begin(), v.end(), [&](const std::string& n) { return gone.count(n) > 0; }), v.end()); };
        for (auto& kv : routes) strip(kv.second.point_names, rm_pts);
        for (auto& kv : areas)  { strip(kv.second.related_nav_point_names, rm_pts); strip(kv.second.manual_linked_point_names, rm_pts); strip(kv.second.related_junction_ids, rm_jcts); }
        for (auto& kv : rooms)  { strip(kv.second.related_nav_point_names, rm_pts); strip(kv.second.manual_anchor_point_names, rm_pts); strip(kv.second.related_junction_ids, rm_jcts); }
    } else {
        // 字段校验（同批内重复也算失败）+ ADD_ONLY / UPDATE_ONLY 守卫
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->points) {
                if (m.name.empty()) { rej.push_back("point::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("point:" + m.name + ":duplicate in request"); continue; }
                if (!guard("point", m.name, points.count(m.name) > 0)) continue;
                points[m.name] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->areas) {
                if (m.name.empty()) { rej.push_back("area::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("area:" + m.name + ":duplicate in request"); continue; }
                if (m.type < nav_interfaces::msg::AreaInfo::VIRTUAL_WALL || m.type > nav_interfaces::msg::AreaInfo::BUMPY_AREA) { rej.push_back("area:" + m.name + ":type out of range"); continue; }
                if (m.polygon.size() < 3) { rej.push_back("area:" + m.name + ":polygon needs >= 3 points"); continue; }
                if (!guard("area", m.name, areas.count(m.name) > 0)) continue;
                areas[m.name] = AreaManager::fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->rooms) {
                if (m.name.empty()) { rej.push_back("room::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("room:" + m.name + ":duplicate in request"); continue; }
                if (m.type > nav_interfaces::msg::RoomInfo::STORAGE) { rej.push_back("room:" + m.name + ":type out of range"); continue; }
                if (m.polygon.size() < 3) { rej.push_back("room:" + m.name + ":polygon needs >= 3 points"); continue; }
                if (!guard("room", m.name, rooms.count(m.name) > 0)) continue;
                Room room = RoomManager::fromMsg(m);
                const std::string src = room.source.empty() ? std::string(DSGSource::MANUAL) : room.source;
                rooms[m.name] = RoomManager::enrich(std::move(room), src, now());
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->routes) {
                if (m.name.empty()) { rej.push_back("route::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("route:" + m.name + ":duplicate in request"); continue; }
                if (m.point_names.empty()) { rej.push_back("route:" + m.name + ":point_names is empty"); continue; }
                if (!guard("route", m.name, routes.count(m.name) > 0)) continue;
                routes[m.name] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->junctions) {
                if (m.id.empty()) { rej.push_back("junction::id is empty"); continue; }
                if (!seen.insert(m.id).second) { rej.push_back("junction:" + m.id + ":duplicate in request"); continue; }
                if (!guard("junction", m.id, jcts.count(m.id) > 0)) continue;
                jcts[m.id] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req->edges) {
                if (m.id.empty()) { rej.push_back("edge::id is empty"); continue; }
                if (!seen.insert(m.id).second) { rej.push_back("edge:" + m.id + ":duplicate in request"); continue; }
                if (!guard("edge", m.id, edges.count(m.id) > 0)) continue;
                edges[m.id] = fromMsg(m);
            }
        }
        if (!rej.empty()) { fail(guard_hit ? (add_only ? ST::E_ASSET_EXISTS : ST::E_ASSET_NOT_FOUND) : ST::E_ASSET_VALIDATION, "validation failed; nothing applied"); return; }
    }
    // 引用校验：所有引用必须在「应用后」的集合中可解析
    auto has_point = [&](const std::string& n) { return points.count(n) > 0; };
    auto has_jct   = [&](const std::string& i) { return jcts.count(i) > 0; };
    auto check_ep  = [&](const std::string& ekind, const std::string& eid, const RouteEndpoint& ep, const char* side) {
        if (ep.type == "nav")           { if (!has_point(ep.id)) rej.push_back(ekind + ":" + eid + ":" + side + " nav point '" + ep.id + "' not found"); }
        else if (ep.type == "junction") { if (!has_jct(ep.id))   rej.push_back(ekind + ":" + eid + ":" + side + " junction '" + ep.id + "' not found"); }
        else rej.push_back(ekind + ":" + eid + ":" + side + " endpoint type '" + ep.type + "' must be nav|junction");
    };
    for (const auto& kv : edges) { check_ep("edge", kv.first, kv.second.from, "from"); check_ep("edge", kv.first, kv.second.to, "to"); }
    for (const auto& kv : routes) for (const auto& n : kv.second.point_names) if (!has_point(n)) rej.push_back("route:" + kv.first + ":point '" + n + "' not found");
    for (const auto& kv : areas) {
        for (const auto& n : kv.second.related_nav_point_names)  if (!has_point(n)) rej.push_back("area:" + kv.first + ":related nav point '" + n + "' not found");
        for (const auto& n : kv.second.manual_linked_point_names) if (!has_point(n)) rej.push_back("area:" + kv.first + ":linked nav point '" + n + "' not found");
        for (const auto& i : kv.second.related_junction_ids)     if (!has_jct(i))   rej.push_back("area:" + kv.first + ":related junction '" + i + "' not found");
    }
    for (const auto& kv : rooms) {
        for (const auto& n : kv.second.related_nav_point_names)   if (!has_point(n)) rej.push_back("room:" + kv.first + ":related nav point '" + n + "' not found");
        for (const auto& n : kv.second.manual_anchor_point_names) if (!has_point(n)) rej.push_back("room:" + kv.first + ":anchor nav point '" + n + "' not found");
        for (const auto& i : kv.second.related_junction_ids)      if (!has_jct(i))   rej.push_back("room:" + kv.first + ":related junction '" + i + "' not found");
    }
    if (!rej.empty()) { fail(ST::E_ASSET_REF_UNRESOLVED, "unresolved references; nothing applied"); return; }
    // 贝塞尔边端点吸附到端点坐标（原单条 set_route_edge 的 snapEdgeEndpoints；按应用后的集合取坐标）
    {
        auto ep_pos = [&](const RouteEndpoint& ep, std::pair<float, float>& out) -> bool {
            if (ep.type == "nav")      { auto it = points.find(ep.id); if (it == points.end()) return false; out = {static_cast<float>(it->second.x), static_cast<float>(it->second.y)}; return true; }
            if (ep.type == "junction") { auto it = jcts.find(ep.id);   if (it == jcts.end())   return false; out = {it->second.x, it->second.y}; return true; }
            return false;
        };
        for (const auto& m : req->edges) {
            if (remove) break;
            auto it = edges.find(m.id); if (it == edges.end() || it->second.edge_type != "BEZIER") continue;
            std::pair<float, float> a, b; if (!ep_pos(it->second.from, a) || !ep_pos(it->second.to, b)) continue;
            auto& cp = it->second.control_points;
            if (cp.empty()) cp = {a, b};
            else if (cp.size() == 1) { const auto mid = cp.front(); cp = {a, mid, b}; }
            else { cp.front() = a; cp.back() = b; }
        }
    }
    // ── 应用（六个 store 整体替换）──
    point_store_->setPoints(points);
    route_store_->setRoutes(routes);
    area_store_->setAreas(areas);
    room_store_->setRooms(rooms);
    { std::vector<RouteJunction> v; v.reserve(jcts.size());  for (auto& kv : jcts)  v.push_back(kv.second); junction_store_->set(v); }
    { std::vector<RouteEdge> v;     v.reserve(edges.size()); for (auto& kv : edges) v.push_back(kv.second); edge_store_->set(v); }
    // 批量路径不经事件总线，空间归属索引（Area/Room 的 related_*）要显式重算，
    // 否则与单条 set_* 路径行为不一致（X-Wiz 批量写后 related_* 停留在提交值）。
    // rebuildAll 会经 on_*_changed 钩子发布 areas/rooms 并按 auto_save 参数落盘。
    if (spatial_indexer_) spatial_indexer_->rebuildAll();
    res->points_count    = static_cast<uint32_t>(points.size());
    res->areas_count     = static_cast<uint32_t>(areas.size());
    res->rooms_count     = static_cast<uint32_t>(rooms.size());
    res->routes_count    = static_cast<uint32_t>(routes.size());
    res->junctions_count = static_cast<uint32_t>(jcts.size());
    res->edges_count     = static_cast<uint32_t>(edges.size());
    last_error_code_ = ST::E_NONE;
    // ── 可选落盘（等价于 save_assets：原子写 + 产一份版本）──
    std::string saved;
    if (req->save) {
        if (assets_file_.empty()) {
            last_error_code_ = ST::E_ASSET_SAVE_FAILED;
            refreshAllOutputs();
            res->success = false; res->error_code = last_error_code_;
            res->message = "applied in memory but assets_file_ is not set; cannot save";
            return;
        }
        try {
            // 从 store 取（含索引重算后的 related_*），不能用应用前的局部副本
            file_manager_->saveAllAssetsToFile(assets_file_, point_store_->getPointsMap(), route_store_->getRoutesMap(),
                                               area_store_->getAreasMap(), junction_store_->getList(),
                                               edge_store_->getAll(), room_store_->getRoomsMap());
            const std::string ts = MakeAssetRingBackup(assets_file_, get_logger());
            saved = " saved" + (ts.empty() ? std::string("") : " (version " + ts + ")");
        } catch (const std::exception& ex) {
            last_error_code_ = ST::E_ASSET_SAVE_FAILED;
            refreshAllOutputs();
            res->success = false; res->error_code = last_error_code_;
            res->message = std::string("applied in memory but save failed: ") + ex.what();
            return;
        }
    }
    refreshAllOutputs();
    res->success = true;
    res->error_code = ST::E_NONE;
    static const char* kModeName[] = {"replaced", "merged", "added", "updated", "removed"};
    res->message = std::string(kModeName[req->mode]) + (replace ? "(kinds=" + std::to_string(k) + ")" : "") + ": " +
                   std::to_string(points.size()) + " points, " + std::to_string(areas.size()) + " areas, " +
                   std::to_string(rooms.size()) + " rooms, " + std::to_string(routes.size()) + " routes, " +
                   std::to_string(jcts.size()) + " junctions, " + std::to_string(edges.size()) + " edges" + saved;
    RCLCPP_INFO(get_logger(), "set_assets %s", res->message.c_str());
}

void MapAssetManagerNode::saveAssetsCallback(
    const std::shared_ptr<nav_interfaces::srv::SaveAssets::Request>,
    std::shared_ptr<nav_interfaces::srv::SaveAssets::Response> res) {
    // 注：map_name 入参当前忽略——只对已加载地图（current_map_name_）操作；指定他图保存留待后续
    try {
        if (assets_file_.empty()) {
            res->success = false;
            res->message = "No map loaded yet (waiting for map_manager/map_state)";
            return;
        }
        saveAllAssets();
        // 显式 save_assets → 产一份资产 ring 版本（DEC-006/007），供 command_board 统一版本引用/回滚
        const std::string backup_ts = MakeAssetRingBackup(assets_file_, get_logger());
        res->success = true;
        res->assets_path = assets_file_;
        res->backup_ts = backup_ts;
        res->message = "Assets saved" +
                       (backup_ts.empty() ? std::string("") : (" (version " + backup_ts + ")"));
        RCLCPP_INFO(get_logger(), "Assets saved via service to %s%s", assets_file_.c_str(),
                    backup_ts.empty() ? "" : (" [version " + backup_ts + "]").c_str());
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
        res->message = "No map loaded (waiting for map_manager/map_state)";
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
            route_graph_mgr_->publishMarkers();

            RCLCPP_INFO(get_logger(),
                        "transform_assets [current map '%s']: x=%.3f y=%.3f yaw=%.4f",
                        current_map_name_.c_str(), req->x, req->y, req->yaw);

        } else {
            // ── 非当前地图：从文件加载 → 变换 → 写回 ─────────────────

            const std::string file_path = resolveAssetsFile(req->map_name);

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

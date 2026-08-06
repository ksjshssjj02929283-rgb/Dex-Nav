/**
 * @file test_stores.cpp
 * @brief PointStore / RouteStore / AreaStore 单元测试
 *
 * 覆盖范围：
 *   - PointStore  : 增删改查、重复添加、不存在查询/删除
 *   - RouteStore  : 增删改查、级联更新（onPointRemoved / onPointRenamed）
 *   - AreaStore   : 增删改查
 *
 * 运行方式：
 *   colcon test --packages-select map_asset_manager
 *   或 直接执行：./build/map_asset_manager/test_stores
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/route_manager/route_store.hpp"

using namespace map_asset_manager;

// ═══════════════════════════════════════════════════════════════════════
// 辅助函数
// ═══════════════════════════════════════════════════════════════════════

static NavPoint makePoint(const std::string& name,
                          const std::string& type = "STATION",
                          double x = 1.0, double y = 2.0, double yaw = 0.0,
                          double tol = 0.05) {
    NavPoint p;
    p.name = name;
    p.type = type;
    p.x = x;
    p.y = y;
    p.yaw = yaw;
    p.tolerance = tol;
    return p;
}

static Route makeRoute(const std::string& name,
                       const std::vector<std::string>& pts) {
    Route r;
    r.name = name;
    r.point_names = pts;
    return r;
}

static Area makeArea(const std::string& name,
                     uint32_t type = AreaType::SPEED_LIMIT_AREA,
                     double max_speed = 0.3) {
    Area a;
    a.name      = name;
    a.type      = type;
    a.max_speed = static_cast<float>(max_speed);
    return a;
}

// ═══════════════════════════════════════════════════════════════════════
// PointStore 测试
// ═══════════════════════════════════════════════════════════════════════

class PointStoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        store_ = std::make_unique<PointStore>(rclcpp::get_logger("test_point_store"));
    }
    std::unique_ptr<PointStore> store_;
};

// ── 新增 ────────────────────────────────────────────────────────────────

TEST_F(PointStoreTest, AddPoint_NewPoint_ReturnsOne) {
    auto pt = makePoint("p1");
    EXPECT_EQ(store_->addPoint(pt), 1);
}

TEST_F(PointStoreTest, AddPoint_DuplicateOverwrites_ReturnsTwo) {
    store_->addPoint(makePoint("p1"));
    auto pt2 = makePoint("p1", "CHARGING");
    EXPECT_EQ(store_->addPoint(pt2), 2);
    // 覆盖后类型已更新
    auto got = store_->getPoint("p1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->type, "CHARGING");
}

TEST_F(PointStoreTest, AddPoint_MultiplePoints_AllStored) {
    store_->addPoint(makePoint("p1"));
    store_->addPoint(makePoint("p2"));
    store_->addPoint(makePoint("p3"));
    EXPECT_EQ(store_->getAllPoints().size(), 3u);
}

// ── 查询 ────────────────────────────────────────────────────────────────

TEST_F(PointStoreTest, GetPoint_Exists_ReturnsCorrectData) {
    store_->addPoint(makePoint("charging_1", "CHARGING", 3.0, 4.0, 1.57, 0.1));
    auto got = store_->getPoint("charging_1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->name, "charging_1");
    EXPECT_EQ(got->type, "CHARGING");
    EXPECT_DOUBLE_EQ(got->x, 3.0);
    EXPECT_DOUBLE_EQ(got->y, 4.0);
    EXPECT_DOUBLE_EQ(got->yaw, 1.57);
    EXPECT_DOUBLE_EQ(got->tolerance, 0.1);
}

TEST_F(PointStoreTest, GetPoint_NotExists_ReturnsNullopt) {
    EXPECT_FALSE(store_->getPoint("nonexistent").has_value());
}

TEST_F(PointStoreTest, HasPoint_AfterAdd_ReturnsTrue) {
    store_->addPoint(makePoint("p1"));
    EXPECT_TRUE(store_->hasPoint("p1"));
}

TEST_F(PointStoreTest, HasPoint_NotAdded_ReturnsFalse) {
    EXPECT_FALSE(store_->hasPoint("p_missing"));
}

TEST_F(PointStoreTest, GetAllPoints_Empty_ReturnsEmptyVector) {
    EXPECT_TRUE(store_->getAllPoints().empty());
}

TEST_F(PointStoreTest, GetAllPoints_AfterAdd_ContainsAllPoints) {
    store_->addPoint(makePoint("p1"));
    store_->addPoint(makePoint("p2"));
    auto all = store_->getAllPoints();
    EXPECT_EQ(all.size(), 2u);
}

// ── 更新 ────────────────────────────────────────────────────────────────

TEST_F(PointStoreTest, UpdatePoint_Exists_UpdatesData) {
    store_->addPoint(makePoint("p1", "STATION", 1.0, 2.0));
    auto updated = makePoint("p1", "DOORWAIT", 9.0, 8.0);
    EXPECT_TRUE(store_->updatePoint(updated));

    auto got = store_->getPoint("p1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->type, "DOORWAIT");
    EXPECT_DOUBLE_EQ(got->x, 9.0);
}

TEST_F(PointStoreTest, UpdatePoint_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->updatePoint(makePoint("ghost")));
}

// ── 删除 ────────────────────────────────────────────────────────────────

TEST_F(PointStoreTest, RemovePoint_Exists_RemovesIt) {
    store_->addPoint(makePoint("p1"));
    EXPECT_TRUE(store_->removePoint("p1"));
    EXPECT_FALSE(store_->hasPoint("p1"));
    EXPECT_TRUE(store_->getAllPoints().empty());
}

TEST_F(PointStoreTest, RemovePoint_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->removePoint("ghost"));
}

TEST_F(PointStoreTest, RemovePoint_OnlyRemovesTarget) {
    store_->addPoint(makePoint("p1"));
    store_->addPoint(makePoint("p2"));
    store_->removePoint("p1");
    EXPECT_FALSE(store_->hasPoint("p1"));
    EXPECT_TRUE(store_->hasPoint("p2"));
    EXPECT_EQ(store_->getAllPoints().size(), 1u);
}

// ── 清空 ────────────────────────────────────────────────────────────────

TEST_F(PointStoreTest, Clear_RemovesAllPoints) {
    store_->addPoint(makePoint("p1"));
    store_->addPoint(makePoint("p2"));
    store_->clear();
    EXPECT_TRUE(store_->getAllPoints().empty());
}

// ── GetPointsMap ────────────────────────────────────────────────────────

TEST_F(PointStoreTest, GetPointsMap_ContainsCorrectEntries) {
    store_->addPoint(makePoint("p1"));
    store_->addPoint(makePoint("p2"));
    auto m = store_->getPointsMap();
    EXPECT_EQ(m.size(), 2u);
    EXPECT_TRUE(m.count("p1"));
    EXPECT_TRUE(m.count("p2"));
}

// ── SetPoints (批量) ────────────────────────────────────────────────────

TEST_F(PointStoreTest, SetPoints_ReplacesExistingData) {
    store_->addPoint(makePoint("old"));
    std::unordered_map<std::string, NavPoint> batch;
    batch["new1"] = makePoint("new1");
    batch["new2"] = makePoint("new2");
    store_->setPoints(batch);
    EXPECT_FALSE(store_->hasPoint("old"));
    EXPECT_TRUE(store_->hasPoint("new1"));
    EXPECT_TRUE(store_->hasPoint("new2"));
}

// ═══════════════════════════════════════════════════════════════════════
// RouteStore 测试
// ═══════════════════════════════════════════════════════════════════════

class RouteStoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        store_ = std::make_unique<RouteStore>(rclcpp::get_logger("test_route_store"));
    }
    std::unique_ptr<RouteStore> store_;
};

// ── 新增 ────────────────────────────────────────────────────────────────

TEST_F(RouteStoreTest, AddRoute_New_ReturnsTrue) {
    EXPECT_TRUE(store_->addRoute(makeRoute("r1", {"p1", "p2", "p3"})));
}

TEST_F(RouteStoreTest, AddRoute_Duplicate_ReturnsFalse) {
    store_->addRoute(makeRoute("r1", {"p1"}));
    EXPECT_FALSE(store_->addRoute(makeRoute("r1", {"p2"})));
}

// ── 查询 ────────────────────────────────────────────────────────────────

TEST_F(RouteStoreTest, GetRoute_Exists_ReturnsCorrectData) {
    store_->addRoute(makeRoute("route_a", {"p1", "p2", "p3"}));
    auto got = store_->getRoute("route_a");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->name, "route_a");
    EXPECT_EQ(got->point_names.size(), 3u);
    EXPECT_EQ(got->point_names[0], "p1");
    EXPECT_EQ(got->point_names[2], "p3");
}

TEST_F(RouteStoreTest, GetRoute_NotExists_ReturnsNullopt) {
    EXPECT_FALSE(store_->getRoute("ghost").has_value());
}

TEST_F(RouteStoreTest, HasRoute_AfterAdd_ReturnsTrue) {
    store_->addRoute(makeRoute("r1", {}));
    EXPECT_TRUE(store_->hasRoute("r1"));
}

TEST_F(RouteStoreTest, GetAllRoutes_ReturnsAllRoutes) {
    store_->addRoute(makeRoute("r1", {"p1"}));
    store_->addRoute(makeRoute("r2", {"p2"}));
    EXPECT_EQ(store_->getAllRoutes().size(), 2u);
}

// ── 更新 ────────────────────────────────────────────────────────────────

TEST_F(RouteStoreTest, UpdateRoute_Exists_UpdatesPointNames) {
    store_->addRoute(makeRoute("r1", {"p1", "p2"}));
    EXPECT_TRUE(store_->updateRoute(makeRoute("r1", {"p1", "p2", "p3"})));
    auto got = store_->getRoute("r1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->point_names.size(), 3u);
}

TEST_F(RouteStoreTest, UpdateRoute_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->updateRoute(makeRoute("ghost", {})));
}

// ── 删除 ────────────────────────────────────────────────────────────────

TEST_F(RouteStoreTest, RemoveRoute_Exists_RemovesIt) {
    store_->addRoute(makeRoute("r1", {"p1"}));
    EXPECT_TRUE(store_->removeRoute("r1"));
    EXPECT_FALSE(store_->hasRoute("r1"));
}

TEST_F(RouteStoreTest, RemoveRoute_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->removeRoute("ghost"));
}

// ── 清空 ────────────────────────────────────────────────────────────────

TEST_F(RouteStoreTest, Clear_RemovesAllRoutes) {
    store_->addRoute(makeRoute("r1", {}));
    store_->addRoute(makeRoute("r2", {}));
    store_->clear();
    EXPECT_TRUE(store_->getAllRoutes().empty());
}

// ── 事件响应：点位删除级联 ──────────────────────────────────────────────

TEST_F(RouteStoreTest, OnPointRemoved_RemovesPointFromAllRoutes) {
    store_->addRoute(makeRoute("r1", {"p1", "p2", "p3"}));
    store_->addRoute(makeRoute("r2", {"p2", "p4"}));
    store_->addRoute(makeRoute("r3", {"p5", "p6"}));  // 不含 p2

    auto modified = store_->onPointRemoved("p2");

    // r1 和 r2 被修改，r3 未被修改
    EXPECT_EQ(modified.size(), 2u);

    auto r1 = store_->getRoute("r1");
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->point_names.size(), 2u);
    EXPECT_EQ(r1->point_names[0], "p1");
    EXPECT_EQ(r1->point_names[1], "p3");

    auto r2 = store_->getRoute("r2");
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r2->point_names.size(), 1u);
    EXPECT_EQ(r2->point_names[0], "p4");

    auto r3 = store_->getRoute("r3");
    ASSERT_TRUE(r3.has_value());
    EXPECT_EQ(r3->point_names.size(), 2u);  // 未变
}

TEST_F(RouteStoreTest, OnPointRemoved_PointNotInAnyRoute_ReturnsEmpty) {
    store_->addRoute(makeRoute("r1", {"p1", "p2"}));
    auto modified = store_->onPointRemoved("p_not_exists");
    EXPECT_TRUE(modified.empty());
}

TEST_F(RouteStoreTest, OnPointRemoved_EmptyStore_ReturnsEmpty) {
    auto modified = store_->onPointRemoved("p1");
    EXPECT_TRUE(modified.empty());
}

// ── 事件响应：点位重命名级联 ────────────────────────────────────────────

TEST_F(RouteStoreTest, OnPointRenamed_UpdatesAllRoutes) {
    store_->addRoute(makeRoute("r1", {"old_name", "p2"}));
    store_->addRoute(makeRoute("r2", {"p3", "old_name"}));
    store_->addRoute(makeRoute("r3", {"p4", "p5"}));  // 不含 old_name

    auto modified = store_->onPointRenamed("old_name", "new_name");

    EXPECT_EQ(modified.size(), 2u);

    auto r1 = store_->getRoute("r1");
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->point_names[0], "new_name");

    auto r2 = store_->getRoute("r2");
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r2->point_names[1], "new_name");

    auto r3 = store_->getRoute("r3");
    ASSERT_TRUE(r3.has_value());
    // r3 未被修改
    EXPECT_EQ(r3->point_names[0], "p4");
}

TEST_F(RouteStoreTest, OnPointRenamed_PointOccursMultipleTimes_AllRenamed) {
    // 一个路线中同一点位出现多次（异常但需处理）
    store_->addRoute(makeRoute("r1", {"p1", "p1", "p2"}));
    store_->onPointRenamed("p1", "p1_new");
    auto r1 = store_->getRoute("r1");
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->point_names[0], "p1_new");
    EXPECT_EQ(r1->point_names[1], "p1_new");
}

// ═══════════════════════════════════════════════════════════════════════
// AreaStore 测试
// ═══════════════════════════════════════════════════════════════════════

class AreaStoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        store_ = std::make_unique<AreaStore>(rclcpp::get_logger("test_area_store"));
    }
    std::unique_ptr<AreaStore> store_;
};

// ── 新增 ────────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, AddArea_New_ReturnsTrue) {
    EXPECT_TRUE(store_->addArea(makeArea("a1")));
}

TEST_F(AreaStoreTest, AddArea_Duplicate_ReturnsFalse) {
    store_->addArea(makeArea("a1"));
    EXPECT_FALSE(store_->addArea(makeArea("a1")));
}

TEST_F(AreaStoreTest, AddArea_WithPolygon_StoresPolygon) {
    Area a = makeArea("zone1", AreaType::RESTRICTED_AREA);
    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 1.0; p2.y = 0.0;
    p3.x = 1.0; p3.y = 1.0;
    a.polygon = {p1, p2, p3};

    store_->addArea(a);
    auto got = store_->getArea("zone1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->polygon.size(), 3u);
    EXPECT_DOUBLE_EQ(got->polygon[1].x, 1.0);
}

// ── 查询 ────────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, GetArea_Exists_ReturnsCorrectData) {
    store_->addArea(makeArea("slow_zone", AreaType::SPEED_LIMIT_AREA, 0.5));
    auto got = store_->getArea("slow_zone");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->name, "slow_zone");
    EXPECT_EQ(got->type, AreaType::SPEED_LIMIT_AREA);
    EXPECT_FLOAT_EQ(got->max_speed, 0.5f);
}

TEST_F(AreaStoreTest, GetArea_NotExists_ReturnsNullopt) {
    EXPECT_FALSE(store_->getArea("ghost").has_value());
}

TEST_F(AreaStoreTest, HasArea_AfterAdd_ReturnsTrue) {
    store_->addArea(makeArea("a1"));
    EXPECT_TRUE(store_->hasArea("a1"));
}

TEST_F(AreaStoreTest, HasArea_NotAdded_ReturnsFalse) {
    EXPECT_FALSE(store_->hasArea("missing"));
}

TEST_F(AreaStoreTest, GetAllAreas_ReturnsAll) {
    store_->addArea(makeArea("a1"));
    store_->addArea(makeArea("a2"));
    store_->addArea(makeArea("a3"));
    EXPECT_EQ(store_->getAllAreas().size(), 3u);
}

// ── 更新 ────────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, UpdateArea_Exists_UpdatesData) {
    store_->addArea(makeArea("a1", AreaType::SPEED_LIMIT_AREA, 0.3));
    Area updated = makeArea("a1", AreaType::RESTRICTED_AREA, 0.0);

    geometry_msgs::msg::Point p;
    p.x = 5.0; p.y = 5.0;
    updated.polygon = {p};

    EXPECT_TRUE(store_->updateArea(updated));
    auto got = store_->getArea("a1");
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->type, AreaType::RESTRICTED_AREA);
    EXPECT_EQ(got->polygon.size(), 1u);
}

TEST_F(AreaStoreTest, UpdateArea_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->updateArea(makeArea("ghost")));
}

// ── 删除 ────────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, RemoveArea_Exists_RemovesIt) {
    store_->addArea(makeArea("a1"));
    EXPECT_TRUE(store_->removeArea("a1"));
    EXPECT_FALSE(store_->hasArea("a1"));
}

TEST_F(AreaStoreTest, RemoveArea_NotExists_ReturnsFalse) {
    EXPECT_FALSE(store_->removeArea("ghost"));
}

TEST_F(AreaStoreTest, RemoveArea_OnlyRemovesTarget) {
    store_->addArea(makeArea("a1"));
    store_->addArea(makeArea("a2"));
    store_->removeArea("a1");
    EXPECT_FALSE(store_->hasArea("a1"));
    EXPECT_TRUE(store_->hasArea("a2"));
}

// ── 清空 ────────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, Clear_RemovesAllAreas) {
    store_->addArea(makeArea("a1"));
    store_->addArea(makeArea("a2"));
    store_->clear();
    EXPECT_TRUE(store_->getAllAreas().empty());
}

// ── GetAreasMap ─────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, GetAreasMap_ContainsCorrectEntries) {
    store_->addArea(makeArea("a1"));
    store_->addArea(makeArea("a2"));
    auto m = store_->getAreasMap();
    EXPECT_EQ(m.size(), 2u);
    EXPECT_TRUE(m.count("a1"));
    EXPECT_TRUE(m.count("a2"));
}

// ── 关联点位 ────────────────────────────────────────────────────────────

TEST_F(AreaStoreTest, AddArea_WithLinkedPoint_StoresLinkedPoint) {
    Area a = makeArea("charging_zone", AreaType::CHARGING_AREA);
    a.manual_linked_point_names.push_back("charging_1");
    store_->addArea(a);
    auto got = store_->getArea("charging_zone");
    ASSERT_TRUE(got.has_value());
    ASSERT_EQ(got->manual_linked_point_names.size(), 1u);
    EXPECT_EQ(got->manual_linked_point_names.front(), "charging_1");
}

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

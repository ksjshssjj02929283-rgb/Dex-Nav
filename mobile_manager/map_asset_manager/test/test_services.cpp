/**
 * @file test_services.cpp
 * @brief NavPointManager / RouteManager / RouteGraphManager ROS2 Service 集成测试
 *
 * 策略：
 *   - 直接构造 NavPointManager / RouteManager（注入 Store + EventBus）
 *   - 用同一进程内的 rclcpp 客户端调用服务，通过 Executor 驱动
 *   - 不依赖外部 map_folder 话题或文件系统
 *
 * 运行方式：
 *   colcon test --packages-select map_asset_manager
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/point_manager/nav_point_manager.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/route_graph_manager/junction_store.hpp"
#include "map_asset_manager/route_graph_manager/route_edge_store.hpp"
#include "map_asset_manager/route_graph_manager/route_graph_manager.hpp"
#include "map_asset_manager/route_manager/route_manager.hpp"
#include "map_asset_manager/route_manager/route_store.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"
#include "nav_interfaces/srv/get_nav_point.hpp"
#include "nav_interfaces/srv/get_route.hpp"

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════
// 辅助函数
// ═══════════════════════════════════════════════════════════════════════

static nav_interfaces::msg::NavPoint makeMsg(const std::string& name,
                                             const std::string& type = "STATION",
                                             float x = 1.0f, float y = 2.0f,
                                             float theta = 0.0f) {
    nav_interfaces::msg::NavPoint pt;
    pt.name = name;
    pt.type = type;
    pt.x = x;
    pt.y = y;
    pt.theta = theta;
    return pt;
}

// 同步调用服务（阻塞直到收到响应或超时）
template <typename SrvT>
typename SrvT::Response::SharedPtr callService(
    rclcpp::Node::SharedPtr client_node,
    const std::string& service_name,
    typename SrvT::Request::SharedPtr request,
    rclcpp::executors::SingleThreadedExecutor& exec,
    std::chrono::milliseconds timeout = 2000ms) {

    auto client = client_node->create_client<SrvT>(service_name);
    EXPECT_TRUE(client->wait_for_service(timeout)) << "Service not available: " << service_name;

    auto future = client->async_send_request(request);
    auto status = exec.spin_until_future_complete(future, timeout);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
    }
    return future.get();
}

// ═══════════════════════════════════════════════════════════════════════
// 测试夹具：NavPointManager
// ═══════════════════════════════════════════════════════════════════════

class NavPointManagerServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 宿主节点（注册服务）
        host_node_ = std::make_shared<rclcpp::Node>("test_host_nav");
        // 客户端节点（发起请求）
        client_node_ = std::make_shared<rclcpp::Node>("test_client_nav");

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(host_node_);
        executor_->add_node(client_node_);

        point_store_ = std::make_shared<map_asset_manager::PointStore>(
            host_node_->get_logger());
        event_bus_ = std::make_shared<map_asset_manager::DataEventBus>();

        save_called_ = 0;
        auto save_fn = [this]() { ++save_called_; };

        point_mgr_ = std::make_shared<map_asset_manager::NavPointManager>(
            host_node_.get(), point_store_, event_bus_, save_fn);

        // 服务名（~/xxx → /test_host_nav/xxx）
        prefix_ = "/test_host_nav";
    }

    void TearDown() override {
        executor_->cancel();
        point_mgr_.reset();
        host_node_.reset();
        client_node_.reset();
    }

    template <typename SrvT>
    typename SrvT::Response::SharedPtr call(const std::string& srv_name,
                                            typename SrvT::Request::SharedPtr req) {
        return callService<SrvT>(client_node_, prefix_ + srv_name, req, *executor_);
    }

    rclcpp::Node::SharedPtr host_node_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<map_asset_manager::PointStore> point_store_;
    std::shared_ptr<map_asset_manager::DataEventBus> event_bus_;
    std::shared_ptr<map_asset_manager::NavPointManager> point_mgr_;
    std::string prefix_;
    int save_called_;
};

// ── 新增导航点 ──────────────────────────────────────────────────────────



// ── 查询导航点 ──────────────────────────────────────────────────────────


TEST_F(NavPointManagerServiceTest, GetPoint_NotExists_ReturnsFailure) {
    auto req = std::make_shared<nav_interfaces::srv::GetNavPoint::Request>();
    req->name = "ghost";
    auto res = call<nav_interfaces::srv::GetNavPoint>("/get_point", req);
    ASSERT_NE(res, nullptr);
    EXPECT_FALSE(res->success);
}


// ── 更新导航点 ──────────────────────────────────────────────────────────



// ── 删除导航点 ──────────────────────────────────────────────────────────



// ── 删除点位触发路网级联（via EventBus） ──────────────────────────────


// ═══════════════════════════════════════════════════════════════════════
// 测试夹具：RouteManager
// ═══════════════════════════════════════════════════════════════════════

class RouteManagerServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        host_node_ = std::make_shared<rclcpp::Node>("test_host_route");
        client_node_ = std::make_shared<rclcpp::Node>("test_client_route");

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(host_node_);
        executor_->add_node(client_node_);

        point_store_ = std::make_shared<map_asset_manager::PointStore>(
            host_node_->get_logger());
        route_store_ = std::make_shared<map_asset_manager::RouteStore>(
            host_node_->get_logger());
        event_bus_ = std::make_shared<map_asset_manager::DataEventBus>();

        save_called_ = 0;
        route_mgr_ = std::make_shared<map_asset_manager::RouteManager>(
            host_node_.get(), route_store_, point_store_, event_bus_,
            [this]() { ++save_called_; });

        prefix_ = "/test_host_route";

        // 预置导航点
        point_store_->addPoint({"p1", "STATION", 1.0, 2.0, 0.0, 0.05});
        point_store_->addPoint({"p2", "STATION", 3.0, 4.0, 0.0, 0.05});
        point_store_->addPoint({"p3", "STATION", 5.0, 6.0, 0.0, 0.05});
    }

    void TearDown() override {
        executor_->cancel();
        route_mgr_.reset();
        host_node_.reset();
        client_node_.reset();
    }

    template <typename SrvT>
    typename SrvT::Response::SharedPtr call(const std::string& srv_name,
                                            typename SrvT::Request::SharedPtr req) {
        return callService<SrvT>(client_node_, prefix_ + srv_name, req, *executor_);
    }

    rclcpp::Node::SharedPtr host_node_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<map_asset_manager::PointStore> point_store_;
    std::shared_ptr<map_asset_manager::RouteStore> route_store_;
    std::shared_ptr<map_asset_manager::DataEventBus> event_bus_;
    std::shared_ptr<map_asset_manager::RouteManager> route_mgr_;
    std::string prefix_;
    int save_called_;
};

// ── 新增路线 ────────────────────────────────────────────────────────────





// ── 查询路线 ────────────────────────────────────────────────────────────

TEST_F(RouteManagerServiceTest, GetRoute_Exists_ReturnsData) {
    route_store_->addRoute({"route_a", {"p1", "p2", "p3"}});

    auto req = std::make_shared<nav_interfaces::srv::GetRoute::Request>();
    req->name = "route_a";
    auto res = call<nav_interfaces::srv::GetRoute>("/get_route", req);

    ASSERT_NE(res, nullptr);
    EXPECT_TRUE(res->success);
    EXPECT_EQ(res->name, "route_a");
    EXPECT_EQ(res->point_names.size(), 3u);
    EXPECT_EQ(res->point_names[0], "p1");
    EXPECT_EQ(res->point_names[2], "p3");
}

TEST_F(RouteManagerServiceTest, GetRoute_NotExists_Failure) {
    auto req = std::make_shared<nav_interfaces::srv::GetRoute::Request>();
    req->name = "ghost_route";
    auto res = call<nav_interfaces::srv::GetRoute>("/get_route", req);
    ASSERT_NE(res, nullptr);
    EXPECT_FALSE(res->success);
}

// ── 删除路线 ────────────────────────────────────────────────────────────



// ── 保存触发 ────────────────────────────────────────────────────────────


// ═══════════════════════════════════════════════════════════════════════
// 测试夹具：RouteGraphManager
// ═══════════════════════════════════════════════════════════════════════

class RouteGraphManagerServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        host_node_ = std::make_shared<rclcpp::Node>("test_host_route_graph");
        client_node_ = std::make_shared<rclcpp::Node>("test_client_route_graph");

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(host_node_);
        executor_->add_node(client_node_);

        point_store_ = std::make_shared<map_asset_manager::PointStore>(
            host_node_->get_logger());
        junction_store_ = std::make_shared<map_asset_manager::JunctionStore>(
            host_node_->get_logger());
        edge_store_ = std::make_shared<map_asset_manager::RouteEdgeStore>(
            host_node_->get_logger());
        event_bus_ = std::make_shared<map_asset_manager::DataEventBus>();

        point_store_->addPoint({"p1", "STATION", 1.0, 2.0, 0.0, 0.05});
        point_store_->addPoint({"p2", "STATION", 8.0, 9.0, 0.0, 0.05});
        graph_mgr_ = std::make_shared<map_asset_manager::RouteGraphManager>(
            host_node_.get(), junction_store_, edge_store_, point_store_,
            event_bus_, [this]() { ++save_called_; });
        prefix_ = "/test_host_route_graph";
    }

    void TearDown() override {
        executor_->cancel();
        graph_mgr_.reset();
        host_node_.reset();
        client_node_.reset();
    }

    template <typename SrvT>
    typename SrvT::Response::SharedPtr call(
        const std::string& srv_name, typename SrvT::Request::SharedPtr req) {
        return callService<SrvT>(client_node_, prefix_ + srv_name, req, *executor_);
    }

    static geometry_msgs::msg::Point point(double x, double y) {
        geometry_msgs::msg::Point value;
        value.x = x;
        value.y = y;
        return value;
    }

    rclcpp::Node::SharedPtr host_node_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<map_asset_manager::PointStore> point_store_;
    std::shared_ptr<map_asset_manager::JunctionStore> junction_store_;
    std::shared_ptr<map_asset_manager::RouteEdgeStore> edge_store_;
    std::shared_ptr<map_asset_manager::DataEventBus> event_bus_;
    std::shared_ptr<map_asset_manager::RouteGraphManager> graph_mgr_;
    std::string prefix_;
    int save_called_{0};
};


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

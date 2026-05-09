#pragma once

#include <nav_interfaces/srv/get_nav_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cl_nav2z/client_behaviors/cb_navigate_global_position.hpp>
#include <cl_nav2z/cl_nav2z.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace smacc2_nav_mission
{

/**
 * @brief 通过 nav_point_manager 的 GetNavPoint 服务查询点位坐标，
 *        再驱动 ClNav2Z 向 Nav2 发送 NavigateToPose Goal。
 *
 * 使用方法：
 *   configure_orthogonal<OrNavigation, CbNavigateToNavPoint>("station_A");
 *
 * 依赖：
 *   - OrNavigation 中的 ClNav2Z（发送 Action Goal）
 *   - nav_point_manager_node 节点（提供 GetNavPoint 服务）
 */
class CbNavigateToNavPoint : public cl_nav2z::CbNav2ZClientBehaviorBase
{
public:
  explicit CbNavigateToNavPoint(
    std::string point_name,
    std::string nav_point_manager_ns = "/nav_point_manager_node")
  : point_name_(point_name), nav_point_manager_ns_(nav_point_manager_ns)
  {
  }

  // 供 runtimeConfigure 动态修改目标点名称
  void setPointName(const std::string & name) { point_name_ = name; }

  void onEntry() override
  {
    auto node = getNode();

    // 1. 创建 GetNavPoint 服务客户端
    auto client = node->create_client<nav_interfaces::srv::GetNavPoint>(
      nav_point_manager_ns_ + "/get_point");

    // 2. 等待服务就绪，超时 3s
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(
        getLogger(),
        "[CbNavigateToNavPoint] GetNavPoint service not available: %s/get_point",
        nav_point_manager_ns_.c_str());
      this->postFailureEvent();
      return;
    }

    // 3. 发送服务请求
    auto request = std::make_shared<nav_interfaces::srv::GetNavPoint::Request>();
    request->name = point_name_;

    auto future = client->async_send_request(request);

    // 4. 同步等待响应（最多 5s）
    if (rclcpp::spin_until_future_complete(
        node->get_node_base_interface(), future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(getLogger(), "[CbNavigateToNavPoint] Service call timed out for point: %s",
        point_name_.c_str());
      this->postFailureEvent();
      return;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(getLogger(), "[CbNavigateToNavPoint] Point '%s' not found in nav_point_manager",
        point_name_.c_str());
      this->postFailureEvent();
      return;
    }

    // 5. 从响应中取坐标，构造 NavigateToPose Goal
    const auto & pt = response->nav_point;
    RCLCPP_INFO(getLogger(), "[CbNavigateToNavPoint] Navigating to '%s': x=%.3f y=%.3f theta=%.3f",
      pt.name.c_str(), static_cast<double>(pt.x), static_cast<double>(pt.y),
      static_cast<double>(pt.theta));

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = pt.x;
    goal.pose.pose.position.y = pt.y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.theta);
    goal.pose.pose.orientation = tf2::toMsg(q);

    // 6. 通过 ClNav2Z 发送 Goal（使用父类 sendGoal）
    this->sendGoal(goal);
  }

  void onExit() override
  {
    RCLCPP_INFO(getLogger(), "[CbNavigateToNavPoint] exiting, point: %s", point_name_.c_str());
  }

private:
  std::string point_name_;
  std::string nav_point_manager_ns_;
};

}  // namespace smacc2_nav_mission

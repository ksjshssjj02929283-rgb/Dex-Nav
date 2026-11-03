// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "nav2_controller/controller_server.hpp"
#include <condition_variable>
namespace nav2_controller
{
using SnapshotClock = std::chrono::steady_clock;

/**
 * @brief 快照化多速率控制运行上下文结构体（内部实现细节，Pimpl 风格隔离）。
 * 
 * 协调 10 Hz 独立规划线程（Worker）与 25 Hz 高频控制线程之间的无锁/短锁数据交接。
 */
struct ControllerServer::SnapshotRuntime
{
  /**
   * @brief 控制线程向规划线程投递的微型请求快照（避免高频复制大对象）。
   */
  struct Request
  {
    uint64_t session{}, revision{};                    ///< 导航任务代次 session 与路径/限速变更版本 revision
    std::shared_ptr<const nav_msgs::msg::Path> path;   ///< 全局路径只读共享指针（仅在路径变化时在锁外构建）
    geometry_msgs::msg::PoseStamped pose;              ///< 当前机器人在全局路径参考系下的位姿
    geometry_msgs::msg::Twist velocity;                ///< 当前底盘里程计速度
    nav2_msgs::msg::SpeedLimit limit;                  ///< 当前活动限速指令
    bool has_limit{};                                  ///< 是否处于限速状态
    double acceleration{};                             ///< 上一控制周期跟踪的目标加速度反馈
    bool acceleration_valid{};                         ///< 加速度反馈是否新鲜且有效
    double target_velocity{};                          ///< 上一控制周期跟踪的目标速度反馈
    bool target_velocity_valid{};                      ///< 目标速度反馈是否新鲜且有效
  };

  /**
   * @brief 规划线程向控制线程交付的规划计算产物快照。
   */
  struct Result
  {
    uint64_t session{}, revision{}, sequence{};        ///< 对应的请求代次、版本与自增流水号
    nav_interfaces::msg::RobotTrajectory trajectory;   ///< LocalPlanner + SpeedPlanner 生成的时空轨迹
    nav2_core::SpeedPlanningDiagnostics diagnostics;   ///< 速度规划器诊断信息（含末端制动储备、是否可行）
    SnapshotClock::time_point completed;               ///< 规划完成时的单调时间戳（用于计算时效性 max_age）
    bool success{};                                    ///< 轨迹是否成功生成且通过点合法性校验
  };

  std::mutex mutex;                                    ///< 用于保护请求与结果快照交接的短互斥锁
  std::condition_variable wake;                        ///< 用于唤醒规划工作线程的条件变量
  std::thread worker;                                  ///< 10 Hz 持久化后台规划线程
  bool quit{false};                                    ///< 线程退出标志
  uint64_t session{0}, revision{0}, limit_revision{0}; ///< 当前全局代次、路径版本及限速版本
  nav2_msgs::msg::SpeedLimit limit;                    ///< 缓存的最新限速参数
  bool has_limit{false};                               ///< 是否存在活动限速
  double feedback_acceleration{0};                     ///< 控制端反馈的目标加速度
  double feedback_velocity{0};                         ///< 控制端反馈的目标速度
  bool feedback_valid{false};                          ///< 反馈有效性标志
  SnapshotClock::time_point feedback_time{};           ///< 反馈更新时刻
  SnapshotClock::time_point accepted_completed{};      ///< 当前已被控制端采纳的轨迹完成时刻
  uint64_t accepted_revision{0}, accepted_session{0}, accepted_limit{0}; ///< 已采纳的版本号
  double planning_hz{10.0};                            ///< 规划工作线程执行频率 (Hz)
  double max_age{0.3};                                 ///< 允许消费的轨迹最大陈旧时长 (s)
  double local_control_stall_timeout{30.0};             ///< 局部控制连续静止超时 (s)
  double stall_linear_velocity_epsilon{0.01};          ///< 局部静止判定线速度阈值 (m/s)
  double stall_angular_velocity_epsilon{0.01};         ///< 局部静止判定角速度阈值 (rad/s)
  bool local_stall_active{false};                      ///< 是否正在累计局部静止时长
  SnapshotClock::time_point local_stall_started{};      ///< 本轮连续静止起始时刻
  Request request;                                     ///< 待规划消费的最新请求快照
  bool has_request{false};                             ///< 是否存在待处理请求
  std::shared_ptr<const Result> result;                ///< 供控制线程读取的最新不可变结果快照
  rclcpp_lifecycle::LifecyclePublisher<nav_interfaces::msg::RobotTrajectory>::SharedPtr publisher; ///< local_trajectory 发布器
};

}

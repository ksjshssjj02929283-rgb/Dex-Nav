// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


namespace nav2_behavior_tree
{

/**
 * @enum nav2_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @brief 行为树单轮回调的分段耗时，仅用于超周期诊断
 */
struct BtLoopTiming
{
  double preempt_ms{0.0};
  double topic_logger_flush_ms{0.0};
  double navigator_on_loop_ms{0.0};

  bool feedback_timing_valid{false};
  double feedback_tf_ms{0.0};
  double feedback_path_copy_ms{0.0};
  double feedback_path_calculation_ms{0.0};
  double feedback_odom_eta_ms{0.0};
  double feedback_finalize_ms{0.0};
  double feedback_publish_ms{0.0};
  std::size_t feedback_path_poses{0};
};

/**
 * @brief AdaptiveReplanning单轮内部耗时；total包含child，其余阶段互斥
 */
struct BtAdaptiveReplanningTiming
{
  bool valid{false};
  double total_ms{0.0};
  double ports_ms{0.0};
  double goal_update_ms{0.0};
  double tf_ms{0.0};
  double path_copy_ms{0.0};
  double path_scan_ms{0.0};
  double decision_ms{0.0};
  double child_ms{0.0};
  double other_ms{0.0};
  std::size_t path_poses{0};
};

/**
 * @brief 单轮tickRoot内部的诊断耗时，记录实际执行的BT叶节点和AdaptiveReplanning分段
 */
struct BtTickTiming
{
  struct NodeTiming
  {
    const std::string * name{nullptr};
    double duration_ms{0.0};
  };

  void reset()
  {
    slowest_nodes = {};
    status_event_ms = 0.0;
    status_event_count = 0;
    adaptive = {};
  }

  void recordNode(const std::string & name, const double duration_ms)
  {
    for (auto & node : slowest_nodes) {
      if (node.name == &name) {
        node.duration_ms = std::max(node.duration_ms, duration_ms);
        sortSlowestNodes();
        return;
      }
    }

    if (duration_ms <= slowest_nodes.back().duration_ms) {
      return;
    }
    NodeTiming & slowest = slowest_nodes.back();
    slowest.name = &name;
    slowest.duration_ms = duration_ms;
    sortSlowestNodes();
  }

  std::array<NodeTiming, 3> slowest_nodes;
  double status_event_ms{0.0};
  std::size_t status_event_count{0};
  BtAdaptiveReplanningTiming adaptive;

private:
  void sortSlowestNodes()
  {
    std::sort(
      slowest_nodes.begin(), slowest_nodes.end(),
      [](const auto & lhs, const auto & rhs) {return lhs.duration_ms > rhs.duration_ms;});
  }
};

/**
 * @brief 记录BT叶节点一次executeTick的耗时；未提供诊断对象时不读取时钟
 */
class BtScopedNodeTimer
{
public:
  BtScopedNodeTimer(BtTickTiming * timing, const std::string & node_name)
  : timing_(timing), node_name_(node_name)
  {
    if (timing_ != nullptr) {
      start_ = std::chrono::steady_clock::now();
    }
  }

  ~BtScopedNodeTimer()
  {
    if (timing_ != nullptr) {
      const double duration_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
      timing_->recordNode(node_name_, duration_ms);
    }
  }

private:
  BtTickTiming * timing_;
  const std::string & node_name_;
  std::chrono::steady_clock::time_point start_;
};

/**
 * @class nav2_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BehaviorTreeEngine
   * @param plugin_libraries vector of BT plugin library names to load
   */
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   * @param tree BT to execute
   * @param onLoop Function to execute on each iteration of BT execution
   * @param cancelRequested Function to check if cancel was requested during BT execution
   * @param loopTimeout Time period for each iteration of BT execution
   * @return nav2_behavior_tree::BtStatus Status of BT execution
   */
  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief 执行行为树，并在超周期时输出onLoop内部的分段耗时
   * @param tree 要执行的行为树
   * @param onLoop 每轮行为树执行后调用的函数
   * @param cancelRequested 检查导航是否已取消的函数
   * @param loopTimeout 单轮行为树的周期预算
   * @param loopTiming onLoop回调填写的分段耗时
   * @return 行为树最终执行状态
   */
  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout,
    const BtLoopTiming * loopTiming);

  /**
   * @brief 执行行为树，并补充tickRoot叶节点与AdaptiveReplanning内部耗时
   * @param tree 要执行的行为树
   * @param onLoop 每轮行为树执行后调用的函数
   * @param cancelRequested 检查导航是否已取消的函数
   * @param loopTimeout 单轮行为树的周期预算
   * @param loopTiming onLoop回调填写的分段耗时
   * @param tickTiming tickRoot执行期间由行为树节点填写的诊断耗时
   * @return 行为树最终执行状态
   */
  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout,
    const BtLoopTiming * loopTiming,
    BtTickTiming * tickTiming);

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to explicitly reset all BT nodes to initial state
   * @param root_node Pointer to BT root node
   */
  void haltAllActions(BT::TreeNode * root_node);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

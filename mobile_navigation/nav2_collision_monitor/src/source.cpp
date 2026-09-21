// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), source_timeout_(source_timeout),
  base_shift_correction_(base_shift_correction)
{
}

Source::~Source()
{
}

bool Source::getLatestTransform(
  const std::string & source_frame, const rclcpp::Time & source_stamp,
  tf2::Transform & tf_transform) const
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  try {
    if (base_shift_correction_) {
      const auto latest_base = tf_buffer_->lookupTransform(
        global_frame_id_, base_frame_id_, tf2::TimePointZero);
      const rclcpp::Time target_time(latest_base.header.stamp);
      // A zero stamp denotes an all-static chain. Dynamic TF must still be recent;
      // transform_tolerance is an age bound here, never a blocking wait budget.
      if (target_time.nanoseconds() != 0 &&
        (node->now() - target_time).seconds() > tf2::durationToSec(transform_tolerance_))
      {
        throw tf2::ExtrapolationException("Latest base TF exceeds transform_tolerance age bound");
      }
      const auto source_in_fixed = tf_buffer_->lookupTransform(
        global_frame_id_, source_frame, source_stamp,
        rclcpp::Duration::from_nanoseconds(0));
      tf2::Transform fixed_from_base;
      tf2::Transform fixed_from_source;
      tf2::fromMsg(latest_base.transform, fixed_from_base);
      tf2::fromMsg(source_in_fixed.transform, fixed_from_source);
      tf_transform = fixed_from_base.inverse() * fixed_from_source;
    } else {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_id_, source_frame, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, tf_transform);
    }
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 5000,
      "[%s]: Nonblocking source TF unavailable: %s", source_name_.c_str(), ex.what());
    return false;
  }
}

void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();
}

bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (dt > source_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

}  // namespace nav2_collision_monitor

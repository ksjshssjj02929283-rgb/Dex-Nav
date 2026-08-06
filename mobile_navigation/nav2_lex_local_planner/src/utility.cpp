// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 几何/路径函数：geometry_msgs::msg::PoseStamped；tf2::getYaw；
// tf2::Quaternion::setRPY + toMsg。
#include <cmath>
#include <vector>
#include <string>

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_lex_local_planner/data_type.hpp"
#include "nav2_lex_local_planner/utility.hpp"
#include "nav2_lex_local_planner/planning_params.hpp"

namespace nav2_lex_local_planner
{
/**
 * @description: 获取轨迹上距离当前位置最近的轨迹点（前方）
 */
int getNextClosePointIndex(const std::vector<nav2_lex_local_planner::WayPoint> & trajectory,
    const nav2_lex_local_planner::WayPoint & curr_pos,
    const int & prevIndex)
{
  if (trajectory.size() < 2 || prevIndex < 0)
    return 0;
  double dis = 0, min_dis = DBL_MAX;
  int min_index = prevIndex;

  for (int i = prevIndex; i < static_cast<int>(trajectory.size()); i++) {
    dis = distance2points_pow(trajectory[i].pos, curr_pos.pos);

    if (dis < min_dis) {
      min_index = i;
      min_dis = dis;
    }
  }

  if (min_index < static_cast<int>(trajectory.size()) - 2) {
    nav2_lex_local_planner::GPSPoint closest, next;
    closest = trajectory[min_index].pos;
    next = trajectory[min_index + 1].pos;
    nav2_lex_local_planner::GPSPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
    double length1 = calLength(v_1);
    nav2_lex_local_planner::GPSPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
    double length2 = calLength(v_2);
    double angle = cast_from_0_to_2PI_Angle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
    if (angle <= M_PI_2)
      min_index = min_index + 1;
  }
  return min_index;
}


double calDiffBetweenTwoAngle(const double & a1, const double & a2)
{
  double diff = a1 - a2;
  if (diff < 0)
    diff = a2 - a1;
  if (diff > M_PI)
    diff = 2.0 * M_PI - diff;
  return diff;
}


/**
 * @description: 将当前角度转换到 0 ~ 2pi
 */
double cast_from_0_to_2PI_Angle(const double & ang)
{
  double angle = 0;
  if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
    angle = fmod(ang, 2.0 * M_PI);
  } else {
    angle = ang;
  }

  if (angle < 0) {
    angle = 2.0 * M_PI + angle;
  }
  return angle;
}


/**
 * @description: 将当前角度转换到 -pi ~ pi
 */
double cast_from_PI_to_PI_Angle(const double & ang)
{
  double angle = 0;
  if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
    angle = fmod(ang, 2.0 * M_PI);
  } else {
    angle = ang;
  }

  if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}


double diffBetweenTwoAngle(const double & a1, const double & a2)
{
  double diff = a1 - a2;
  if (diff < 0)
    diff = -diff;
  if (diff > M_PI)
    diff = 2 * M_PI - diff;
  return diff;
}


/**
 * @brief 计算中心轨迹上距离当前位置最近的点的相对信息
 */
bool getRelativeInfo(const std::vector<nav2_lex_local_planner::WayPoint> & trajectory,
    const nav2_lex_local_planner::WayPoint & p,
    nav2_lex_local_planner::RelativeInfo & info)
{
  if (trajectory.size() < 2)
    return false;

  nav2_lex_local_planner::WayPoint p0, p1;
  if (trajectory.size() == 2) {
    p0 = trajectory[0];
    // p1 是 p0 和下一个点的中点
    p1 = nav2_lex_local_planner::WayPoint((p0.pos.x + trajectory[1].pos.x) / 2.0,
        (p0.pos.y + trajectory[1].pos.y) / 2.0,
        (p0.pos.z + trajectory[1].pos.z) / 2.0,
        p0.pos.yaw);
    info.iBack = 0;
    info.iFront = 1;
  } else {
    // 获取下一个最近的点
    info.iFront = getNextClosePointIndex(trajectory, p);

    if (info.iFront > 0)
      info.iBack = info.iFront - 1;  // iBack 是 iFront 的前一个点
    else
      info.iBack = 0;

    if (info.iFront == 0) {
      p0 = trajectory[info.iFront];
      p1 = trajectory[info.iFront + 1];
    } else if (info.iFront > 0 && info.iFront < static_cast<int>(trajectory.size()) - 1) {
      p0 = trajectory[info.iFront - 1];
      p1 = trajectory[info.iFront];
    } else {
      p0 = trajectory[info.iFront - 1];
      p1 = nav2_lex_local_planner::WayPoint((p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                                       (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                                       (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0,
                                       p0.pos.yaw);
    }
  }

  nav2_lex_local_planner::WayPoint prevWP = p0;
  nav2_lex_local_planner::Mat3 rotationMat(-p1.pos.yaw);
  nav2_lex_local_planner::Mat3 translationMat(-p.pos.x, -p.pos.y);
  nav2_lex_local_planner::Mat3 invRotationMat(p1.pos.yaw);
  nav2_lex_local_planner::Mat3 invTranslationMat(p.pos.x, p.pos.y);

  // p0 和 p1 转换到当前机器人坐标系下（点 p 的坐标系）
  p0.pos = translationMat * p0.pos;
  p0.pos = rotationMat * p0.pos;

  p1.pos = translationMat * p1.pos;
  p1.pos = rotationMat * p1.pos;

  double k = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
  info.perp_distance = p1.pos.y - k * p1.pos.x;

  if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) {
    info.perp_distance = 0;
  }

  info.to_front_distance = fabs(p1.pos.x);

  info.perp_point = p1;
  info.perp_point.pos.x = 0;
  info.perp_point.pos.y = info.perp_distance;

  info.perp_point.pos = invRotationMat * info.perp_point.pos;
  info.perp_point.pos = invTranslationMat * info.perp_point.pos;

  info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
  info.angle_diff = nav2_lex_local_planner::diffBetweenTwoAngle(p1.pos.yaw, p.pos.yaw) * RAD2DEG_ME;

  info.direct_distance = hypot(p1.pos.y - p.pos.y, p1.pos.x - p.pos.x);
  return true;
}


/**
 * @brief 对路径进行采样
 */
void fixPathDensity(std::vector<nav2_lex_local_planner::WayPoint> & path, const double & pathDensity)
{
  if (path.size() == 0 || pathDensity == 0)
    return;
  double dis = 0, ang = 0;
  double margin = pathDensity * 0.01;
  double remaining = 0;
  int nPoints = 0;
  std::vector<nav2_lex_local_planner::WayPoint> fixedPath;
  fixedPath.push_back(path[0]);
  size_t start = 0, next = 1;
  while (next < path.size()) {
    dis += hypot(path[next].pos.x - path[next - 1].pos.x, path[next].pos.y - path[next - 1].pos.y) + remaining;
    ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);

    if (dis < pathDensity - margin) {
      next++;
      remaining = 0;
    } else if (dis > (pathDensity + margin)) {
      nav2_lex_local_planner::WayPoint point_start = path[start];
      nPoints = dis / pathDensity;
      for (int j = 0; j < nPoints; j++) {
        point_start.pos.x = point_start.pos.x + pathDensity * cos(ang);
        point_start.pos.y = point_start.pos.y + pathDensity * sin(ang);
        fixedPath.push_back(point_start);
      }
      remaining = dis - nPoints * pathDensity;
      start++;
      path[start].pos = point_start.pos;
      dis = 0;
      next++;
    } else {
      dis = 0;
      remaining = 0;
      fixedPath.push_back(path[next]);
      next++;
      start = next - 1;
    }
  }
  path = fixedPath;
}


void poseStampedMsgToWayPoint(
        const geometry_msgs::msg::PoseStamped & pose,
        nav2_lex_local_planner::WayPoint & wp)
{
  wp.pos.x = pose.pose.position.x;
  wp.pos.y = pose.pose.position.y;
  wp.pos.z = pose.pose.position.z;
  wp.pos.yaw = tf2::getYaw(pose.pose.orientation);
  wp.v = 1;
}

/**
 * @brief 将路径消息转换为 WayPoint
 */
void poseVec2LocalLane(
        const std::vector<geometry_msgs::msg::PoseStamped> & pos_vec,
        std::vector<nav2_lex_local_planner::WayPoint> & path)
{
  path.clear();
  for (size_t i = 0; i < pos_vec.size(); i++) {
    nav2_lex_local_planner::WayPoint wp;
    wp.pos.x = pos_vec[i].pose.position.x;
    wp.pos.y = pos_vec[i].pose.position.y;
    wp.pos.z = pos_vec[i].pose.position.z;
    wp.pos.yaw = tf2::getYaw(pos_vec[i].pose.orientation);
    wp.v = 1;
    path.push_back(wp);
  }
}

void localLane2PoseVec(
        const std::vector<nav2_lex_local_planner::WayPoint> & path,
        std::vector<geometry_msgs::msg::PoseStamped> & pos_vec,
        const std::string & frame_id)
{
  pos_vec.clear();
  for (size_t i = 0; i < path.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = path[i].pos.x;
    pose.pose.position.y = path[i].pos.y;
    pose.pose.position.z = path[i].pos.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, path[i].pos.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    pos_vec.push_back(pose);
  }
}


void calculatePathYaw(std::vector<nav2_lex_local_planner::WayPoint> & pathIn)
{
  int length = pathIn.size();
  if (length <= 1) {
    if (length == 1) {
      pathIn[0].pos.yaw = 0.0;
    }
  }

  for (int i = 0; i < length - 1; ++i) {
    double dx = pathIn[i + 1].pos.x - pathIn[i].pos.x;
    double dy = pathIn[i + 1].pos.y - pathIn[i].pos.y;
    double theta = atan2(dy, dx);
    pathIn[i].pos.yaw = theta;
  }

  pathIn.back().pos.yaw = pathIn[length - 2].pos.yaw;
}


/**
 * @brief 对路径进行平滑
 * @param path 需要平滑的路径
 * @param weight_data 前后路径的权重
 * @param weight_smooth 上次平滑的权重
 * @param tolerance 平滑增量的阈值
 */
void smoothPath(std::vector<nav2_lex_local_planner::WayPoint> & path, double weight_data,
  double weight_smooth, double tolerance)
{
  if (path.size() <= 2)
    return;

  const std::vector<nav2_lex_local_planner::WayPoint> & path_in = path;
  std::vector<nav2_lex_local_planner::WayPoint> smoothPath_out = path_in;

  double change = tolerance;
  double xtemp, ytemp;
  int nIterations = 0;

  int size = path_in.size();

  while (change >= tolerance && nIterations < 100) {
    change = 0.0;
    for (int i = 1; i < size - 1; i++) {
      xtemp = smoothPath_out[i].pos.x;
      ytemp = smoothPath_out[i].pos.y;

      smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
      smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

      smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
      smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

      change += fabs(xtemp - smoothPath_out[i].pos.x);
      change += fabs(ytemp - smoothPath_out[i].pos.y);
    }
    nIterations++;
  }
  path = smoothPath_out;
}

/**
 * @brief 计算路径的角度和距离 cost
 */
double calcAngleAndCost(std::vector<nav2_lex_local_planner::WayPoint> & path)
{
  if (path.size() < 2)
    return 0;
  if (path.size() == 2) {
    path[0].pos.yaw = nav2_lex_local_planner::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;
    path[1].pos.yaw = path[0].pos.yaw;
    path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
    return path[1].cost;
  }

  path[0].pos.yaw = nav2_lex_local_planner::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
  path[0].cost = 0;

  for (int j = 1; j < static_cast<int>(path.size()) - 1; j++) {
    path[j].pos.yaw = nav2_lex_local_planner::cast_from_0_to_2PI_Angle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
  }

  int j = static_cast<int>(path.size()) - 1;
  path[j].pos.yaw = path[j - 1].pos.yaw;
  path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

  return path[j].cost;
}

}  // namespace nav2_lex_local_planner

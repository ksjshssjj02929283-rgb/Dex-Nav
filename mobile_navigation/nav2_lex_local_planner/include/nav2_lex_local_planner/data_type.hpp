// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 纯数据结构定义（无 ROS 依赖）。从 dcm lex_local_planner/data_type.h 原样移植，
// 仅替换 include guard。NUM_COSTS = 3 多代价（避障 / 换道 / 距离）。
#ifndef NAV2_LEX_LOCAL_PLANNER__DATA_TYPE_HPP_
#define NAV2_LEX_LOCAL_PLANNER__DATA_TYPE_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cfloat>

namespace nav2_lex_local_planner
{

#define NUM_COSTS 3

class GPSPoint
{
public:
  double x;
  double y;
  double z;
  double yaw;

  GPSPoint()
  {
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;
  }

  GPSPoint(const double& x, const double& y, const double& z, const double& yaw)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
  }

  std::string ToString()
  {
    std::stringstream str;
    str.precision(12);
    str << "X:" << x << ", Y:" << y << ", Z:" << z << ", Yaw:" << yaw << std::endl;
    return str.str();
  }
};


class Rotation
{
public:
  double x;
  double y;
  double z;
  double w;

  Rotation()
  {
    x = 0;
    y = 0;
    z = 0;
    w = 0;
  }
};


class WayPoint
{
public:
  GPSPoint pos;
  Rotation rot;
  double v;
  double cost;
  int laneId;

  WayPoint()
  {
    v = 0;
    cost = 0;
    laneId = -1;
  }

  WayPoint(const double& x, const double& y, const double& z, const double& a)
  {
    pos.x = x;
    pos.y = y;
    pos.z = z;
    pos.yaw = a;

    v = 0;
    cost = 0;
    laneId = -1;
  }
};


class Mat3
{
  double m[3][3];

public:
  Mat3()
  {
    // initialize Identity by default
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        m[i][j] = 0;

    m[0][0] = m[1][1] = m[2][2] = 1;
  }

  Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
  {
    m[0][0] = (mirrorX == true) ? -1 : 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = (mirrorY == true) ? -1 : 1;
    m[1][2] = transY;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double transX, double transY)
  {
    m[0][0] = 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = 1;
    m[1][2] = transY;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double rotation_angle)
  {
    double c = std::cos(rotation_angle);
    double s = std::sin(rotation_angle);
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = 0;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = 0;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(GPSPoint rotationCenter)
  {
    double c = std::cos(rotationCenter.yaw);
    double s = std::sin(rotationCenter.yaw);
    double u = rotationCenter.x;
    double v = rotationCenter.y;
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = -u * c + v * s + u;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = -u * s - v * c + v;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  GPSPoint operator*(GPSPoint v)
  {
    GPSPoint _v = v;
    v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
    v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
    return v;
  }
};


class RelativeInfo
{
public:
  double perp_distance;
  double to_front_distance;  // negative
  double from_back_distance;
  int iFront;
  int iBack;
  WayPoint perp_point;
  double angle_diff;  // degrees
  double direct_distance;

  RelativeInfo()
  {
    perp_distance = 0;
    to_front_distance = 0;
    iFront = 0;
    iBack = 0;
    angle_diff = 0;
    direct_distance = 0;
  }
};


class CAR_BASIC_INFO
{
public:
  double wheel_base;
  double length;
  double width;

  CAR_BASIC_INFO()
  {
    wheel_base = 2.7;
    length = 4.3;
    width = 1.82;
  }
};


struct state_t;
struct neighbor_t;


struct state_t
{
  float x = 0;
  float y = 0;
  float z = 0;
  float theta = 0;
  float costsToRoot[NUM_COSTS] = {FLT_MAX};

  int idx = -1;
  int idy = -1;
  int stateId = -1;

  bool validFlag = false;

  state_t* parentState = NULL;
  std::vector<neighbor_t> neighborList;
};


struct neighbor_t
{
  state_t* neighbor;
  float edgeCosts[NUM_COSTS];  // the cost from this state to neighbor
  neighbor_t()
  {
    neighbor = NULL;
    for (int i = 0; i < NUM_COSTS; ++i)
      edgeCosts[i] = FLT_MAX;
  }
};


}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__DATA_TYPE_HPP_

#ifndef MISSION_MANAGER__NAV_POINT_HPP_
#define MISSION_MANAGER__NAV_POINT_HPP_

#include <string>

namespace pnc {

enum NavPointType {
    UNKNOWN = -1,   // 未知类型
    FREE_GOAL = 0,  // 自由目标点
    STATION = 1,    // 站点（默认类型）
    CHARGING = 2,   // 充电点
    PARKING = 3,    // 停车点
    PICKUP = 4,     // 取货点
};

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
} PointPose;

typedef struct {
    std::string name;                                 // 目标点的名称
    int id;                                           // 目标点的id
    NavPointType point_type = NavPointType::STATION;  // 目标点的类型
    PointPose pose;
    // TODO: 可以添加其他属性，如描述信息、优先级等
} NavPoint;

}  // namespace pnc

#endif  // MISSION_MANAGER__NAV_POINT_HPP_

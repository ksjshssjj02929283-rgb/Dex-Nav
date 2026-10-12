#ifndef MAP_MANAGER_GRID_EDIT_HPP_
#define MAP_MANAGER_GRID_EDIT_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace map_manager {

// 底图笔画编辑的栅格化：只做像素运算、不碰文件，前后端一致性测试直接调它。
// 规则与前端预览逐格一致（见地图编辑方案「前后端像素一致性」一节），改这里必须同步前端。

// 与 nav_interfaces/msg/GridStroke.msg 的常量保持一致；核心层不依赖 ROS msg。
constexpr uint8_t kToolWall = 1;      // 覆盖到的非障碍格 → 障碍（0）
constexpr uint8_t kToolErase = 2;     // 覆盖到的障碍格 → 自由（255）；未知保持未知
constexpr uint8_t kToolUnknown = 3;   // 覆盖到的非未知格 → 未知
constexpr uint8_t kShapePolyline = 1; // 折线（单点 = 圆点），按笔刷直径画胶囊
constexpr uint8_t kShapeRect = 2;     // 两个对角点围成的轴对齐填充矩形，忽略宽度

struct GridStrokeSpec {
    uint8_t tool = 0;
    uint8_t shape = 0;
    std::vector<std::pair<double, double>> points;  // map 坐标系，米
    double width_m = 0.0;                           // 笔刷直径
};

// 栅格几何：origin 指向图像左下角，图像行序自上而下。
struct GridGeometry {
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    int width = 0;
    int height = 0;
};

// 与 map_server 同口径的三值判定阈值（来自每张图自己的 map.yaml）。
struct GridThresholds {
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
};

enum class CellClass { kFree, kOccupied, kUnknown };

// p > 255·(1−free_thresh) → 自由；p < 255·(1−occupied_thresh) → 障碍；其余 → 未知。
CellClass ClassifyPixel(uint8_t p, const GridThresholds& th);

// 「未知」写回的灰度：205（map_saver 约定）在本图阈值下确实落在未知区间就用 205，
// 否则取未知区间中值——free_thresh=0.2 的图上 205 会被判成自由。
// 阈值非法（未知区间为空）时返回 -1。
int UnknownGray(const GridThresholds& th);

// 阈值合法：0 < free_thresh < occupied_thresh < 1 且未知区间非空。
bool ValidThresholds(const GridThresholds& th);

// 校验笔画参数；合法返回空串，否则返回英文原因（用于 E_MAP_EDIT_INVALID 的 message）。
std::string ValidateStrokes(const std::vector<GridStrokeSpec>& strokes);

// 按顺序把笔画应用到 8 位单通道灰度图上（原地修改），返回发生变化的像素数。
// 只改被覆盖、且三值类别确实要变的像素，其余像素（含中间灰度）原样保留；越界部分裁掉。
// 调用方须先保证 ValidateStrokes 通过、ValidThresholds 为真、gray 为 CV_8UC1。
int ApplyStrokes(cv::Mat* gray, const GridGeometry& geo, const GridThresholds& th,
                 const std::vector<GridStrokeSpec>& strokes);

}  // namespace map_manager

#endif  // MAP_MANAGER_GRID_EDIT_HPP_

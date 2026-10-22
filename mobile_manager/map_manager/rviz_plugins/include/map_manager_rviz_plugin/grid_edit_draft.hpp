#ifndef MAP_MANAGER_RVIZ_PLUGIN__GRID_EDIT_DRAFT_HPP_
#define MAP_MANAGER_RVIZ_PLUGIN__GRID_EDIT_DRAFT_HPP_

#include <QObject>

#include <cstdint>
#include <utility>
#include <vector>

namespace map_manager_rviz_plugin {

// 与 nav_interfaces/msg/GridStroke.msg 的常量一致
constexpr uint8_t kToolWall = 1;
constexpr uint8_t kToolErase = 2;
constexpr uint8_t kToolUnknown = 3;
constexpr uint8_t kShapePolyline = 1;
constexpr uint8_t kShapeRect = 2;

enum class DrawMode { kFree = 0, kLine = 1, kRect = 2 };

struct DraftStroke {
    uint8_t tool = kToolWall;
    uint8_t shape = kShapePolyline;
    std::vector<std::pair<double, double>> points;  // map 坐标系，米
    double width_m = 0.2;
};

// 底图编辑草稿：画笔工具往里加笔画，面板读它预览、提交。
// 两者是 RViz 里独立加载的插件，用进程内单例共享；只在 Qt 主线程访问。
class GridEditDraft : public QObject {
    Q_OBJECT

public:
    static GridEditDraft& instance();

    // 当前画笔设置（面板写，工具读）
    uint8_t tool = kToolWall;
    DrawMode mode = DrawMode::kFree;
    double width_m = 0.2;
    bool editing = false;  // 面板「开始编辑」后才允许画

    const std::vector<DraftStroke>& strokes() const { return strokes_; }
    bool hasActive() const { return active_; }
    const DraftStroke& active() const { return current_; }

    void beginStroke(double x, double y);
    // 自由绘制：追加采样点；直线 / 矩形：更新终点（直线按住 Shift 时吸附到 45° 的整数倍）
    void moveStroke(double x, double y, bool snap45);
    void endStroke();
    void undo();
    void clear();

Q_SIGNALS:
    void changed();

private:
    GridEditDraft() = default;

    std::vector<DraftStroke> strokes_;
    DraftStroke current_;
    bool active_ = false;
};

}  // namespace map_manager_rviz_plugin

#endif  // MAP_MANAGER_RVIZ_PLUGIN__GRID_EDIT_DRAFT_HPP_

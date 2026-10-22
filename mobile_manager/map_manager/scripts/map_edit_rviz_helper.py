#!/usr/bin/env python3
"""在 RViz 上手动试底图编辑的辅助节点（开发调试用，非产品前端）。

用 RViz 自带的「Publish Point」工具在地图上点点，点连成笔画；调服务保存 / 回滚。
保存走 map_manager 的 map_command OP_EDIT_GRID，与 X-Wiz 前端走的是同一个接口。

  ros2 run map_manager map_edit_rviz_helper.py
  # 可选参数：--ros-args -p tool:=wall|erase|unknown -p width_m:=0.2 -p map_name:=<图名，空=当前图>

RViz 里：Fixed Frame = map；加 Map 显示 /map（Durability = Transient Local）；
加 MarkerArray 显示 /map_edit_helper/preview（待保存笔画预览）；工具栏选「Publish Point」逐点点击。

服务（std_srvs/srv/Trigger）：
  /map_edit_helper/end_stroke     结束当前一笔，下一次点击开始新的一笔（换工具 / 宽度前也先调它）
  /map_edit_helper/undo           撤销最后一个点（当前笔为空时撤销上一笔）
  /map_edit_helper/clear          清空所有未保存笔画
  /map_edit_helper/save           一次提交全部笔画；编辑的是当前图时自动重新加载，RViz 的 /map 随即更新
  /map_edit_helper/rollback_last  回滚到最近一次保存之前的版本
换工具：ros2 param set /map_edit_helper tool erase

注意：自动重新加载等同 map_command OP_LOAD，在整机运行时会触发定位重新加载，只在测试环境用。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_interfaces.msg import GridStroke, MapState
from nav_interfaces.srv import GetMaps, MapManagerCommand, RollbackMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

TOOLS = {"wall": GridStroke.TOOL_WALL, "erase": GridStroke.TOOL_ERASE, "unknown": GridStroke.TOOL_UNKNOWN}
COLORS = {"wall": (0.9, 0.1, 0.1), "erase": (0.1, 0.8, 0.2), "unknown": (0.5, 0.5, 0.5)}


class MapEditRvizHelper(Node):
    def __init__(self):
        super().__init__("map_edit_helper")
        self.declare_parameter("tool", "wall")
        self.declare_parameter("width_m", 0.2)
        self.declare_parameter("map_name", "")
        self.declare_parameter("auto_reload", True)
        # 部署 launch 里 command_board 的节点名是 command_board；单独 ros2 run 起的默认名是 command_board_manager
        self.declare_parameter("rollback_service", "/command_board/rollback_map")

        self.strokes = []      # 已结束的笔画：(tool_name, width, [(x, y)])
        self.current = []      # 正在画的一笔的点
        self.current_map = ""
        self.last_save = None  # (map_name, backup_ts)

        cbg = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointStamped, "/clicked_point", self.on_click, 10, callback_group=cbg)
        self.create_subscription(MapState, "/map_manager/map_state", self.on_map_state, latched,
                                 callback_group=cbg)
        self.preview_pub = self.create_publisher(MarkerArray, "~/preview", latched)
        self.cmd = self.create_client(MapManagerCommand, "/map_manager/map_command", callback_group=cbg)
        self.get_maps = self.create_client(GetMaps, "/map_manager/get_maps", callback_group=cbg)
        self.rollback = self.create_client(RollbackMap, self.get_parameter("rollback_service").value,
                                           callback_group=cbg)
        for name, cb in (("end_stroke", self.srv_end), ("undo", self.srv_undo), ("clear", self.srv_clear),
                         ("save", self.srv_save), ("rollback_last", self.srv_rollback)):
            self.create_service(Trigger, "~/" + name, cb, callback_group=cbg)
        self.get_logger().info("Ready. Use RViz 'Publish Point' to click points; "
                               "call ~/save to commit, ~/end_stroke to start a new stroke.")

    # ── 输入 ──────────────────────────────────────────────────────────
    def tool_name(self):
        t = self.get_parameter("tool").value
        return t if t in TOOLS else "wall"

    def target_map(self):
        return self.get_parameter("map_name").value or self.current_map

    def on_map_state(self, msg):
        self.current_map = msg.map_name

    def on_click(self, msg):
        if msg.header.frame_id not in ("", "map"):
            self.get_logger().warn(f"clicked point in frame '{msg.header.frame_id}', expected 'map'; ignored")
            return
        self.current.append((msg.point.x, msg.point.y))
        self.publish_preview()

    def finish_current(self):
        if self.current:
            self.strokes.append((self.tool_name(), float(self.get_parameter("width_m").value), self.current))
            self.current = []

    # ── 服务 ──────────────────────────────────────────────────────────
    def srv_end(self, _req, res):
        self.finish_current()
        self.publish_preview()
        res.success, res.message = True, f"{len(self.strokes)} strokes pending"
        return res

    def srv_undo(self, _req, res):
        if self.current:
            self.current.pop()
        elif self.strokes:
            self.strokes.pop()
        self.publish_preview()
        res.success, res.message = True, f"{len(self.strokes)} strokes + {len(self.current)} points pending"
        return res

    def srv_clear(self, _req, res):
        self.strokes, self.current = [], []
        self.publish_preview()
        res.success, res.message = True, "cleared"
        return res

    def srv_save(self, _req, res):
        self.finish_current()
        name = self.target_map()
        if not name or not self.strokes:
            res.success, res.message = False, "no target map or no strokes"
            return res
        revision = ""
        for m in self.call(self.get_maps, GetMaps.Request()).maps:
            if m.name == name:
                revision = m.grid_revision
        req = MapManagerCommand.Request(op=MapManagerCommand.Request.OP_EDIT_GRID, map_name=name,
                                        expected_revision=revision)
        for tool, width, pts in self.strokes:
            s = GridStroke(tool=TOOLS[tool], shape=GridStroke.SHAPE_POLYLINE, width_m=width)
            s.points = [Point(x=x, y=y) for x, y in pts]
            req.strokes.append(s)
        r = self.call(self.cmd, req)
        res.success, res.message = r.success, f"[{r.error_code}] {r.message}"
        if r.success:
            self.strokes = []
            if r.backup_ts:
                self.last_save = (name, r.backup_ts)
            if r.requires_reload and self.get_parameter("auto_reload").value:
                lr = self.call(self.cmd, MapManagerCommand.Request(op=MapManagerCommand.Request.OP_LOAD,
                                                                   map_name=name))
                res.message += f"; reload: {lr.message}"
        self.publish_preview()
        self.get_logger().info(f"save: {res.message}")
        return res

    def srv_rollback(self, _req, res):
        if not self.last_save:
            res.success, res.message = False, "nothing saved in this session"
            return res
        name, ts = self.last_save
        req = RollbackMap.Request(map_version_ts=ts)
        if name != self.current_map:
            req.map_name = name  # 当前图不传 map_name：command_board 恢复后自己重新下发
        r = self.call(self.rollback, req)
        res.success, res.message = r.success, r.message
        if r.success:
            self.last_save = None
        self.get_logger().info(f"rollback: {r.message}")
        return res

    # ── 工具 ──────────────────────────────────────────────────────────
    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service not available: {client.srv_name}")
        return client.call(req)

    def publish_preview(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        items = self.strokes + ([(self.tool_name(), float(self.get_parameter("width_m").value), self.current)]
                                if self.current else [])
        for i, (tool, width, pts) in enumerate(items):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = "strokes", i
            m.type = Marker.LINE_STRIP if len(pts) > 1 else Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = max(width, 0.02)
            r, g, b = COLORS[tool]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.6
            m.pose.orientation.w = 1.0
            if len(pts) > 1:
                m.points = [Point(x=x, y=y, z=0.05) for x, y in pts]
            else:
                m.pose.position = Point(x=pts[0][0], y=pts[0][1], z=0.05)
            arr.markers.append(m)
        self.preview_pub.publish(arr)


def main():
    rclpy.init()
    node = MapEditRvizHelper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

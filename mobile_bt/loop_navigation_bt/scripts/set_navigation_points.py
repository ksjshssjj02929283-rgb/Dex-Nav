#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import json

import os
import curses
import time
from datetime import datetime

WAYPOINT_SAVE_PATH = "/home/dexforce/w1/dexe_mobile_application/mobile_bt/loop_navigation_bt/config/navigation_points.json"
Charging_POINT_SAVE_PATH = "/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json"

log_messages = []  # 存储日志消息


def add_log(message):
    global log_messages
    """添加日志消息"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    log_messages.append(f"[{timestamp}] {message}")
    # 限制日志数量，只保留最新的50条
    if len(log_messages) > 50:
        log_messages = log_messages[-50:]


class TfEcho(Node):

    def __init__(self):
        super().__init__("tf_echo")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.current_pose = None

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link",
                                                rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation

            self.current_pose = {
                "x": round(trans.x, 3),
                "y": round(trans.y, 3),
                "z": round(trans.z, 3),
                "qx": round(rot.x, 3),
                "qy": round(rot.y, 3),
                "qz": round(rot.z, 3),
                "qw": round(rot.w, 3),
            }
        except Exception as e:
            add_log(f"Could not get transform: {e}")

    def get_current_pose(self):
        """获取当前位姿"""
        return self.current_pose


# ======================
# 主 UI
# ======================
def main(stdscr):
    # 初始化颜色
    curses.start_color()
    curses.use_default_colors()

    # 定义颜色对 - Ubuntu 终端类似的颜色
    # 使用标准颜色定义，接近Ubuntu终端的颜色
    curses.init_pair(1, curses.COLOR_WHITE, -1)  # 白色文字，透明背景
    curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLACK)  # 白色文字，黑色背景
    curses.init_pair(3, 8, -1)  # 使用灰色文字，透明背景 (8是灰色)

    # 使用默认背景色，不强制设置背景
    stdscr.bkgd(' ', curses.color_pair(1))  # 使用白色文字，透明背景

    rclpy.init()
    tf_echo_node = TfEcho()

    curses.curs_set(0)  # 隐藏光标
    stdscr.nodelay(True)  # 非阻塞输入
    stdscr.timeout(100)  # 100ms 刷新

    pose = None
    waypoints = []
    chargingpoints = []

    while True:
        # ---- 获取 pose ----
        current_pose = tf_echo_node.get_current_pose()
        if current_pose is not None:
            pose = current_pose

        # ---- 清屏 ----
        stdscr.erase()

        # ---- Pose 面板 ----
        stdscr.addstr(1, 2, "Current Pose", curses.A_BOLD)
        if pose is not None:
            stdscr.addstr(2, 4, f"x: {pose['x']:.3f}")
            stdscr.addstr(3, 4, f"y: {pose['y']:.3f}")
            stdscr.addstr(4, 4, f"z: {pose['z']:.3f}")

            stdscr.addstr(6, 4, f"qx: {pose['qx']:.3f}")
            stdscr.addstr(7, 4, f"qy: {pose['qy']:.3f}")
            stdscr.addstr(8, 4, f"qz: {pose['qz']:.3f}")
            stdscr.addstr(9, 4, f"qw: {pose['qw']:.3f}")

        # ---- Waypoints 面板 ----
        stdscr.addstr(1, 30, "Waypoints", curses.A_BOLD)
        for i, wp in enumerate(waypoints[-5:], 1):
            stdscr.addstr(
                1 + i,
                30,
                f"{i}. {wp['name']}: ({wp['pose']['x']:.3f}, {wp['pose']['y']:.3f}, {wp['pose']['z']:.3f})",
            )

        # ---- Charging Points 面板 ----
        stdscr.addstr(1, 70, "Charging Points", curses.A_BOLD)
        # 显示最后一个充电点
        if chargingpoints:
            last_cp = chargingpoints[-1]  # 获取最后一个充电点
            stdscr.addstr(
                2,
                70,
                f"Charging: ({last_cp['pose']['x']:.3f}, {last_cp['pose']['y']:.3f}, {last_cp['pose']['z']:.3f})",
            )

        # ---- Help ----
        stdscr.addstr(
            10,
            2,
            "[w]Add WayPoint [c] Add Charging Point[s] Save pose  [q] Quit",
            curses.A_DIM,
        )

        # ---- 刷新 ----
        stdscr.refresh()

        # ---- 日志面板 ----
        stdscr.addstr(12, 2, "Logs", curses.A_BOLD)
        # 计算可用于显示日志的行数
        log_display_count = min(10, len(log_messages))
        start_line = max(0, len(log_messages) - log_display_count)  # 从底部开始显示

        for i in range(log_display_count):
            if start_line + i < len(log_messages):
                log_msg = log_messages[start_line + i]
                max_y, max_x = stdscr.getmaxyx()
                try:
                    stdscr.addstr(13 + i, 4, log_msg[:max_x - 6])  # 限制长度以避免超出屏幕
                except:
                    break  # 防止超出屏幕边界

        # ---- 键盘输入 ----
        key = stdscr.getch()
        if key == ord("w"):
            if pose is None:
                add_log("No pose available")
                continue
            waypoints.append({
                "name": f"wp_{len(waypoints)+1}",
                "pose": pose.copy(),
                "time": datetime.now().strftime("%H:%M:%S"),
            })
            add_log(f"Added waypoint: {waypoints[-1]}")

        elif key == ord("c"):
            if pose is None:
                add_log("No pose available")
                continue
            chargingpoints.append({
                "name": f"cp_{len(chargingpoints)+1}",
                "pose": pose.copy(),
                "time": datetime.now().strftime("%H:%M:%S"),
            })
            add_log(f"Added charging point: {chargingpoints[-1]}")

        elif key == ord("s"):
            # 按照指定的JSON格式保存航点
            waypoint_data = {"frame_id": "map", "points": []}

            for wp in waypoints:
                # 创建航点数据
                waypoint_point = {
                    "position": {
                        "x": wp["pose"]["x"],
                        "y": wp["pose"]["y"],
                        "z": wp["pose"]["z"],
                    },
                    "orientation": {
                        "x": wp["pose"].get("qx", 0.0),
                        "y": wp["pose"].get("qy", 0.0),
                        "z": wp["pose"].get("qz", 0.0),
                        "w": wp["pose"].get("qw", 1.0),
                    },
                }
                waypoint_data["points"].append(waypoint_point)

                os.makedirs(os.path.dirname(WAYPOINT_SAVE_PATH), exist_ok=True)

            # 保存到JSON文件
            with open(WAYPOINT_SAVE_PATH, "w") as f:
                json.dump(waypoint_data, f, indent=2)

            add_log(f"Saved {waypoint_data} waypoints to {WAYPOINT_SAVE_PATH}")

            # 保存充电点
            if chargingpoints:
                # 只保存最后一个充电点（限制只能有一个）
                cp = chargingpoints[-1]  # 获取最后一个充电点
                charging_data = {
                    "frame_id": "map",
                    "position": {
                        "x": cp["pose"]["x"],
                        "y": cp["pose"]["y"],
                        "z": cp["pose"]["z"],
                    },
                    "orientation": {
                        "x": cp["pose"].get("qx", 0.0),
                        "y": cp["pose"].get("qy", 0.0),
                        "z": cp["pose"].get("qz", 0.0),
                        "w": cp["pose"].get("qw", 1.0),
                    },
                }

                os.makedirs(os.path.dirname(Charging_POINT_SAVE_PATH),
                            exist_ok=True)

                with open(Charging_POINT_SAVE_PATH, "w") as f:
                    json.dump(charging_data, f, indent=2)

                add_log(
                    f"Saved {charging_data} charging points to {Charging_POINT_SAVE_PATH}"
                )
            else:
                # 清空充电点文件
                os.makedirs(os.path.dirname(Charging_POINT_SAVE_PATH),
                            exist_ok=True)
                with open(Charging_POINT_SAVE_PATH, "w") as f:
                    json.dump({}, f, indent=2)

                add_log("Charging points file cleared")
        elif key == ord("q"):
            add_log("Exiting...")
            break

        rclpy.spin_once(tf_echo_node, timeout_sec=0.01)

        time.sleep(0.1)

    # 清理ROS2资源
    tf_echo_node.destroy_node()
    rclpy.shutdown()


# ======================
# 入口
# ======================
if __name__ == "__main__":
    curses.wrapper(main)

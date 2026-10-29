"""nbt env：每轮测试开头采一次版本、环境和关键参数，生成通用约定 3.1「版本与环境」表的自动部分。

采集内容：
  - 机器：主机名、内核、DEXE_* 环境变量、已安装的 dexe 软件包版本（dpkg）
  - 导航状态：当前后端（/command_board/navigation_backend）、当前地图、定位状态
  - 关键参数：各 SOP「测前读参数」里列的参数（节点不在线就标出来，不报错退出）
  - 关键话题：是否有发布者、类型
输出：<out>/env.md（可直接贴进记录）、<out>/env.json。
"""
import argparse
import json
import os
import platform
import socket
import subprocess
import time

from nav_benchmark_tools import common

# (节点, 参数, 用在哪些用例)
PARAMS = [
    ("/planner_server", "current_planner", "G-FREE、G-ROUTE"),
    ("/planner_server", "override_planner_id", "G-FREE、G-ROUTE"),
    ("/planner_server", "route_fallback_planner", "G-FREE、G-ROUTE"),
    ("/planner_server", "RouteGridPlanner.enable_route_costmap_avoidance", "G-ROUTE"),
    ("/planner_server", "RouteGridPlanner.projection_edge_threshold_m", "G-ROUTE"),
    ("/controller_server", "current_controller", "G-FREE、G-ROUTE"),
    ("/controller_server", "current_goal_checker", "G-FREE、G-DOCK"),
    ("/controller_server", "current_speed_planner", "G-MOT、G-STOP"),
    ("/controller_server", "ReachabilitySpeedPlanner.max_speed", "G-MOT-02"),
    ("/controller_server", "ReachabilitySpeedPlanner.accel", "G-MOT-01"),
    ("/controller_server", "ReachabilitySpeedPlanner.decel", "G-MOT-01"),
    ("/controller_server", "high_precision_goal_checker.lat_goal_tolerance", "G-DOCK"),
    ("/controller_server", "high_precision_goal_checker.lon_goal_tolerance", "G-DOCK"),
    ("/controller_server", "high_precision_goal_checker.yaw_goal_tolerance", "G-DOCK"),
    ("/velocity_smoother", "max_velocity", "G-MOT"),
    ("/velocity_smoother", "max_accel", "G-MOT"),
    ("/velocity_smoother", "max_decel", "G-MOT-04"),
    ("/velocity_smoother", "hard_stop_duration", "G-MOT-04"),
    ("/global_costmap/global_costmap", "filters", "G-FREE-02"),
    ("/local_costmap/local_costmap", "footprint", "G-FREE-03、G-ROUTE-05"),
    ("/navigation", "preempt_on_new_goal", "G-SYS-03"),
    ("/navigation", "blocked_hold_sec", "G-STOP-05"),
    ("/navigation", "progress_timeout_sec", "G-STOP-05"),
]

TOPICS = [
    "/localization/state", "/localization/loc_pose", "/mapping/state", "/navigation/state",
    "/command_board/nav_system_status", "/command_board/navigation_backend", "/diagnostics_agg",
    "/feedback/odom", "/laser_scan/front", "/laser_scan/rear", "/orbbec/front/laser_scan",
    "/orbbec/rear/laser_scan", "/imu/chassis", "/map", "/tf", "/plan", "/cmd_vel_nav", "/control/cmd_vel_nav",
    "/control/cmd_vel", "/cmd_vel_hard_stop", "/obstacle_status", "/tracking_path",
]


def run(args, timeout=5):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return ""


def param_text(v):
    """rcl_interfaces/ParameterValue → 文本。"""
    t = v.type
    if t == 1:
        return str(v.bool_value).lower()
    if t == 2:
        return str(v.integer_value)
    if t == 3:
        return f"{v.double_value:g}"
    if t == 4:
        return v.string_value
    if t in (6, 7, 8, 9):
        arr = {6: v.bool_array_value, 7: v.integer_array_value, 8: v.double_array_value, 9: v.string_array_value}[t]
        return "[" + ", ".join(str(x) for x in arr) + "]"
    return "（未设置）"


def collect_ros(timeout):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("nbt_env")
    out = {"params": [], "topics": [], "backend": None, "map": None, "localization": None}
    try:
        end = time.time() + 1.5          # 等发现
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        types = dict(node.get_topic_names_and_types())
        for t in TOPICS:
            pubs = node.count_publishers(t)
            out["topics"].append({"topic": t, "type": ",".join(types.get(t, [])) or "—", "publishers": pubs})

        latest = {}

        def keep(name):
            def cb(m):
                latest[name] = m
            return cb
        node.create_subscription(String, "/command_board/navigation_backend", keep("backend"),
                                 common.qos_for(node, "/command_board/navigation_backend", 1))
        for name, topic, tstr in (("loc", "/localization/state", "nav_interfaces/msg/LocalizationState"),
                                  ("sys", "/command_board/nav_system_status", "nav_interfaces/msg/NavSystemStatus")):
            cls = common.import_msg(tstr)
            if cls:
                node.create_subscription(cls, topic, keep(name), common.qos_for(node, topic))
        end = time.time() + min(timeout, 3.0)
        while time.time() < end and len(latest) < 3:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "backend" in latest:
            out["backend"] = latest["backend"].data
        if "sys" in latest:
            out["map"] = latest["sys"].current_map_name
        if "loc" in latest:
            m = latest["loc"]
            cls = type(m)
            out["localization"] = (f"status {m.status}（{common.constant_name(cls, lambda k: k.startswith('LOC_'), m.status)}）"
                                   f"，match_level {m.match_level}，score {m.score:.2f}")

        by_node = {}
        for n, p, use in PARAMS:
            by_node.setdefault(n, []).append((p, use))
        for n, plist in by_node.items():
            cli = node.create_client(GetParameters, f"{n}/get_parameters")
            if not cli.wait_for_service(timeout_sec=timeout):
                for p, use in plist:
                    out["params"].append({"node": n, "param": p, "value": "节点不在线", "used_by": use})
                continue
            def query(names):
                fut = cli.call_async(GetParameters.Request(names=names))
                rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
                return fut.result().values if fut.done() and fut.result() else None

            names = [p for p, _ in plist]
            vals = query(names)
            if vals is not None and len(vals) == len(names):
                texts = [param_text(v) for v in vals]
            else:
                # 一批里只要有一个参数没声明，服务端就整批返回空，这时逐个查，区分「未声明」和「超时」
                texts = []
                for p in names:
                    v = query([p])
                    texts.append("读取超时" if v is None else (param_text(v[0]) if v else "未声明"))
            for (p, use), text in zip(plist, texts):
                out["params"].append({"node": n, "param": p, "used_by": use, "value": text})
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt env", description="采集版本、环境和关键参数（通用约定 3.1）")
    ap.add_argument("-o", "--out", default="", help="输出目录，默认 ~/nav_eval/env_<时间>")
    ap.add_argument("--label", default="", help="测试轮次 / 用例说明，写进表头")
    ap.add_argument("--timeout", type=float, default=2.0, help="每个节点的参数服务等待秒数")
    a = ap.parse_args(argv)
    a.out = a.out or common.default_out("env")
    os.makedirs(a.out, exist_ok=True)

    machine = {
        "time": common.now_text(), "hostname": socket.gethostname(), "kernel": platform.release(),
        "arch": platform.machine(),
        "dexe_env": {k: v for k, v in os.environ.items() if k.startswith("DEXE_")},
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "dexe_debs": run(["dpkg-query", "-W", "-f", "${Package} ${Version}\n", "dexe*"]),
    }
    ros = collect_ros(a.timeout)

    L = [f"# 版本与环境{('：' + a.label) if a.label else ''}", "", f"采集时间：{machine['time']}", "",
         "## 自动采集", "",
         common.md_table(["项", "值"], [
             ["主机名 / 架构 / 内核", f"{machine['hostname']} / {machine['arch']} / {machine['kernel']}"],
             ["DEXE 环境变量", "，".join(f"{k}={v}" for k, v in sorted(machine["dexe_env"].items())) or "—"],
             ["ROS 发行版", machine["ros_distro"] or "—"],
             ["导航后端", ros["backend"] or "未读到（/command_board/navigation_backend 无发布）"],
             ["当前地图", ros["map"] or "未读到"],
             ["定位状态", ros["localization"] or "未读到"],
         ]), "",
         "已安装的 dexe 软件包：", "", "```", machine["dexe_debs"] or "（dpkg 里没有 dexe 开头的包）", "```", "",
         "## 需要手填（通用约定 3.1）", "",
         common.md_table(["项", "填写"], [["测试轮次 / 日期", ""], ["机器 SN / 机型", ""], ["NX 镜像版本 / XWiz 版本", ""],
                                         ["相对出厂的配置改动", "（没有写「无」）"]]), "",
         "## 关键参数", "",
         common.md_table(["节点", "参数", "当前值", "用在"], [[p["node"], p["param"], p["value"], p["used_by"]] for p in ros["params"]]),
         "", "## 关键话题", "",
         common.md_table(["话题", "类型", "发布者数"], [[t["topic"], t["type"], t["publishers"]] for t in ros["topics"]])]
    with open(os.path.join(a.out, "env.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.join(a.out, "env.json"), "w") as f:
        json.dump({"label": a.label, "machine": machine, "ros": ros}, f, ensure_ascii=False, indent=2)
    print("\n".join(L))
    print(f"\n已保存：{a.out}/env.md、env.json")


if __name__ == "__main__":
    main()

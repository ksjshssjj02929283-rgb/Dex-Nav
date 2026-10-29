"""nbt record：按用例录包。默认话题 = 通用约定 5.1，再按用例族追加话题。

  nbt record --case G-ROUTE-03            # 默认 + 路网相关话题，输出 ~/nav_eval/bags/G-ROUTE-03_<时间>
  nbt record --case S-TRAY --split-min 5  # 长跑按 5 min 分段
  nbt record --list                       # 看各用例族追加了哪些话题
  nbt record --case G-STOP --dry-run      # 只打印命令和缺失话题，不录
  nbt record --case SPEED                 # 能力摸底速度测试（nbt pathnav run --record 自动用它）
录之前会检查话题是否有发布者，没有的列出来提醒（仍会录，话题上线后 rosbag2 自动订阅）。
"""
import argparse
import os
import shlex
import subprocess
import sys
import time

BASE = [
    # 传感器
    "/laser_scan/front", "/laser_scan/rear", "/imu/chassis", "/feedback/odom",
    "/orbbec/front/laser_scan", "/orbbec/rear/laser_scan",
    # 建图 / 定位
    "/mapping/state", "/localization/state", "/localization/loc_pose", "/tf", "/tf_static", "/map",
    # 导航
    "/plan", "/cmd_vel_nav", "/control/cmd_vel_nav", "/control/cmd_vel", "/cmd_vel_hard_stop", "/navigation/state",
    # 系统
    "/command_board/nav_system_status", "/diagnostics_agg",
]

# 用例族 → (追加话题, 是否需要隐藏话题, 说明)
PRESETS = {
    "G-MAP": ([], False, "建图：默认话题已含 /mapping/state、/map"),
    "G-LOC": (["/initialpose"], False, "定位：默认话题 + 手动给初值"),
    "G-FREE": (["/navigation/global_path", "/global_costmap/costmap", "/local_costmap/costmap",
                "/keepout_filter_mask", "/speed_limit"], False, "自由导航：全局路径、代价地图、禁行区掩码"),
    "G-ROUTE": (["/navigation/global_path", "/planner_server/RouteGridPlanner/route_plan",
                 "/map_asset_manager/assets", "/map_asset_manager/route_graph_markers", "/speed_limit"],
                False, "路网导航（nav2 方案）：路网规划结果、资产"),
    "G-DOCK": (["/navigation/global_path", "/local_plan"], False, "到点精度"),
    "G-MOT": (["/speed_limit", "/navigation/speed_plan_diagnostics", "/local_plan", "/speed_filter_mask"], False,
              "运动控制：限速、速度规划诊断、限速区掩码"),
    "G-STOP": (["/obstacle_status", "/tracking_path", "/local_costmap/costmap", "/local_plan"], False,
               "停障 / 紧急避障：障碍状态、局部代价地图"),
    "G-CHG": (["/command_board/navigation_backend", "/auto_charging/_action/feedback",
               "/auto_charging/_action/status"], True, "回充：action 反馈是隐藏话题，需 --include-hidden-topics"),
    "G-STB": (["/navigation/global_path", "/obstacle_status", "/tracking_path"], False, "长稳：建议 --split-min 5"),
    "G-SYS": (["/command_board/safety_alarm", "/command_board/navigation_backend"], False, "状态与部署"),
    "S-TRAY": (["/tracking_path", "/navigation_status", "/obstacle_status",
                "/map_asset_manager/route_graph_markers", "/command_board/navigation_backend"], False,
               "蓝思 tray（path_navigation 方案）"),
    "SPEED": (["/cmd_vel_nav_path", "/tracking_path", "/navigation_status", "/obstacle_status",
               "/path_follower_node/debug/reference_path", "/navigate_through_graph_points/_action/status",
               "/navigate_through_graph_points/_action/feedback", "/feedback/joint",
               "/command_board/navigation_backend"], True,
              "能力摸底速度测试（path_navigation）：四级速度、路网任务边界、参考路径、关节；action 话题是隐藏话题"),
}


def family_of(case):
    """'G-ROUTE-03' → 'G-ROUTE'；'S-TRAY-01' → 'S-TRAY'；已是族名原样返回。"""
    case = case.upper()
    if case in PRESETS:
        return case
    parts = case.split("-")
    return "-".join(parts[:2]) if len(parts) >= 2 else case


def topics_for(case, extra=()):
    fam = family_of(case) if case else ""
    add, hidden, _ = PRESETS.get(fam, ([], False, ""))
    seen, out = set(), []
    for t in BASE + list(add) + list(extra):
        if t and t not in seen:
            seen.add(t)
            out.append(t)
    return out, hidden, fam


def missing_topics(topics, wait=1.5):
    """返回当前没有发布者的话题；没有 ROS 环境时返回 None。"""
    try:
        import rclpy
    except ImportError:
        return None
    rclpy.init()
    node = rclpy.create_node("nbt_record_probe")
    try:
        end = time.time() + wait
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return [t for t in topics if node.count_publishers(t) == 0]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt record", description="按用例录包（通用约定 5.1 默认话题 + 用例族追加）")
    ap.add_argument("--case", default="", help="用例 ID 或用例族，如 G-ROUTE-03、G-STOP、S-TRAY-01")
    ap.add_argument("--extra", nargs="*", default=[], help="额外话题")
    ap.add_argument("-o", "--out", default="", help="包输出路径，默认 ~/nav_eval/bags/<用例>_<时间>")
    ap.add_argument("--split-min", type=float, default=0, help="按分钟分段（长跑用），0 = 不分段")
    ap.add_argument("--dry-run", action="store_true", help="只打印命令和缺失话题")
    ap.add_argument("--list", action="store_true", help="列出各用例族的追加话题")
    a = ap.parse_args(argv)

    if a.list:
        print("默认话题（通用约定 5.1）：\n  " + "\n  ".join(BASE))
        for fam, (add, hidden, note) in PRESETS.items():
            print(f"\n{fam}：{note}" + ("（含隐藏话题）" if hidden else ""))
            for t in add:
                print(f"  + {t}")
        return 0

    topics, hidden, fam = topics_for(a.case, a.extra)
    if a.case and fam not in PRESETS:
        print(f"注意：{a.case} 不属于已知用例族（{', '.join(PRESETS)}），只录默认话题 + --extra", file=sys.stderr)
    tag = a.case.upper() if a.case else "bag"
    out = a.out or os.path.expanduser(time.strftime(f"~/nav_eval/bags/{tag}_%Y%m%d_%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    cmd = ["ros2", "bag", "record", "-o", out]
    if a.split_min > 0:
        cmd += ["--max-bag-duration", str(int(a.split_min * 60))]
    if hidden:
        cmd.append("--include-hidden-topics")
    cmd += topics

    miss = missing_topics(topics)
    if miss:
        print(f"以下 {len(miss)} 个话题当前没有发布者（仍会录，上线后自动订阅）：")
        for t in miss:
            print(f"  - {t}")
    print("命令：" + " ".join(shlex.quote(c) for c in cmd))
    if a.dry_run:
        return 0
    print(f"录包中，Ctrl-C 结束。输出：{out}", flush=True)
    p = subprocess.Popen(cmd)
    while True:
        try:
            return p.wait()
        except KeyboardInterrupt:
            continue        # 子进程同一进程组也收到 SIGINT，等它自己写完 metadata 再退，不能 kill


if __name__ == "__main__":
    sys.exit(main())

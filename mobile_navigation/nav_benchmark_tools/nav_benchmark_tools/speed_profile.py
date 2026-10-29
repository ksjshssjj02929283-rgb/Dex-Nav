"""nbt speed profile：路网链路（path_navigation）的速度档：看、改、恢复。

  nbt speed profile show                                 # 跟踪器 / 平滑器的速度与加减速参数
  nbt speed profile set 0.75 --batch E075                # 跟踪器速度上限改为 0.75（平滑器前进上限须 ≥ 0.75）
  nbt speed profile set 1.0 --batch E100 --with-smoother # 直线最大速度：跟踪器与平滑器前进上限一起改
  nbt speed profile restore --batch E075                 # 恢复到该批次第一次改档前的基线

改档门禁（任一不满足就拒绝，--force 跳过）：没有进行中的路网任务、车已停稳、平滑器前进上限 ≥ 档位（或带 --with-smoother）。
第一次 set 时把改动前的参数存成 <批次目录>/profile/original.json（之后不覆盖），每次 set / restore 后读回比对并存档。
只改速度上限；加减速度、jerk、倒车速度（min_velocity）不动。改完要重新下发任务，正在执行的路线不按新档位。
"""
import argparse
import json
import os
import subprocess
import sys
import time

from nav_benchmark_tools import common, params
from nav_benchmark_tools import navlog as nl


def busy_and_still(node, still_v, hold, timeout=5.0):
    """返回 (有进行中的任务, 停稳, 说明)。"""
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latest, odom = {}, []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(GoalStatusArray, nl.STATUS_TOPIC, lambda m: latest.update(s=m), qos)
    node.create_subscription(Odometry, nl.ODOM, lambda m: odom.append(
        (time.time(), nl.signed_speed(m.twist.twist.linear.x, m.twist.twist.linear.y), m.twist.twist.angular.z)), 50)
    end, still_since = time.time() + timeout, None
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        recent = [o for o in odom if o[0] > time.time() - 0.3]
        if recent and all(abs(v) < still_v for _, v, _ in recent):
            still_since = still_since or time.time()
            if time.time() - still_since >= hold and time.time() > end - timeout + 1.0:
                break
        else:
            still_since = None
    active = [s for s in latest["s"].status_list if s.status in (1, 2, 3)] if "s" in latest else []
    still = still_since is not None and time.time() - still_since >= hold
    why = []
    if active:
        why.append(f"有 {len(active)} 个进行中的路网任务")
    if not odom:
        why.append(f"没收到 {nl.ODOM}")
    elif not still:
        why.append("车没停稳")
    return bool(active), still, "；".join(why)


def dump_yaml(node_name, path):
    try:
        out = subprocess.run(["ros2", "param", "dump", node_name], capture_output=True, text=True, timeout=15)
    except (subprocess.SubprocessError, OSError):
        return False
    if out.returncode != 0 or not out.stdout.strip():
        return False
    with open(path, "w") as f:
        f.write(out.stdout)
    return True


def show_rows(snap):
    rows = []
    for n, names in params.SPEED_PARAMS.items():
        vals = snap.get(n)
        for p in names:
            rows.append([n, p, "节点不在线" if vals is None else vals.get(p)])
    return rows


def save(pdir, tag, snap, extra=None):
    os.makedirs(pdir, exist_ok=True)
    path = os.path.join(pdir, f"{time.strftime('%Y%m%d_%H%M%S')}_{tag}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"time": common.now_text(), "tag": tag, "params": snap, **(extra or {})}, f, ensure_ascii=False,
                  indent=2)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt speed profile", description="路网链路速度档：看 / 改 / 恢复")
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("show")
    for name in ("set", "restore"):
        p = sub.add_parser(name)
        if name == "set":
            p.add_argument("speed", type=float, help="跟踪器速度上限 m/s")
            p.add_argument("--with-smoother", action="store_true", help="平滑器前进上限（max_velocity 第一个数）一起改")
        p.add_argument("--batch", required=True, help="批次号，参数存档放 <批次目录>/profile/")
        p.add_argument("-o", "--out", default="", help="批次目录，默认 ~/nbt_eval/capability/<批次号>")
        p.add_argument("--force", action="store_true", help="跳过「无任务 + 停稳」门禁（仅调试用）")
    a = ap.parse_args(argv)

    import rclpy
    rclpy.init()
    node = rclpy.create_node("nbt_speed_profile")
    try:
        snap = params.snapshot(node)
        if a.op == "show":
            print(common.md_table(["节点", "参数", "值"], show_rows(snap)))
            return 0 if all(v is not None for v in snap.values()) else 2
        if snap.get(params.FOLLOWER) is None or snap.get(params.SMOOTHER) is None:
            print(f"✗ {params.FOLLOWER} 或 {params.SMOOTHER} 不在线（导航后端不是 path_navigation？）")
            return 2
        bdir = os.path.abspath(os.path.expanduser(a.out or f"~/nbt_eval/capability/{a.batch}"))
        pdir = os.path.join(bdir, "profile")
        orig_path = os.path.join(pdir, "original.json")

        if not a.force:
            active, still, why = busy_and_still(node, nl.DEFAULTS["still_v"], nl.DEFAULTS["still_hold"])
            if active or not still:
                print(f"✗ 不能改档：{why}。先 nbt pathnav cancel 并等车停稳")
                return 2

        if a.op == "set":
            smax = list(snap[params.SMOOTHER]["max_velocity"] or [])
            if not os.path.exists(orig_path):
                os.makedirs(pdir, exist_ok=True)
                with open(orig_path, "w", encoding="utf-8") as f:
                    json.dump({"time": common.now_text(), "params": snap}, f, ensure_ascii=False, indent=2)
                for n, fn in ((params.FOLLOWER, "original_follower.yaml"),
                              (params.SMOOTHER, "original_smoother.yaml")):
                    if not dump_yaml(n, os.path.join(pdir, fn)):
                        print(f"  ⚠ ros2 param dump {n} 失败，只存了 original.json")
                print(f"  基线已存 {orig_path}")
            changes = {params.FOLLOWER: {"max_linear_velocity": float(a.speed)}}
            if a.with_smoother:
                if not smax:
                    print("✗ 读不到平滑器 max_velocity")
                    return 2
                changes[params.SMOOTHER] = {"max_velocity": [float(a.speed)] + [float(x) for x in smax[1:]]}
            elif not smax or smax[0] < a.speed - 1e-9:
                print(f"✗ 平滑器前进上限 {smax[0] if smax else '?'} < {a.speed}：只改跟踪器跑不到本档；"
                      f"确认底盘许可后加 --with-smoother")
                return 2
        else:
            if not os.path.exists(orig_path):
                print(f"✗ 没有基线 {orig_path}（这个批次没用 nbt speed profile set 改过档）")
                return 2
            with open(orig_path, encoding="utf-8") as f:
                base = json.load(f)["params"]
            changes = {params.FOLLOWER: {"max_linear_velocity": float(base[params.FOLLOWER]["max_linear_velocity"])},
                       params.SMOOTHER: {"max_velocity": [float(x) for x in base[params.SMOOTHER]["max_velocity"]]}}

        bad = []
        for n, vals in changes.items():
            res = params.set_params(node, n, vals)
            if res is None:
                bad.append(f"{n} 无应答")
                continue
            bad += [f"{n} {p}：{why or '被拒绝'}" for p, ok, why in res if not ok]
        after = params.snapshot(node)
        for n, vals in changes.items():
            for p, want in vals.items():
                got = (after.get(n) or {}).get(p)
                same = (got == want) if isinstance(want, list) else (got is not None and abs(got - want) < 1e-9)
                if not same:
                    bad.append(f"{n} {p} 读回 {got}，期望 {want}")
        path = save(pdir, f"set_{a.speed:g}" if a.op == "set" else "restore", after, {"changes": changes})
        print(common.md_table(["节点", "参数", "值"], show_rows(after)))
        if bad:
            print("\n✗ " + "\n✗ ".join(bad))
            return 1
        print(f"\n✓ {'已改档到 ' + format(a.speed, 'g') + ' m/s' if a.op == 'set' else '已恢复基线'}，"
              f"读回一致；存档 {path}\n  改档后重新下发任务，正在执行的路线不按新档位")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

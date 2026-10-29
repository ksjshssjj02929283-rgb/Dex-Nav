#!/usr/bin/env python3
"""紧急制动测试工具（G-MOT-04）：让机器人直线加速到目标速度，再由脚本自己发停车指令，测制动距离。

两种停车方式：
  hard  向硬停话题（默认 /cmd_vel_hard_stop）发 std_msgs/Empty。velocity_smoother 收到后立刻输出零速并保持
        hard_stop_duration（默认 0.5 s），跳过 max_decel，机器人按底盘自身能力刹停。nav2 控制器碰撞检测
        「Collision ahead. Hard stop active.」和控制失败时走的就是这条通道。
  soft  把输入改成零速，经 velocity_smoother 的 max_decel 减速（nav2 配置 −0.75 m/s²），作对照。
触发时刻由脚本自己记录，比按急停键后再从约 1 Hz 的状态话题里找时刻准。

前提：
  - nav2 后端在跑（velocity_smoother 激活，输入话题 /cmd_vel_nav）；当前没有导航任务（否则控制器也在发速度）
  - 手柄不要接管（cmd_vel_mux 里手柄优先级更高）
  - 前方直线空地 ≥ 加速距离 + 2 m（0.7 m/s 约需 3 m 加速）；测试人员手持急停
用法：
  nbt brake run --speed 0.7 --mode hard -o ~/brake_eval --label "空载"
  nbt brake run --speed 0.5 --mode soft -o ~/brake_eval --load-kg 5
  nbt brake run --speed 0.7 --mode hard -o ~/brake_eval --scan-topic /laser_scan/front
      # 正前方正对墙时，用雷达测到墙距离的变化做交叉核对（不受轮子打滑影响）
每次运行在 <out>/runs.csv 追加一行，原始采样存 <out>/run_<时间>_samples.csv，并打印一行可贴进 G-MOT-04 记录表的 markdown。
"""
import argparse
import bisect
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

try:
    import tf2_ros
except ImportError:  # 没有 tf2_ros 时只用里程计
    tf2_ros = None


def stamp_sec(msg_stamp, fallback):
    s = msg_stamp.sec + msg_stamp.nanosec * 1e-9
    return s if s > 0 else fallback


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def travel_dir(series, t, interp_fn, lookback=0.5, min_move=0.02):
    """series [(t, x, y, yaw)] 在 t 之前 lookback 秒内的位移方向；位移不足 min_move 米时返回 t 前一个采样的朝向。"""
    pa, pb = interp_fn(series, t - lookback), interp_fn(series, t)
    if pa is not None and pb is not None and math.hypot(pb[0] - pa[0], pb[1] - pa[1]) >= min_move:
        return math.atan2(pb[1] - pa[1], pb[0] - pa[0])
    k = max(0, bisect.bisect_left([x[0] for x in series], t) - 1)
    return series[k][3]


def interp(series, t):
    """series: [(t, x, y, yaw)] 按时间排序；返回 t 时刻的 (x, y)。"""
    if not series:
        return None
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return series[0][1:3]
    if i >= len(series):
        return series[-1][1:3]
    (t0, x0, y0), (t1, x1, y1) = series[i - 1][:3], series[i][:3]
    k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return x0 + k * (x1 - x0), y0 + k * (y1 - y0)


class BrakeTest(Node):
    def __init__(self, a):
        super().__init__("brake_test")
        self.a = a
        self.cmd_pub = self.create_publisher(Twist, a.cmd_topic, 10)
        self.hs_pub = self.create_publisher(Empty, a.hard_stop_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.odom = []   # (t, vx, x, y, yaw)
        self.scan = []   # (t, d)
        self.tf = []     # (t, x, y)
        self.create_subscription(Odometry, a.odom_topic, self.on_odom, qos_profile_sensor_data)
        if a.scan_topic:
            self.create_subscription(LaserScan, a.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.tf_buf = None
        if tf2_ros and not a.no_tf:
            self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_timer(0.02, self.on_tf_timer)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        t = stamp_sec(m.header.stamp, self.now_s())
        p = m.pose.pose
        self.odom.append((t, m.twist.twist.linear.x, p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_scan(self, m):
        t = stamp_sec(m.header.stamp, self.now_s())
        half = math.radians(self.a.scan_half_deg)
        vals = []
        for i, rng in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if abs(math.atan2(math.sin(ang), math.cos(ang))) <= half and m.range_min < rng < m.range_max:
                vals.append(rng)
        if vals:
            vals.sort()
            self.scan.append((t, vals[len(vals) // 2]))

    def on_tf_timer(self):
        try:
            tr = self.tf_buf.lookup_transform(self.a.map_frame, self.a.base_frame, Time())
        except Exception:  # noqa: BLE001  TF 暂时拿不到就跳过这一拍
            return
        t = stamp_sec(tr.header.stamp, self.now_s())
        if not self.tf or t > self.tf[-1][0]:
            self.tf.append((t, tr.transform.translation.x, tr.transform.translation.y,
                            yaw_of(tr.transform.rotation)))

    def speed(self):
        return self.odom[-1][1] if self.odom else None

    def send(self, v):
        msg = Twist()
        msg.linear.x = float(v)
        self.cmd_pub.publish(msg)

    def spin_for(self, sec, v=None, hard=False):
        end = time.time() + sec
        last_hs = 0.0
        while time.time() < end and rclpy.ok():
            if v is not None:
                self.send(v)
            if hard and time.time() - last_hs >= 0.2:   # 持续发，保证保持期内控制输入被忽略
                self.hs_pub.publish(Empty())
                last_hs = time.time()
            rclpy.spin_once(self, timeout_sec=0.05)

    def safe_stop(self):
        for _ in range(5):
            self.send(0.0)
            self.hs_pub.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.05)


def analyse(n, a, t_trig, mode):
    od = sorted(n.odom)
    after = [o for o in od if o[0] >= t_trig]
    before = [o for o in od if o[0] < t_trig]
    if not before or len(after) < 5:
        return None, "触发前后的里程计数据不够"
    v_trig = before[-1][1]
    # 响应：速度比触发时下降超过 resp_dv 的第一个时刻
    t_resp = next((o[0] for o in after if o[1] <= v_trig - a.resp_dv), None)
    # 静止：|v| < still_v 并保持 still_hold 秒的起点
    t_stop = None
    for i, o in enumerate(after):
        if abs(o[1]) < a.still_v:
            j = i
            while j < len(after) and abs(after[j][1]) < a.still_v:
                j += 1
            if after[min(j, len(after) - 1)][0] - o[0] >= a.still_hold or j == len(after):
                t_stop = o[0]
                break
    if t_stop is None:
        return None, "没有等到机器人停稳"
    def along(series):
        """沿触发前行驶方向的位移 mm。方向取同一序列触发前 0.5 s 的位移方向（各来源在自己的坐标系里算，
        map 与 odom 朝向不同也不受影响）；那段位移太短时退到触发前一个采样的朝向。"""
        p0, p1 = interp(series, t_trig), interp(series, t_stop)
        if p0 is None or p1 is None:
            return None
        yaw = travel_dir(series, t_trig, interp)
        return ((p1[0] - p0[0]) * math.cos(yaw) + (p1[1] - p0[1]) * math.sin(yaw)) * 1000.0

    dist_odom = along([(o[0], o[2], o[3], o[4]) for o in od])
    dist_tf = along(n.tf) if len(n.tf) > 5 else None
    dist_scan = None
    if len(n.scan) > 3:
        sc = sorted(n.scan)
        ts = [s[0] for s in sc]
        i0 = bisect.bisect_left(ts, t_trig)
        if 0 < i0 < len(sc):
            (ta, da), (tb, db) = sc[i0 - 1], sc[i0]
            d0 = da + (db - da) * ((t_trig - ta) / (tb - ta) if tb > ta else 0.0)
            rest = [d for t, d in sc if t >= t_stop + 0.2]
            if rest:
                rest.sort()
                dist_scan = (d0 - rest[len(rest) // 2]) * 1000.0
    # 峰值减速度：按 --decel-window（默认 50 ms）的时间窗做差分，减少采样抖动的影响
    seg = [o for o in after if o[0] <= t_stop + 0.1]
    peak = None
    decs = []
    for i0, o in enumerate(seg):
        j0 = next((k for k in range(i0 + 1, len(seg)) if seg[k][0] - o[0] >= a.decel_window), None)
        if j0 is None:
            break
        decs.append((o[1] - seg[j0][1]) / (seg[j0][0] - o[0]))
    if decs:
        peak = max(decs)
    resp_ms = (t_resp - t_trig) * 1000.0 if t_resp else None
    brake_s = t_stop - (t_resp or t_trig)
    mean_dec = v_trig / brake_s if brake_s > 0 else None
    res = {"mode": mode, "v_trig": v_trig, "resp_ms": resp_ms, "stop_s": t_stop - t_trig,
           "dist_odom_mm": dist_odom, "dist_tf_mm": dist_tf, "dist_scan_mm": dist_scan,
           "peak_decel": peak, "mean_decel": mean_dec}
    if mode == "soft":
        res["theory_mm"] = v_trig * v_trig / (2 * a.soft_decel) * 1000.0
    return res, ""


def f(x, n=0):
    return "" if x is None else (f"{x:.{n}f}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt brake run", description="紧急制动测试（G-MOT-04）")
    ap.add_argument("--speed", type=float, required=True, help="目标速度 m/s（前进，> 0）")
    ap.add_argument("--mode", choices=["hard", "soft"], default="hard")
    ap.add_argument("-o", "--out", default=os.path.expanduser("~/brake_eval"))
    ap.add_argument("--label", default="", help="工况说明")
    ap.add_argument("--load-kg", type=float, default=0.0)
    ap.add_argument("--cmd-topic", default="/cmd_vel_nav", help="velocity_smoother 输入话题")
    ap.add_argument("--hard-stop-topic", default="/cmd_vel_hard_stop")
    ap.add_argument("--odom-topic", default="/feedback/odom")
    ap.add_argument("--scan-topic", default="", help="可选：正前方雷达，用到墙距离做交叉核对")
    ap.add_argument("--scan-half-deg", type=float, default=2.0)
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--no-tf", action="store_true", help="不取 map → base_link，只用里程计")
    ap.add_argument("--max-distance", type=float, default=4.0, help="加速段最长行驶距离 m，超过就中止")
    ap.add_argument("--steady-tol", type=float, default=0.03, help="稳定速度容差 m/s")
    ap.add_argument("--steady-time", type=float, default=1.0, help="稳定保持多久后触发 s")
    ap.add_argument("--resp-dv", type=float, default=0.02, help="速度下降多少算开始响应 m/s")
    ap.add_argument("--still-v", type=float, default=0.01, help="静止判定速度 m/s")
    ap.add_argument("--still-hold", type=float, default=0.5, help="静止保持 s")
    ap.add_argument("--decel-window", type=float, default=0.05, help="算峰值减速度的差分时间窗 s")
    ap.add_argument("--soft-decel", type=float, default=0.75, help="soft 模式理论减速度（平滑器 max_decel）m/s²")
    ap.add_argument("--yes", action="store_true", help="不等回车确认，直接开始")
    a = ap.parse_args(argv)
    if not 0.05 <= a.speed <= 1.0:
        sys.exit("目标速度只允许 0.05～1.0 m/s")
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    n = BrakeTest(a)
    try:
        t_wait = time.time() + 5.0
        while not n.odom and time.time() < t_wait:
            rclpy.spin_once(n, timeout_sec=0.1)
        if not n.odom:
            sys.exit(f"5 s 内没收到 {a.odom_topic}")
        n.spin_for(0.5)
        subs_cmd = n.count_subscribers(a.cmd_topic)
        subs_hs = n.count_subscribers(a.hard_stop_topic)
        print(f"订阅者：{a.cmd_topic} {subs_cmd} 个，{a.hard_stop_topic} {subs_hs} 个；当前速度 {n.speed():.3f} m/s")
        if subs_cmd == 0:
            sys.exit("速度输入话题没有订阅者（velocity_smoother 没在跑？），中止")
        if a.mode == "hard" and subs_hs == 0:
            sys.exit("硬停话题没有订阅者，hard 模式测不了，中止")
        if abs(n.speed()) > a.still_v * 2:
            sys.exit("机器人没有静止（可能有导航任务或遥控在发速度），中止")
        if not a.yes:
            input(f"即将以 {a.speed} m/s 直线前进，最多 {a.max_distance} m，然后 {a.mode} 停车。确认前方无人无障碍，按回车开始…")
        # 加速段
        x0 = n.odom[-1][2:4]
        steady_since = None
        t_lim = time.time() + 20.0
        aborted = ""
        while rclpy.ok():
            n.spin_for(0.05, v=a.speed)
            v = n.speed()
            x = n.odom[-1][2:4]
            travelled = math.hypot(x[0] - x0[0], x[1] - x0[1])
            if abs(v - a.speed) <= a.steady_tol:
                steady_since = steady_since or time.time()
                if time.time() - steady_since >= a.steady_time:
                    break
            else:
                steady_since = None
            if travelled >= a.max_distance:
                aborted = f"加速段已走 {travelled:.2f} m 仍未稳定在 {a.speed} m/s"
                break
            if time.time() > t_lim:
                aborted = "加速 20 s 仍未稳定"
                break
        if aborted:
            n.safe_stop()
            sys.exit(f"中止：{aborted}（目标速度不能超过 velocity_smoother 的 max_velocity，nbt env 可查；速度规划不参与）")
        # 触发
        t_trig = n.now_s()
        if a.mode == "hard":
            n.hs_pub.publish(Empty())
        n.spin_for(5.0, v=0.0, hard=(a.mode == "hard"))
        n.safe_stop()
        res, why = analyse(n, a, t_trig, a.mode)
    except KeyboardInterrupt:
        n.safe_stop()
        print("用户中断，已发零速和硬停")
        return
    finally:
        pass

    tag = time.strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(a.out, f"run_{tag}_samples.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["src", "t_rel_s", "vx", "x", "y"])
        for o in n.odom:
            w.writerow(["odom", f"{o[0] - t_trig:.4f}", f"{o[1]:.4f}", f"{o[2]:.4f}", f"{o[3]:.4f}"])
        for o in n.tf:
            w.writerow(["tf", f"{o[0] - t_trig:.4f}", "", f"{o[1]:.4f}", f"{o[2]:.4f}"])
        for o in n.scan:
            w.writerow(["scan_front_m", f"{o[0] - t_trig:.4f}", "", f"{o[1]:.4f}", ""])
    if res is None:
        print(f"分析失败：{why}；原始数据见 run_{tag}_samples.csv")
        n.destroy_node()
        rclpy.shutdown()
        return
    runs = os.path.join(a.out, "runs.csv")
    new = not os.path.exists(runs)
    cols = ["time", "label", "mode", "target_speed", "v_trig", "load_kg", "resp_ms", "stop_s", "dist_tf_mm",
            "dist_odom_mm", "dist_scan_mm", "theory_mm", "peak_decel", "mean_decel", "samples"]
    with open(runs, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(cols)
        w.writerow([tag, a.label, a.mode, a.speed, f(res["v_trig"], 3), a.load_kg, f(res["resp_ms"]),
                    f(res["stop_s"], 3), f(res["dist_tf_mm"]), f(res["dist_odom_mm"]), f(res["dist_scan_mm"]),
                    f(res.get("theory_mm")), f(res["peak_decel"], 2), f(res["mean_decel"], 2),
                    f"run_{tag}_samples.csv"])
    dist_main = res["dist_tf_mm"] if res["dist_tf_mm"] is not None else res["dist_odom_mm"]
    print(f"\n结果（{a.mode}，{a.label or '—'}）：触发时速度 {res['v_trig']:.3f} m/s，响应延迟 {f(res['resp_ms'])} ms，"
          f"触发到停稳 {res['stop_s']:.3f} s")
    print(f"  制动距离：定位 TF {f(res['dist_tf_mm'])} mm ｜ 里程计 {f(res['dist_odom_mm'])} mm ｜ 雷达到墙 "
          f"{f(res['dist_scan_mm'])} mm" + (f" ｜ 平滑器理论值 {f(res['theory_mm'])} mm" if a.mode == "soft" else ""))
    print(f"  峰值减速度 {f(res['peak_decel'], 2)} m/s²，平均减速度 {f(res['mean_decel'], 2)} m/s²")
    chan = "④ 硬停" if a.mode == "hard" else "③ 平滑器停车"
    print("\nG-MOT-04 记录表行：")
    print(f"| | {tag} | nav2 | {chan} | {a.speed} | {res['v_trig']:.3f} | {a.load_kg:g} | 脚本触发 | +{res['stop_s']:.3f} s | "
          f"{f(res['resp_ms'])} | {f(dist_main)} | {f(res['peak_decel'], 2)} | | run_{tag}_samples.csv | "
          f"里程计 {f(res['dist_odom_mm'])} / 雷达 {f(res['dist_scan_mm'])} mm |")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

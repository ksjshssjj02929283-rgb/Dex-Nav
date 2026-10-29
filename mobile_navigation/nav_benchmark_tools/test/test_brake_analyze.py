"""合成「0.7 m/s 匀速 → 硬停 → 40 ms 后 3 m/s² 减速到停」的包，核对 brake analyze 的识别、距离和判定来源。"""
import csv
import math
import os

import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

from nav_benchmark_tools import brake_analyze

T0 = 1_800_000_000.0
V, HS, LAT, DEC = 0.7, 3.0, 0.04, 3.0
THEORY = (V * LAT + V * V / (2 * DEC)) * 1000.0
MAP_ODOM = (2.0, -1.0, 0.6)          # map→odom：平移 + 旋转，检验复合
ODOM_YAW = 0.3                         # 底盘在 odom 系里的行驶方向


def stamp(h, t):
    h.stamp.sec = int(T0 + t)
    h.stamp.nanosec = int(((T0 + t) % 1) * 1e9)


def tf(parent, child, t, x, y, yaw):
    m = TransformStamped()
    stamp(m.header, t)
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y = x, y
    m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


def write_bag(path, with_tf=True, pose_every=5, pose_yaw_ok=True):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = [("/feedback/odom", "nav_msgs/msg/Odometry"),
              ("/localization/loc_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
              ("/cmd_vel_nav", "geometry_msgs/msg/Twist"), ("/cmd_vel_hard_stop", "std_msgs/msg/Empty")]
    if with_tf:
        topics.append(("/tf", "tf2_msgs/msg/TFMessage"))
    for n, t in topics:
        w.create_topic(rosbag2_py.TopicMetadata(name=n, type=t, serialization_format="cdr"))

    def put(n, m, t, w=w):
        w.write(n, serialize_message(m), int((T0 + t) * 1e9))
    mx, my, mw = MAP_ODOM
    v, s, i = V, 0.0, 0
    while i <= 600:
        t = i * 0.01
        if i == int(HS * 100):
            put("/cmd_vel_hard_stop", Empty(), t)
        if t >= HS + LAT:
            v = max(0.0, v - DEC * 0.01)
        ox, oy = s * math.cos(ODOM_YAW), s * math.sin(ODOM_YAW)
        o = Odometry()
        stamp(o.header, t)
        o.pose.pose.position.x, o.pose.pose.position.y = ox, oy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(ODOM_YAW / 2), math.cos(ODOM_YAW / 2)
        o.twist.twist.linear.x = v
        put("/feedback/odom", o, t)
        if with_tf:
            trs = [tf("odom", "base_footprint", t, ox, oy, ODOM_YAW)]
            if i % 4 == 0:
                trs.append(tf("map", "odom", t, mx, my, mw))
            put("/tf", TFMessage(transforms=trs), t)
        if i % pose_every == 0:
            p = PoseWithCovarianceStamped()
            stamp(p.header, t)
            p.pose.pose.position.x = mx + math.cos(mw) * ox - math.sin(mw) * oy
            p.pose.pose.position.y = my + math.sin(mw) * ox + math.cos(mw) * oy
            pyaw = mw + ODOM_YAW if pose_yaw_ok else 0.0      # 朝向字段错了也要算对（方向取位移方向）
            p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(pyaw / 2), math.cos(pyaw / 2)
            put("/localization/loc_pose", p, t)
        if i % 5 == 0:
            tw = Twist()
            tw.linear.x = 0.0 if t >= HS else V
            put("/cmd_vel_nav", tw, t)
        s += v * 0.01
        i += 1
    del w


def run(tmp_path, name, **kw):
    bag = str(tmp_path / name)
    write_bag(bag, **kw)
    out = str(tmp_path / (name + "_out"))
    brake_analyze.main([bag, "-o", out, "--trigger", "hard_stop"])
    with open(os.path.join(out, "events.csv")) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1 and os.path.exists(os.path.join(out, "summary.md"))
    return rows[0]


def test_tf_is_judgement(tmp_path):
    r = run(tmp_path, "tf")
    assert r["trigger"] == "hard_stop" and 20 <= float(r["resp_ms"]) <= 70
    assert r["dist_src"] == "定位 TF"
    assert math.isclose(float(r["dist_mm"]), THEORY, abs_tol=15.0)
    assert math.isclose(float(r["dist_tf_mm"]), float(r["dist_odom_mm"]), abs_tol=2.0)


def test_low_rate_pose_without_tf_falls_back_to_odom(tmp_path):
    r = run(tmp_path, "slow", with_tf=False, pose_every=50)     # 定位话题 2 Hz
    assert r["dist_src"] == "里程计"
    assert math.isclose(float(r["dist_mm"]), THEORY, abs_tol=15.0)
    assert "只作参考" in r["note"] and "判定值改用里程计" in r["note"]


def test_fast_pose_without_tf(tmp_path):
    r = run(tmp_path, "fast", with_tf=False, pose_every=5, pose_yaw_ok=False)      # 定位话题 20 Hz
    assert r["dist_src"] == "定位话题"
    assert math.isclose(float(r["dist_mm"]), THEORY, abs_tol=15.0)

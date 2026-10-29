#!/usr/bin/env python3
"""建图资源评估工具（G-MAP-03）。

在机器人上与建图同时运行：
  - 每隔 --interval 秒采样建图进程（默认 cartographer_node、cartographer_occupancy_grid_node）
    的 CPU 和内存，以及整机 CPU 和内存；
  - 订阅 /mapping/state，记录建图阶段切换（MAPPING / SAVING / IDLE / ERROR）和收尾进度；
  - 结束时读 map.yaml 的面积字段，输出汇总，并生成一行可直接贴进 SOP 记录表的 Markdown。

只用 Python 标准库读 /proc；ROS 部分需要先 source ROS2 和 nav_interfaces，
没有 ROS 环境时自动退化为只采资源（--no-ros 效果相同）。

用法：
  nbt res map -o ~/map_eval/R2-1_1000a --label "≤1000 / 通用测试场"
  nbt res map -o out --auto-stop        # 收尾完成后 10 s 自动退出
  nbt res map -o out --no-ros --duration 600

输出（-o 目录下）：
  samples.csv   每次采样一行：时间、所处阶段、各进程 CPU% / RSS、合计、整机 CPU% / 内存
  events.csv    /mapping/state 的变化（status、stage、progress、error_code）
  summary.json  汇总：各阶段时长、峰值内存、CPU 均值 / 峰值、面积
"""
import argparse
import csv
import json
import os
import re
import signal
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024
NCPU = os.cpu_count() or 1
STATUS_NAME = {-1: "ERROR", 0: "IDLE", 1: "MAPPING", 2: "SAVING"}


# ---------------------------------------------------------------- /proc 读取

def read_proc_targets(names, match_cmdline):
    """返回 {pid: 进程名}。comm 只保留 15 个字符，所以按 argv[0] 的文件名匹配。"""
    found = {}
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if not argv or not argv[0]:
            continue
        exe = os.path.basename(argv[0].decode(errors="replace"))
        if exe in names:
            found[int(d)] = exe
        elif match_cmdline:
            full = b" ".join(argv).decode(errors="replace")
            for n in names:
                if n in full:
                    found[int(d)] = n
                    break
    return found


def read_proc_cpu_ticks(pid):
    with open(f"/proc/{pid}/stat") as f:
        s = f.read()
    # comm 可能含空格，按最后一个 ')' 切开
    fields = s[s.rindex(")") + 2:].split()
    return int(fields[11]) + int(fields[12])  # utime + stime


def read_proc_rss_mb(pid):
    with open(f"/proc/{pid}/statm") as f:
        return int(f.read().split()[1]) * PAGE_KB / 1024.0


def read_sys_cpu():
    with open("/proc/stat") as f:
        v = [int(x) for x in f.readline().split()[1:]]
    idle = v[3] + (v[4] if len(v) > 4 else 0)
    return sum(v), idle


def read_sys_mem_mb():
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, val = line.split(":", 1)
            info[k] = int(val.split()[0])
    total = info["MemTotal"] / 1024.0
    avail = info.get("MemAvailable", info["MemFree"]) / 1024.0
    return total, total - avail


# ---------------------------------------------------------------- 采样器

class Sampler:
    def __init__(self, names, match_cmdline):
        self.names = names
        self.match_cmdline = match_cmdline
        self.prev_proc = {}   # pid -> (ticks, t)
        self.prev_sys = None
        self.mem_total = read_sys_mem_mb()[0]

    def sample(self):
        now = time.time()
        targets = read_proc_targets(self.names, self.match_cmdline)
        per_name = {n: {"cpu": 0.0, "rss": 0.0, "pids": 0} for n in self.names}
        cur = {}
        for pid, name in targets.items():
            try:
                ticks = read_proc_cpu_ticks(pid)
                rss = read_proc_rss_mb(pid)
            except (OSError, ValueError):
                continue
            cur[pid] = (ticks, now)
            cpu = 0.0
            if pid in self.prev_proc:
                pt, ptime = self.prev_proc[pid]
                dt = now - ptime
                if dt > 0:
                    cpu = (ticks - pt) / CLK_TCK / dt * 100.0  # 单核百分比
            per_name[name]["cpu"] += cpu
            per_name[name]["rss"] += rss
            per_name[name]["pids"] += 1
        self.prev_proc = cur

        total, idle = read_sys_cpu()
        sys_cpu = 0.0
        if self.prev_sys:
            dtot, didle = total - self.prev_sys[0], idle - self.prev_sys[1]
            if dtot > 0:
                sys_cpu = (dtot - didle) / dtot * 100.0  # 整机平均（所有核）
        self.prev_sys = (total, idle)
        _, mem_used = read_sys_mem_mb()
        return now, per_name, sys_cpu, mem_used


# ---------------------------------------------------------------- ROS

class MappingStateListener:
    """订阅 /mapping/state，只在 status / stage / error_code 变化或进度跨过 10% 时记事件。"""

    def __init__(self, topic):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from nav_interfaces.msg import MappingState
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("mapping_resource_monitor")
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.node.create_subscription(MappingState, topic, self._cb, qos)
        self.events = []
        self.status = None
        self.map_name = ""
        self.map_path = ""
        self._last = None
        self.saving_done_at = None  # SAVING -> IDLE 的本机时间

    def _cb(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        key = (m.status, m.stage, m.error_code, int(m.progress // 10))
        if m.map_name:
            self.map_name = m.map_name
        if m.map_path:
            self.map_path = m.map_path
        if key == self._last:
            return
        if self.status == 2 and m.status == 0:
            self.saving_done_at = time.time()
        self._last = key
        self.status = m.status
        self.events.append({
            "recv_time": time.time(), "stamp": st, "status": m.status,
            "status_name": STATUS_NAME.get(m.status, str(m.status)),
            "stage": m.stage, "progress": round(m.progress, 1),
            "error_code": m.error_code, "map_name": m.map_name,
        })

    def spin_once(self, timeout):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


# ---------------------------------------------------------------- 汇总

def first_event(events, cond, after=None):
    for e in events:
        if (after is None or e["recv_time"] >= after["recv_time"]) and cond(e):
            return e
    return None


def phase_durations(events):
    """按 header.stamp 求建图时长和收尾（生成）时长；同时给出本机接收时间差作参考。"""
    out = {}
    mapping = first_event(events, lambda e: e["status"] == 1)
    saving = first_event(events, lambda e: e["status"] == 2, mapping)
    done = first_event(events, lambda e: e["status"] == 0, saving) if saving else None
    error = first_event(events, lambda e: e["status"] == -1, mapping)
    if mapping and saving:
        out["mapping_s"] = round(saving["stamp"] - mapping["stamp"], 2)
    if saving and done:
        out["finalize_s"] = round(done["stamp"] - saving["stamp"], 2)
        out["finalize_s_wall"] = round(done["recv_time"] - saving["recv_time"], 2)
        out["finalize_ok"] = done["progress"] >= 100 and done["error_code"] == 0
        out["finalize_error_code"] = done["error_code"]
    if error:
        out["error_code"] = error["error_code"]
    stages = {}
    if saving:
        seq = [e for e in events if e["recv_time"] >= saving["recv_time"] and e["status"] == 2]
        for a, b in zip(seq, seq[1:] + ([done] if done else [])):
            if a["stage"]:
                stages[a["stage"]] = round(stages.get(a["stage"], 0) + b["stamp"] - a["stamp"], 2)
    if stages:
        out["finalize_stages_s"] = stages
    return out


def read_area(maps_dir, map_name, map_path):
    path = os.path.join(map_path, "map.yaml") if map_path else \
        os.path.join(os.path.expanduser(maps_dir), map_name, "slam", "map.yaml")
    res = {"map_yaml": path}
    try:
        with open(path) as f:
            txt = f.read()
    except OSError:
        res["error"] = "map.yaml 不存在或不可读"
        return res
    for k in ("total_area_m2", "effective_area_m2", "resolution"):
        m = re.search(rf"^\s*{k}\s*:\s*([0-9.eE+-]+)", txt, re.M)
        if m:
            res[k] = float(m.group(1))
    return res


def area_bucket(a):
    if a is None:
        return ""
    for lim in (1000, 5000, 10000, 25000):
        if a <= lim:
            return f"≤{lim}"
    return ">25000"


def stats(rows, key, phase=None):
    vals = [r[key] for r in rows if phase is None or r["phase"] == phase]
    if not vals:
        return None
    return {"avg": round(sum(vals) / len(vals), 1), "max": round(max(vals), 1), "n": len(vals)}


# ---------------------------------------------------------------- 主流程

def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt res map", description="建图资源评估（G-MAP-03）")
    ap.add_argument("-o", "--out", required=True, help="输出目录")
    ap.add_argument("--interval", type=float, default=1.0, help="采样间隔 s（默认 1）")
    ap.add_argument("--procs", default="cartographer_node,cartographer_occupancy_grid_node",
                    help="建图进程名，逗号分隔，按 argv[0] 文件名匹配")
    ap.add_argument("--match-cmdline", action="store_true", help="进程名也在完整命令行里做子串匹配")
    ap.add_argument("--topic", default="/mapping/state")
    ap.add_argument("--maps-dir", default="~/.dexe/data/maps")
    ap.add_argument("--map-name", default="", help="不订阅 ROS 时用来找 map.yaml")
    ap.add_argument("--label", default="", help="写进汇总的说明，如「≤1000 / 通用测试场」")
    ap.add_argument("--no-ros", action="store_true", help="不订阅 /mapping/state，只采资源")
    ap.add_argument("--auto-stop", action="store_true", help="收尾完成（SAVING→IDLE）后自动退出")
    ap.add_argument("--auto-stop-delay", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=0, help="最长运行秒数，0 = 不限")
    args = ap.parse_args(argv)

    names = [n.strip() for n in args.procs.split(",") if n.strip()]
    os.makedirs(args.out, exist_ok=True)

    ros = None
    if not args.no_ros:
        try:
            ros = MappingStateListener(args.topic)
        except Exception as e:  # 没有 ROS 环境或缺 nav_interfaces
            print(f"[warn] 订阅 {args.topic} 失败，只采资源：{e}", file=sys.stderr)

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    sampler = Sampler(names, args.match_cmdline)
    sampler.sample()  # 预热，建立 CPU 差分基准
    rows = []
    t0 = time.time()
    fields = ["time", "elapsed_s", "phase"]
    for n in names:
        fields += [f"{n}_cpu_pct", f"{n}_rss_mb", f"{n}_pids"]
    fields += ["sum_cpu_pct", "sum_rss_mb", "sys_cpu_pct", "sys_mem_used_mb", "sys_mem_used_pct"]
    fcsv = open(os.path.join(args.out, "samples.csv"), "w", newline="")
    w = csv.DictWriter(fcsv, fieldnames=fields)
    w.writeheader()
    print(f"[info] 采样 {names}，间隔 {args.interval}s；Ctrl+C 结束。输出：{args.out}")

    next_t = time.time() + args.interval
    while not stop["flag"]:
        if ros:
            ros.spin_once(max(0.0, min(0.1, next_t - time.time())))
        else:
            time.sleep(max(0.0, min(0.1, next_t - time.time())))
        if time.time() < next_t:
            continue
        next_t += args.interval
        now, per, sys_cpu, mem_used = sampler.sample()
        phase = STATUS_NAME.get(ros.status, "UNKNOWN") if ros and ros.status is not None else "UNKNOWN"
        row = {"time": round(now, 3), "elapsed_s": round(now - t0, 1), "phase": phase}
        scpu = srss = 0.0
        for n in names:
            row[f"{n}_cpu_pct"] = round(per[n]["cpu"], 1)
            row[f"{n}_rss_mb"] = round(per[n]["rss"], 1)
            row[f"{n}_pids"] = per[n]["pids"]
            scpu += per[n]["cpu"]
            srss += per[n]["rss"]
        row.update(sum_cpu_pct=round(scpu, 1), sum_rss_mb=round(srss, 1), sys_cpu_pct=round(sys_cpu, 1),
                   sys_mem_used_mb=round(mem_used, 1),
                   sys_mem_used_pct=round(mem_used / sampler.mem_total * 100.0, 1))
        w.writerow(row)
        fcsv.flush()
        rows.append(row)
        if int(row["elapsed_s"]) % 30 == 0:
            print(f"[{row['elapsed_s']:>7}s] {phase:8} 建图进程 CPU {scpu:6.1f}%  RSS {srss:8.1f} MB  "
                  f"整机 CPU {sys_cpu:5.1f}%  内存 {row['sys_mem_used_pct']:5.1f}%")
        if args.duration and now - t0 >= args.duration:
            break
        if args.auto_stop and ros and ros.saving_done_at and now - ros.saving_done_at >= args.auto_stop_delay:
            break
    fcsv.close()

    events = ros.events if ros else []
    with open(os.path.join(args.out, "events.csv"), "w", newline="") as f:
        ew = csv.DictWriter(f, fieldnames=["recv_time", "stamp", "status", "status_name", "stage",
                                           "progress", "error_code", "map_name"])
        ew.writeheader()
        ew.writerows(events)

    map_name = (ros.map_name if ros else "") or args.map_name
    map_path = ros.map_path if ros else ""
    area = read_area(args.maps_dir, map_name, map_path) if (map_name or map_path) else {}
    found = any(r[f"{n}_pids"] for r in rows for n in names)
    summary = {
        "label": args.label, "map_name": map_name, "procs": names, "procs_found": found,
        "interval_s": args.interval, "samples": len(rows), "ncpu": NCPU,
        "durations": phase_durations(events),
        "area": area, "area_bucket": area_bucket(area.get("total_area_m2")),
        "sum_rss_mb": {p: stats(rows, "sum_rss_mb", p) for p in ("MAPPING", "SAVING")},
        "sum_cpu_pct": {p: stats(rows, "sum_cpu_pct", p) for p in ("MAPPING", "SAVING")},
        "sys_cpu_pct": {p: stats(rows, "sys_cpu_pct", p) for p in ("MAPPING", "SAVING")},
        "sys_mem_used_pct": stats(rows, "sys_mem_used_pct"),
        "peak_sum_rss_mb": stats(rows, "sum_rss_mb")["max"] if rows else None,
        "note": "sum_cpu_pct 为建图进程合计的单核百分比（100% = 占满 1 核）；sys_cpu_pct 为整机平均",
    }
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    if not found:
        print(f"[warn] 整个采样期间没找到建图进程 {names}：进程名不对，或建图没在本机运行。"
              "资源数据无效，可用 --procs 指定进程名、--match-cmdline 放宽匹配。", file=sys.stderr)
    d = summary["durations"]
    cpu_all = stats(rows, "sum_cpu_pct")
    md = "| {b} | {lab} | {mn} | {ta} | {ea} | {fin} | {rss} | {cpu} | | {out} | |".format(
        b=summary["area_bucket"], lab=args.label, mn=map_name,
        ta=area.get("total_area_m2", ""), ea=area.get("effective_area_m2", ""),
        fin=d.get("finalize_s", ""), rss=summary["peak_sum_rss_mb"] if found else "未采到进程",
        cpu=f"{cpu_all['avg']} / {cpu_all['max']}" if (cpu_all and found) else "未采到进程", out=args.out)
    print("\n==== 汇总 ====")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("\n记录表行（档位 | 场地 / bag | 地图名 | total | effective | 生成耗时 s | 峰值内存 MB | CPU 平均 / 峰值 % | 检查表 | 证据路径 | 备注）：")
    print(md)
    if ros:
        ros.shutdown()


if __name__ == "__main__":
    main()

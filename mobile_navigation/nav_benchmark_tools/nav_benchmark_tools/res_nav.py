#!/usr/bin/env python3
"""导航资源占用监控（G-SYS-04，也可用于长稳、停障等用例）。

W1 PRO 的 auto 模式（dexe-auto.service → auto_mode.sh）和 dexe-system.service（run_system.sh）
用 taskset 把各组进程绑在不同的核上，导航相关进程不止在 3、4 号核。本工具按「进程树归组」统计：
每个进程向上找祖先，看它是被哪个 ros2 launch / 启动脚本拉起的，再按下面的规则归到组和类别。

两个合计口径：
  纯算法    = 定位 + 感知预处理 + 导航规划控制 + 导航管理接口 + 回充
  带驱动    = 纯算法 + 导航传感器驱动（雷达、IMU、Orbbec 深度相机、底盘驱动）
另外单列：XWiz 后端、非导航（KFC 相机、ACT 推理、末端执行器、标定）、系统（supervisor、遥控、description），
以及 dexe 服务里没有匹配到规则的进程（「dexe 未分类」），便于补规则。

只用 Python 标准库读 /proc，不需要 ROS。在机器人上与测试同时运行：
  nbt res nav -o ~/nav_eval/R2-1_free_nav --duration 300 --label "② 走廊往返导航"
  nbt res nav -o out --rules my_rules.json        # 追加 / 覆盖分组规则
  nbt res nav --list-rules                         # 打印内置规则

输出（-o 目录下）：
  proc_samples.csv   每次采样每个进程一行：时间、pid、组、类别、CPU%（单核百分比）、RSS、运行在哪个核、可用核
  group_samples.csv  每次采样每个组一行
  core_samples.csv   每次采样一行：各核占用率
  summary.json / summary.md   汇总（类别、组、两个合计口径、各核、前 15 个进程、未分类进程）
"""
import argparse
import csv
import json
import os
import re
import signal
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024

# (组名, 类别, 正则) —— 正则匹配进程自身或任一祖先的完整命令行；按顺序，先命中先用。
# 来源：dexe_mobile_application/startup.d/w1_pro/001/pc1/auto_mode.sh、run_system.sh（V0.5.1）
DEFAULT_RULES = [
    ("激光雷达", "传感器驱动", r"dual_udp_lidar"),
    ("IMU", "传感器驱动", r"(^|[ /])imu[ /].*talker|lib/imu/talker"),
    ("Orbbec 深度相机", "传感器驱动", r"orbbec_nodes\.launch|orbbec_camera"),
    ("底盘驱动", "传感器驱动", r"run_w1\.sh|dexe_robot_server|chassis_node|ethercat"),
    ("RGBD 点云处理", "感知预处理", r"multi_camera\.launch|dexe_sensor_process"),
    ("定位", "定位", r"dexe_localization\.launch|localization_node"),
    ("建图", "定位", r"dexe_slam_2d\.launch|cartographer_node"),
    ("nav2 自由导航", "导航规划控制", r"w1_nav2_actual\.launch|navigation_launch\.py"),
    ("路网导航", "导航规划控制", r"path_navigation_bringup\.launch|nav2_schedule_planner"),
    ("精控", "导航规划控制", r"mobile_control_action"),
    ("nav_goal_server", "导航规划控制", r"nav_goal_server"),
    ("command_board 与地图管理", "导航管理接口", r"command_board\.launch|command_board|map_manager|map_asset_manager"),
    ("自动回充", "回充", r"auto_charging|charging_pose_service"),
    ("XWiz 后端", "XWiz 后端", r"gui_xwiz"),
    ("KFC 相机", "非导航", r"kfcv?2?_nodes\.launch"),
    ("ACT 推理", "非导航", r"w1_act|inference_service"),
    ("末端执行器", "非导航", r"run_ee\.sh"),
    ("标定管理", "非导航", r"calib_node_manager"),
    ("supervisor", "系统", r"dexe_supervisor"),
    ("手柄遥控", "系统", r"mobile_joy_control"),
    ("robot description", "系统", r"dexforce_description|robot_state_publisher"),
    ("启动脚本", "系统", r"startup(\.d/[^ ]*)?/(auto_mode|run_system|basic_service|run_devices|act_mode|tele_mode|map_mode|calib_mode)\.sh|systemd-cat"),
]
ALGO_CATS = ["定位", "感知预处理", "导航规划控制", "导航管理接口", "回充"]
DRIVER_CATS = ["传感器驱动"]
CAT_ORDER = ALGO_CATS + DRIVER_CATS + ["XWiz 后端", "非导航", "系统", "dexe 未分类"]
DEXE_CGROUP = re.compile(r"dexe-[a-z-]+\.service")
SHELLS = {"bash", "sh", "dash", "zsh"}


# ---------------------------------------------------------------- /proc

def list_procs():
    """返回 {pid: dict(ppid, cmd, cgroup)}。"""
    procs = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        pid = int(d)
        try:
            with open(f"/proc/{d}/stat") as f:
                st = f.read()
            ppid = int(st[st.rindex(")") + 2:].split()[1])
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            with open(f"/proc/{d}/cgroup") as f:
                cg = f.read()
        except (OSError, ValueError):
            continue
        if not cmd:  # 内核线程
            continue
        m = DEXE_CGROUP.search(cg)
        procs[pid] = {"ppid": ppid, "cmd": cmd, "unit": m.group(0) if m else ""}
    return procs


def read_proc_stat(pid):
    """返回 (utime+stime ticks, 最近运行的核, RSS MB)。"""
    with open(f"/proc/{pid}/stat") as f:
        s = f.read()
    fields = s[s.rindex(")") + 2:].split()
    ticks = int(fields[11]) + int(fields[12])
    last_cpu = int(fields[36])
    rss_mb = int(fields[21]) * PAGE_KB / 1024.0
    return ticks, last_cpu, rss_mb


def read_affinity(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("Cpus_allowed_list:"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return ""


def read_cores():
    cores = {}
    with open("/proc/stat") as f:
        for line in f:
            if re.match(r"cpu\d+ ", line):
                p = line.split()
                v = [int(x) for x in p[1:]]
                idle = v[3] + (v[4] if len(v) > 4 else 0)
                cores[int(p[0][3:])] = (sum(v), idle)
    return cores


def read_mem():
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, val = line.split(":", 1)
            info[k] = int(val.split()[0])
    total = info["MemTotal"] / 1024.0
    return total, total - info.get("MemAvailable", info["MemFree"]) / 1024.0


# ---------------------------------------------------------------- 归组

class Classifier:
    def __init__(self, rules, ancestor_any=False):
        self.rules = [(g, c, re.compile(p)) for g, c, p in rules]
        self.ancestor_any = ancestor_any
        self.cache = {}

    def classify(self, pid, procs):
        """自身及祖先逐级匹配规则；都没命中时，属于 dexe-*.service 的进程归「dexe 未分类」。

        祖先只在同一个 dexe-*.service 内部往上找：服务外的 shell（比如测试人员手敲的 bash -c "..."）
        命令行里可能带着规则关键词，会把无关进程误归组。不属于 dexe 服务的进程只看自己的命令行。
        """
        if pid in self.cache and self.cache[pid][2] == procs[pid]["cmd"]:
            return self.cache[pid][0], self.cache[pid][1]
        unit = procs[pid]["unit"]
        argv = procs[pid]["cmd"].split()
        if not unit and len(argv) > 1 and os.path.basename(argv[0]) in SHELLS and argv[1] == "-c":
            # 服务外的 sh -c "..." 只是包装，关键词出现在它要执行的命令文本里，不算负载
            self.cache[pid] = (None, None, procs[pid]["cmd"])
            return None, None
        chain, cur, seen = [], pid, set()
        while cur in procs and cur not in seen and len(chain) < 40:
            if chain and not self.ancestor_any and (not unit or procs[cur]["unit"] != unit):
                break
            seen.add(cur)
            chain.append(cur)
            cur = procs[cur]["ppid"]
        res = None
        for p in chain:
            for g, c, rx in self.rules:
                if rx.search(procs[p]["cmd"]):
                    res = (g, c)
                    break
            if res:
                break
        if res is None:
            unit = procs[pid]["unit"]
            res = (f"dexe 未分类（{unit}）", "dexe 未分类") if unit else (None, None)
        self.cache[pid] = (res[0], res[1], procs[pid]["cmd"])
        return res


# ---------------------------------------------------------------- 统计

def pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def st(vals):
    if not vals:
        return None
    return {"avg": round(sum(vals) / len(vals), 1), "p95": round(pct(vals, 0.95), 1),
            "max": round(max(vals), 1), "n": len(vals)}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt res nav", description="导航资源占用监控")
    ap.add_argument("-o", "--out", help="输出目录")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=0, help="采样秒数，0 = 到 Ctrl+C 为止")
    ap.add_argument("--label", default="", help="工况说明，写进汇总")
    ap.add_argument("--rules", help="JSON 文件：[[组名, 类别, 正则], ...]，排在内置规则前面（先命中先用）")
    ap.add_argument("--no-default-rules", action="store_true", help="只用 --rules 里的规则")
    ap.add_argument("--list-rules", action="store_true")
    ap.add_argument("--ancestor-any", action="store_true",
                    help="调试用：不限 dexe 服务，任何进程都向上匹配祖先（开发机模拟测试时用）")
    args = ap.parse_args(argv)

    rules = [] if args.no_default_rules else list(DEFAULT_RULES)
    if args.rules:
        with open(args.rules) as f:
            rules = [tuple(r) for r in json.load(f)] + rules
    if args.list_rules:
        for g, c, p in rules:
            print(f"{c:8} | {g:24} | {p}")
        return
    if not args.out:
        ap.error("需要 -o 输出目录")
    os.makedirs(args.out, exist_ok=True)

    clf = Classifier(rules, args.ancestor_any)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    ncores = len(read_cores())
    fp = open(os.path.join(args.out, "proc_samples.csv"), "w", newline="")
    fg = open(os.path.join(args.out, "group_samples.csv"), "w", newline="")
    fc = open(os.path.join(args.out, "core_samples.csv"), "w", newline="")
    wp = csv.writer(fp)
    wg = csv.writer(fg)
    wc = csv.writer(fc)
    wp.writerow(["time", "pid", "group", "category", "cpu_pct", "rss_mb", "last_cpu", "affinity", "cmd"])
    wg.writerow(["time", "group", "category", "cpu_pct", "rss_mb", "nproc"])
    wc.writerow(["time"] + [f"core{i}" for i in range(ncores)] + ["mem_used_mb"])

    prev_ticks, prev_cores = {}, read_cores()
    hist_cat = {c: [] for c in CAT_ORDER}
    hist_cat_rss = {c: [] for c in CAT_ORDER}
    hist_group, hist_group_rss, group_cat, group_aff = {}, {}, {}, {}
    hist_algo, hist_drv = [], []
    hist_core = {i: [] for i in range(ncores)}
    proc_cpu, proc_info = {}, {}
    mem_hist = []
    mem_total = read_mem()[0]
    t0 = time.time()
    # 预热一轮，建立 CPU 差分基准
    procs = list_procs()
    for pid in procs:
        try:
            prev_ticks[pid] = (read_proc_stat(pid)[0], time.time())
        except (OSError, ValueError):
            pass
    print(f"[info] 共 {ncores} 个核，间隔 {args.interval}s；Ctrl+C 结束。输出：{args.out}")
    next_t = time.time() + args.interval
    while not stop["flag"]:
        time.sleep(max(0.0, next_t - time.time()))
        next_t += args.interval
        now = time.time()
        procs = list_procs()
        cat_cpu = {c: 0.0 for c in CAT_ORDER}
        cat_rss = {c: 0.0 for c in CAT_ORDER}
        grp = {}
        new_ticks = {}
        for pid, info in procs.items():
            if pid == os.getpid():
                continue
            g, c = clf.classify(pid, procs)
            try:
                ticks, last_cpu, rss = read_proc_stat(pid)
            except (OSError, ValueError):
                continue
            new_ticks[pid] = (ticks, now)
            if g is None:
                continue
            cpu = 0.0
            if pid in prev_ticks:
                pt, ptime = prev_ticks[pid]
                if now > ptime:
                    cpu = max(0.0, (ticks - pt) / CLK_TCK / (now - ptime) * 100.0)
            aff = read_affinity(pid)
            wp.writerow([round(now, 3), pid, g, c, round(cpu, 1), round(rss, 1), last_cpu, aff, info["cmd"][:200]])
            cat_cpu[c] += cpu
            cat_rss[c] += rss
            gg = grp.setdefault(g, [0.0, 0.0, 0])
            gg[0] += cpu
            gg[1] += rss
            gg[2] += 1
            group_cat[g] = c
            group_aff.setdefault(g, set()).add(aff)
            key = (pid, info["cmd"][:120])
            proc_cpu.setdefault(key, []).append(cpu)
            proc_info[key] = (g, aff)
        prev_ticks = new_ticks
        for g, (cpu, rss, n) in grp.items():
            wg.writerow([round(now, 3), g, group_cat[g], round(cpu, 1), round(rss, 1), n])
            hist_group.setdefault(g, []).append(cpu)
            hist_group_rss.setdefault(g, []).append(rss)
        for c in CAT_ORDER:
            hist_cat[c].append(cat_cpu[c])
            hist_cat_rss[c].append(cat_rss[c])
        algo = sum(cat_cpu[c] for c in ALGO_CATS)
        drv = algo + sum(cat_cpu[c] for c in DRIVER_CATS)
        hist_algo.append(algo)
        hist_drv.append(drv)
        cores = read_cores()
        row = [round(now, 3)]
        for i in range(ncores):
            tot, idle = cores.get(i, (0, 0))
            ptot, pidle = prev_cores.get(i, (0, 0))
            u = (1 - (idle - pidle) / (tot - ptot)) * 100.0 if tot > ptot else 0.0
            hist_core[i].append(u)
            row.append(round(u, 1))
        prev_cores = cores
        mem_used = read_mem()[1]
        mem_hist.append(mem_used)
        wc.writerow(row + [round(mem_used, 1)])
        for f in (fp, fg, fc):
            f.flush()
        el = now - t0
        if int(el) % 30 == 0:
            print(f"[{el:7.0f}s] 纯算法 {algo:6.1f}%  带驱动 {drv:6.1f}%  内存 {mem_used / mem_total * 100:5.1f}%")
        if args.duration and el >= args.duration:
            break
    for f in (fp, fg, fc):
        f.close()

    # ---------------- 汇总
    summary = {
        "label": args.label, "interval_s": args.interval, "samples": len(hist_algo), "ncores": ncores,
        "note": "CPU% 为单核百分比（100% = 占满 1 个核）；core 列为该核自身占用率",
        "pure_algorithm_cpu_pct": st(hist_algo),
        "with_drivers_cpu_pct": st(hist_drv),
        "categories": {c: {"cpu_pct": st(hist_cat[c]), "rss_mb_max": round(max(hist_cat_rss[c]), 1) if hist_cat_rss[c] else None}
                       for c in CAT_ORDER if any(hist_cat[c]) or any(hist_cat_rss[c])},
        "groups": {g: {"category": group_cat[g], "cpu_pct": st(v),
                       "rss_mb_max": round(max(hist_group_rss[g]), 1), "affinity": sorted(group_aff[g])}
                   for g, v in sorted(hist_group.items(), key=lambda kv: -sum(kv[1]))},
        "cores": {f"core{i}": st(v) for i, v in hist_core.items()},
        "mem_used_pct_max": round(max(mem_hist) / mem_total * 100, 1) if mem_hist else None,
        "top_processes": [
            {"pid": k[0], "cmd": k[1], "group": proc_info[k][0], "affinity": proc_info[k][1], "cpu_pct": st(v)}
            for k, v in sorted(proc_cpu.items(), key=lambda kv: -sum(kv[1]) / len(kv[1]))[:15]],
        "unclassified": sorted({k[1] for k in proc_cpu if proc_info[k][0].startswith("dexe 未分类")}),
    }
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    def fmt(s):
        return f"{s['avg']} / {s['p95']} / {s['max']}" if s else "—"
    md = [f"# 导航资源占用汇总 {args.label}", "",
          f"采样 {summary['samples']} 次，间隔 {args.interval} s，{ncores} 核；CPU% 为单核百分比。", "",
          "| 口径 | CPU% 平均 / P95 / 最大 |", "|---|---|",
          f"| 纯算法 | {fmt(summary['pure_algorithm_cpu_pct'])} |",
          f"| 带驱动 | {fmt(summary['with_drivers_cpu_pct'])} |", "",
          "| 类别 | CPU% 平均 / P95 / 最大 | 内存峰值 MB |", "|---|---|---|"]
    for c, v in summary["categories"].items():
        md.append(f"| {c} | {fmt(v['cpu_pct'])} | {v['rss_mb_max']} |")
    md += ["", "| 组 | 类别 | 绑核 | CPU% 平均 / P95 / 最大 | 内存峰值 MB |", "|---|---|---|---|---|"]
    for g, v in summary["groups"].items():
        md.append(f"| {g} | {v['category']} | {', '.join(a for a in v['affinity'] if a)} | {fmt(v['cpu_pct'])} | {v['rss_mb_max']} |")
    md += ["", "| 核 | 占用率 % 平均 / P95 / 最大 |", "|---|---|"]
    for k, v in summary["cores"].items():
        md.append(f"| {k} | {fmt(v)} |")
    if summary["unclassified"]:
        md += ["", "dexe 服务里没匹配到规则的进程（建议补规则）：", ""] + [f"- `{c}`" for c in summary["unclassified"]]
    with open(os.path.join(args.out, "summary.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()

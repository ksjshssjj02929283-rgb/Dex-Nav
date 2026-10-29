#!/usr/bin/env python3
"""长跑资源趋势监控（G-STB-02）：8 h～7×24 h 挂着跑，看内存、CPU 有没有持续增长，进程有没有重启。

进程归组规则与 nbt res nav 相同（共用 res_nav 模块）。只用 Python 标准库读 /proc，
不需要 ROS。为了长时间运行不撑爆磁盘，采样后按时间窗聚合再落盘：

  system_windows.csv  每个窗口一行：内存、swap、负载、各核平均占用、最高温度、监控目录剩余空间
  group_windows.csv   每个窗口每个组一行：RSS、CPU%、进程数、线程数、fd 数
  proc_windows.csv    每个进程窗口每个进程一行（导航相关进程 + dexe 未分类）
  events.csv          进程启动 / 退出 / 重启（同一组同一命令行换了 pid）
  summary.md / summary.json   每隔 --summary-every 秒重写一次，退出时再写一次；中途断电也留着最近一版

用法（在机器人上，长跑开始前启动，结束后 Ctrl+C）：
  nohup nbt res stb -o ~/stb_eval/R2_8h --label "8 h 标准长跑" \\
        --disk ~/.ros/log --disk ~/bags > ~/stb_eval/R2_8h.log 2>&1 &
  nbt res stb --report ~/stb_eval/R2_8h     # 只用已有 CSV 重新生成汇总

「疑似持续增长」只是提示，不是判据：斜率（预热之后按窗口均值线性拟合）超过 --flag-slope-mb-h
且拟合优度 R² ≥ --flag-r2，并且跨度 ≥ --flag-min-hours 才标出。判据阈值以 SOP 为准。
"""
import argparse
import csv
import json
import os
import shutil
import signal
import sys
import time
from collections import defaultdict

from nav_benchmark_tools import res_nav as nrm

TRACKED_CATS = set(nrm.ALGO_CATS + nrm.DRIVER_CATS + ["XWiz 后端", "dexe 未分类"])


# ---------------------------------------------------------------- 采样

def proc_detail(pid):
    """返回 (ticks, rss_mb, threads, fds 或 None)。"""
    with open(f"/proc/{pid}/stat") as f:
        s = f.read()
    fields = s[s.rindex(")") + 2:].split()
    ticks = int(fields[11]) + int(fields[12])
    threads = int(fields[17])
    rss_mb = int(fields[21]) * nrm.PAGE_KB / 1024.0
    try:
        fds = len(os.listdir(f"/proc/{pid}/fd"))
    except OSError:
        fds = None
    return ticks, rss_mb, threads, fds


def read_meminfo():
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":", 1)
            info[k] = int(v.split()[0])
    total = info["MemTotal"] / 1024.0
    used = total - info.get("MemAvailable", info["MemFree"]) / 1024.0
    swap = (info.get("SwapTotal", 0) - info.get("SwapFree", 0)) / 1024.0
    return total, used, swap


def read_temp():
    best = None
    base = "/sys/class/thermal"
    try:
        for d in os.listdir(base):
            if d.startswith("thermal_zone"):
                try:
                    with open(f"{base}/{d}/temp") as f:
                        t = int(f.read().strip()) / 1000.0
                    best = t if best is None else max(best, t)
                except (OSError, ValueError):
                    pass
    except OSError:
        pass
    return best


def mean(v):
    v = [x for x in v if x is not None]
    return sum(v) / len(v) if v else None


def r(x, n=1):
    return None if x is None else round(x, n)


# ---------------------------------------------------------------- 监控主体

class Monitor:
    def __init__(self, args, classifier):
        self.a = args
        self.cls = classifier
        os.makedirs(args.out, exist_ok=True)
        self.files = {}
        self.writers = {}
        self._open("system", "system_windows.csv",
                   ["t_start", "t_end", "elapsed_h", "n", "mem_total_mb", "mem_used_mb_avg", "mem_used_mb_max",
                    "swap_used_mb_max", "load1_avg", "cpu_all_avg", "cpu_core_max_avg", "temp_c_max",
                    "self_cpu_pct", "self_rss_mb"] + [f"disk_free_gb:{p}" for p in args.disk])
        self._open("group", "group_windows.csv",
                   ["t_start", "t_end", "elapsed_h", "group", "category", "n", "rss_mb_avg", "rss_mb_max",
                    "cpu_avg", "cpu_max", "nproc_avg", "threads_avg", "fds_avg"])
        self._open("proc", "proc_windows.csv",
                   ["t_start", "t_end", "elapsed_h", "group", "category", "pid", "cmd", "n", "rss_mb_avg",
                    "rss_mb_max", "cpu_avg", "threads_avg", "fds_avg"])
        self._open("events", "events.csv", ["time", "elapsed_h", "event", "group", "pid", "old_pid", "cmd"])
        self.t0 = time.time()
        self.prev_ticks = {}
        self.prev_cores = nrm.read_cores()
        self.prev_t = None
        self.known = {}          # pid -> (group, cat, cmd, 首次看到的时间, 是否开跑时就在)
        self.vanished = {}       # (group, cmd) -> (pid, time)  退出后等一会儿看是不是重启
        self.transient = 0       # 活不到 --transient-sec 就退出的短命进程（脚本里的 sleep、CLI 调用等），不记事件
        self.reset_win()
        self.reset_pwin()
        self.self_prev = None
        self.last_summary = time.time()

    def _open(self, key, name, header):
        path = os.path.join(self.a.out, name)
        new = not os.path.exists(path) or os.path.getsize(path) == 0
        f = open(path, "a", newline="")
        w = csv.writer(f)
        if new:
            w.writerow(header)
            f.flush()
        self.files[key], self.writers[key] = f, w

    def reset_win(self):
        self.win = {"t_start": time.time(), "n": 0, "mem": [], "swap": [], "load": [], "cpu_all": [],
                    "core_max": [], "temp": [], "groups": defaultdict(lambda: defaultdict(list)), "cat": {}}

    def reset_pwin(self):
        self.pwin = {"t_start": time.time(), "procs": defaultdict(lambda: defaultdict(list)), "meta": {}}

    def elapsed_h(self, t=None):
        return round(((t or time.time()) - self.t0) / 3600.0, 4)

    def event(self, ev, group, pid, old_pid, cmd):
        t = time.time()
        self.writers["events"].writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)), self.elapsed_h(t),
                                         ev, group, pid, old_pid, cmd[:200]])
        self.files["events"].flush()
        if ev != "start" or self.a.verbose:
            print(f"[{time.strftime('%H:%M:%S')}] {ev}: {group} pid={pid} old={old_pid} {cmd[:80]}", flush=True)

    def sample(self, first=False):
        now = time.time()
        procs = nrm.list_procs()
        own = os.getpid()
        dt = (now - self.prev_t) if self.prev_t else None
        cur = {}
        for pid, info in procs.items():
            if pid == own:
                continue
            g, c = self.cls.classify(pid, procs)
            if g is None or c not in TRACKED_CATS:
                continue
            try:
                ticks, rss, thr, fds = proc_detail(pid)
            except (OSError, ValueError, IndexError):
                continue
            cpu = None
            if dt and pid in self.prev_ticks:
                cpu = max(0.0, (ticks - self.prev_ticks[pid]) / nrm.CLK_TCK / dt * 100.0)
            self.prev_ticks[pid] = ticks
            cur[pid] = (g, c, info["cmd"], rss, cpu, thr, fds)
        # 进程启动 / 退出 / 重启
        if first:
            for pid, (g, c, cmd, *_r) in cur.items():
                self.known[pid] = [g, c, cmd, now, True]
        else:
            for pid in [p for p in self.known if p not in cur]:
                g, c, cmd, t_seen, logged = self.known.pop(pid)
                self.prev_ticks.pop(pid, None)
                if logged:
                    self.vanished[(g, cmd)] = (pid, now)
                else:
                    self.transient += 1
            for pid, (g, c, cmd, *_r) in cur.items():
                k = self.known.get(pid)
                if k is None:
                    self.known[pid] = [g, c, cmd, now, False]
                    k = self.known[pid]
                if k[4] or now - k[3] < self.a.transient_sec:
                    continue
                k[4] = True      # 活过了 --transient-sec，才算一个正式进程
                old = self.vanished.get((g, cmd))
                if old and k[3] - old[1] <= self.a.restart_grace:   # 按新进程首次出现的时间判重启
                    del self.vanished[(g, cmd)]
                    self.event("restart", g, pid, old[0], cmd)
                else:
                    self.event("start", g, pid, "", cmd)
            for key, (pid, t) in list(self.vanished.items()):
                # 新进程要先活过 --transient-sec 才会被确认，所以多等这一段再判「退出未重启」
                if now - t >= self.a.restart_grace + self.a.transient_sec + self.a.interval:
                    self.event("exit", key[0], "", pid, key[1])
                    del self.vanished[key]
        # 系统
        total, used, swap = read_meminfo()
        cores = nrm.read_cores()
        busy = []
        for k, (tot, idle) in cores.items():
            if k in self.prev_cores:
                d_t, d_i = tot - self.prev_cores[k][0], idle - self.prev_cores[k][1]
                if d_t > 0:
                    busy.append(100.0 * (d_t - d_i) / d_t)
        self.prev_cores = cores
        w = self.win
        w["n"] += 1
        w["mem_total"] = total
        w["mem"].append(used)
        w["swap"].append(swap)
        w["load"].append(os.getloadavg()[0])
        if busy:
            w["cpu_all"].append(sum(busy) / len(busy))
            w["core_max"].append(max(busy))
        w["temp"].append(read_temp())
        # 组和进程
        gsum = defaultdict(lambda: [0.0, 0.0, 0, 0, 0, False])
        for pid, (g, c, cmd, rss, cpu, thr, fds) in cur.items():
            s = gsum[g]
            s[0] += rss
            s[1] += cpu or 0.0
            s[2] += 1
            s[3] += thr
            s[4] += fds or 0
            s[5] = s[5] or cpu is not None
            w["cat"][g] = c
            pw = self.pwin["procs"][pid]
            pw["rss"].append(rss)
            pw["cpu"].append(cpu)
            pw["thr"].append(thr)
            pw["fds"].append(fds)
            self.pwin["meta"][pid] = (g, c, cmd)
        for g, (rss, cpu, n, thr, fds, has_cpu) in gsum.items():
            gw = w["groups"][g]
            gw["rss"].append(rss)
            gw["cpu"].append(cpu if has_cpu else None)
            gw["n"].append(n)
            gw["thr"].append(thr)
            gw["fds"].append(fds)
        self.prev_t = now

    def self_usage(self):
        ticks, rss, _thr, _fds = proc_detail(os.getpid())
        t = time.time()
        cpu = None
        if self.self_prev:
            cpu = (ticks - self.self_prev[0]) / nrm.CLK_TCK / max(1e-6, t - self.self_prev[1]) * 100.0
        self.self_prev = (ticks, t)
        return cpu, rss

    def flush_window(self):
        w = self.win
        if w["n"] == 0:
            return
        t1 = time.time()
        ts, te = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(x)) for x in (w["t_start"], t1))
        eh = self.elapsed_h((w["t_start"] + t1) / 2)
        cpu_self, rss_self = self.self_usage()
        disks = []
        for p in self.a.disk:
            try:
                disks.append(r(shutil.disk_usage(p).free / 1e9, 2))
            except OSError:
                disks.append(None)
        temps = [x for x in w["temp"] if x is not None]
        self.writers["system"].writerow(
            [ts, te, eh, w["n"], r(w["mem_total"]), r(mean(w["mem"])), r(max(w["mem"])), r(max(w["swap"])),
             r(mean(w["load"]), 2), r(mean(w["cpu_all"])), r(mean(w["core_max"])), r(max(temps)) if temps else None,
             r(cpu_self, 2), r(rss_self)] + disks)
        for g, gw in w["groups"].items():
            cpus = [x for x in gw["cpu"] if x is not None]
            self.writers["group"].writerow(
                [ts, te, eh, g, w["cat"].get(g, ""), len(gw["rss"]), r(mean(gw["rss"])), r(max(gw["rss"])),
                 r(mean(cpus)), r(max(cpus)) if cpus else None, r(mean(gw["n"])), r(mean(gw["thr"])),
                 r(mean(gw["fds"]))])
        self.files["system"].flush()
        self.files["group"].flush()
        self.reset_win()

    def flush_proc_window(self):
        pw = self.pwin
        if not pw["procs"]:
            return
        t1 = time.time()
        ts, te = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(x)) for x in (pw["t_start"], t1))
        eh = self.elapsed_h((pw["t_start"] + t1) / 2)
        for pid, d in pw["procs"].items():
            g, c, cmd = pw["meta"][pid]
            self.writers["proc"].writerow([ts, te, eh, g, c, pid, cmd[:200], len(d["rss"]), r(mean(d["rss"])),
                                           r(max(d["rss"])), r(mean(d["cpu"])), r(mean(d["thr"])),
                                           r(mean(d["fds"]))])
        self.files["proc"].flush()
        self.reset_pwin()

    def close(self):
        self.flush_window()
        self.flush_proc_window()
        for f in self.files.values():
            f.close()


# ---------------------------------------------------------------- 汇总（只读 CSV，--report 与运行中共用）

def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def linfit(xs, ys):
    pts = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pts) < 3:
        return None, None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx <= 0:
        return None, None
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    k = sxy / sxx
    ss_tot = sum((p[1] - my) ** 2 for p in pts)
    ss_res = sum((p[1] - (my + k * (p[0] - mx))) ** 2 for p in pts)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return k, r2


def trend(rows, key, bucket_h, warmup_h):
    """rows: [(elapsed_h, value)]；返回首 / 末时段均值、斜率（/h）、R²、跨度。"""
    rows = [(h, v) for h, v in rows if h is not None and v is not None]
    if not rows:
        return {}
    h_end = max(h for h, _ in rows)
    first = [v for h, v in rows if warmup_h <= h < warmup_h + bucket_h] or [v for h, v in rows if h < bucket_h]
    last = [v for h, v in rows if h > h_end - bucket_h]
    fit = [(h, v) for h, v in rows if h >= warmup_h]
    k, r2 = linfit([h for h, _ in fit], [v for _, v in fit])
    span = (max(h for h, _ in fit) - min(h for h, _ in fit)) if fit else 0.0
    out = {"first": r(mean(first), 1), "last": r(mean(last), 1), "max": r(max(v for _, v in rows), 1),
           "slope_per_h": r(k, 3), "r2": r(r2, 3), "span_h": r(span, 2)}
    if out["first"] is not None and out["last"] is not None:
        out["delta"] = r(out["last"] - out["first"], 1)
    return out


def flagged(t, a):
    return (t.get("slope_per_h") is not None and t["slope_per_h"] > a.flag_slope_mb_h
            and (t.get("r2") or 0) >= a.flag_r2 and (t.get("span_h") or 0) >= a.flag_min_hours)


def build_summary(a, label=""):
    out = a.out
    bucket_h = a.bucket_sec / 3600.0
    warm = a.warmup_min / 60.0

    def load(name):
        p = os.path.join(out, name)
        if not os.path.exists(p):
            return []
        with open(p, newline="") as f:
            return list(csv.DictReader(f))

    sysw, grpw, prw, evs = (load(n) for n in ("system_windows.csv", "group_windows.csv", "proc_windows.csv",
                                               "events.csv"))
    if not sysw:
        print("还没有数据窗口", file=sys.stderr)
        return None
    run_h = max(fnum(x["elapsed_h"]) or 0 for x in sysw)
    res = {"label": label, "out": out, "start": sysw[0]["t_start"], "end": sysw[-1]["t_end"],
           "run_h": r(run_h, 2), "bucket_h": r(bucket_h, 3), "warmup_h": r(warm, 2),
           "flag_rule": f"斜率 > {a.flag_slope_mb_h} MB/h 且 R² ≥ {a.flag_r2} 且跨度 ≥ {a.flag_min_hours} h（提示用，非判据）"}
    pts = lambda rows, k: [(fnum(x["elapsed_h"]), fnum(x[k])) for x in rows]  # noqa: E731
    res["system"] = {"mem_used_mb": trend(pts(sysw, "mem_used_mb_avg"), "", bucket_h, warm),
                     "swap_used_mb_max": trend(pts(sysw, "swap_used_mb_max"), "", bucket_h, warm),
                     "cpu_all": trend(pts(sysw, "cpu_all_avg"), "", bucket_h, warm),
                     "temp_c_max": trend(pts(sysw, "temp_c_max"), "", bucket_h, warm),
                     "self_cpu_pct": trend(pts(sysw, "self_cpu_pct"), "", bucket_h, 0),
                     "mem_total_mb": fnum(sysw[-1]["mem_total_mb"])}
    disks = {}
    for k in sysw[0].keys():
        if k.startswith("disk_free_gb:"):
            t = trend(pts(sysw, k), "", bucket_h, 0)
            if t.get("slope_per_h") and t["slope_per_h"] < 0 and t.get("last") is not None:
                t["hours_to_full"] = r(t["last"] / -t["slope_per_h"], 1)
            disks[k.split(":", 1)[1]] = t
    res["disk_free_gb"] = disks
    groups = defaultdict(list)
    cats = {}
    for x in grpw:
        groups[x["group"]].append(x)
        cats[x["group"]] = x["category"]
    ev_count = defaultdict(lambda: defaultdict(int))
    for e in evs:
        ev_count[e["group"]][e["event"]] += 1
    gres = {}
    for g, rows in groups.items():
        gres[g] = {"category": cats[g], "rss_mb": trend(pts(rows, "rss_mb_avg"), "", bucket_h, warm),
                   "cpu": trend(pts(rows, "cpu_avg"), "", bucket_h, warm),
                   "threads": trend(pts(rows, "threads_avg"), "", bucket_h, warm),
                   "fds": trend(pts(rows, "fds_avg"), "", bucket_h, warm),
                   "restarts": ev_count[g]["restart"], "exits": ev_count[g]["exit"]}
        gres[g]["rss_flag"] = flagged(gres[g]["rss_mb"], a)
    res["groups"] = gres
    procs = defaultdict(list)
    meta = {}
    for x in prw:
        key = (x["group"], x["pid"])
        procs[key].append(x)
        meta[key] = (x["category"], x["cmd"])
    pres = []
    for (g, pid), rows in procs.items():
        t = trend(pts(rows, "rss_mb_avg"), "", bucket_h, warm)
        if not t:
            continue
        pres.append({"group": g, "pid": pid, "cmd": meta[(g, pid)][1][:120], "rss_mb": t,
                     "cpu": trend(pts(rows, "cpu_avg"), "", bucket_h, warm),
                     "fds": trend(pts(rows, "fds_avg"), "", bucket_h, warm), "flag": flagged(t, a)})
    pres.sort(key=lambda p: -(p["rss_mb"].get("slope_per_h") or -1e9))
    for p in pres:
        if p["flag"] and p["group"] in gres:
            gres[p["group"]]["rss_flag"] = True   # 组内进程重启后内存回落，组曲线会掩盖单进程的增长
    res["procs_top_growth"] = pres[:20]
    res["events"] = [e for e in evs if e["event"] != "start"][-50:]
    res["restart_total"] = sum(v["restart"] for v in ev_count.values())
    res["exit_total"] = sum(v["exit"] for v in ev_count.values())
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    write_md(res)
    return res


def fmt(t, k):
    v = t.get(k) if t else None
    return "—" if v is None else str(v)


def write_md(res):
    L = [f"# 长跑资源趋势汇总{('：' + res['label']) if res['label'] else ''}", "",
         f"- 时段：{res['start']} → {res['end']}，共 {res['run_h']} h；首 / 末时段长 {res['bucket_h']} h；"
         f"斜率从第 {res['warmup_h']} h 起按窗口均值线性拟合",
         f"- 「疑似持续增长」标记规则：{res['flag_rule']}",
         f"- 进程重启 {res['restart_total']} 次，退出未重启 {res['exit_total']} 次（明细见下文和 events.csv）",
         f"- 监控自身 CPU 平均 {fmt(res['system']['self_cpu_pct'], 'first')} %", "",
         "## 系统", "",
         "| 指标 | 首时段 | 末时段 | 变化 | 最大 | 斜率 /h | R² |", "|---|---|---|---|---|---|---|"]
    names = {"mem_used_mb": "已用内存 (MB)", "swap_used_mb_max": "swap (MB)", "cpu_all": "全核平均 CPU (%)",
             "temp_c_max": "最高温度 (°C)"}
    for k, n in names.items():
        t = res["system"][k]
        L.append(f"| {n} | {fmt(t, 'first')} | {fmt(t, 'last')} | {fmt(t, 'delta')} | {fmt(t, 'max')} | "
                 f"{fmt(t, 'slope_per_h')} | {fmt(t, 'r2')} |")
    for p, t in res["disk_free_gb"].items():
        L.append(f"| 剩余空间 {p} (GB) | {fmt(t, 'first')} | {fmt(t, 'last')} | {fmt(t, 'delta')} | — | "
                 f"{fmt(t, 'slope_per_h')} | 按当前速度约 {fmt(t, 'hours_to_full')} h 写满 |")
    L += ["", "## 按组（G-STB-02 记录表）", "",
          "> 组内进程重启后内存会回落，组的「变化」「斜率」看不出这种泄漏；「疑似持续增长」按组和组内每个进程分别拟合，"
          "任一命中即标出，细节看下一节的进程行和重启记录。", "",
          "| # | 组 | 类别 | 第 1 时段内存均值 (MB) | 最后时段内存均值 (MB) | 变化 | 内存斜率 (MB/h) | R² | "
          "第 1 时段 CPU 均值 (%) | 最后时段 CPU 均值 (%) | fd 变化 | 重启 / 退出 | 疑似持续增长 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    order = {c: i for i, c in enumerate(nrm.CAT_ORDER)}
    for i, (g, v) in enumerate(sorted(res["groups"].items(), key=lambda kv: (order.get(kv[1]["category"], 99), kv[0])), 1):
        L.append(f"| {i} | {g} | {v['category']} | {fmt(v['rss_mb'], 'first')} | {fmt(v['rss_mb'], 'last')} | "
                 f"{fmt(v['rss_mb'], 'delta')} | {fmt(v['rss_mb'], 'slope_per_h')} | {fmt(v['rss_mb'], 'r2')} | "
                 f"{fmt(v['cpu'], 'first')} | {fmt(v['cpu'], 'last')} | {fmt(v['fds'], 'delta')} | "
                 f"{v['restarts']} / {v['exits']} | {'⚠️ 是' if v['rss_flag'] else '否'} |")
    L += ["", "## 内存斜率最大的进程（前 20）", "",
          "| 组 | pid | 首时段 (MB) | 末时段 (MB) | 斜率 (MB/h) | R² | 跨度 (h) | 疑似持续增长 | 命令行 |",
          "|---|---|---|---|---|---|---|---|---|"]
    for p in res["procs_top_growth"]:
        t = p["rss_mb"]
        L.append(f"| {p['group']} | {p['pid']} | {fmt(t, 'first')} | {fmt(t, 'last')} | {fmt(t, 'slope_per_h')} | "
                 f"{fmt(t, 'r2')} | {fmt(t, 'span_h')} | {'⚠️ 是' if p['flag'] else '否'} | `{p['cmd'][:80]}` |")
    if res["events"]:
        L += ["", "## 进程重启与退出", "", "| 时间 | 运行 (h) | 事件 | 组 | pid | 原 pid | 命令行 |",
              "|---|---|---|---|---|---|---|"]
        for e in res["events"]:
            L.append(f"| {e['time']} | {e['elapsed_h']} | {e['event']} | {e['group']} | {e['pid']} | "
                     f"{e['old_pid']} | `{e['cmd'][:60]}` |")
    with open(os.path.join(res["out"], "summary.md"), "w") as f:
        f.write("\n".join(L) + "\n")


# ---------------------------------------------------------------- 入口

def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt res stb", description="长跑资源趋势监控（G-STB-02）")
    ap.add_argument("-o", "--out", help="输出目录（已存在则在原文件后追加）")
    ap.add_argument("--report", metavar="DIR", help="不采样，只用 DIR 里已有的 CSV 重新生成汇总")
    ap.add_argument("--label", default="", help="工况说明，写进汇总")
    ap.add_argument("--interval", type=float, default=10.0, help="采样间隔 s（默认 10）")
    ap.add_argument("--window", type=float, default=60.0, help="组和系统的聚合窗口 s（默认 60）")
    ap.add_argument("--proc-window", type=float, default=300.0, help="按进程的聚合窗口 s（默认 300）")
    ap.add_argument("--summary-every", type=float, default=3600.0, help="每隔多少秒重写一次汇总（默认 3600）")
    ap.add_argument("--duration", type=float, default=0, help="采样秒数，0 = 到 Ctrl+C / kill 为止")
    ap.add_argument("--disk", action="append", default=[], help="监控剩余空间的目录（可多次给）")
    ap.add_argument("--restart-grace", type=float, default=30.0,
                    help="进程消失后多少秒内出现同组同命令行的新进程算重启（默认 30）")
    ap.add_argument("--transient-sec", type=float, default=60.0,
                    help="活不到这么久就退出的进程算短命进程，不记启动 / 退出 / 重启（默认 60）")
    ap.add_argument("--bucket-sec", type=float, default=3600.0, help="首 / 末时段长度 s（默认 3600 = 1 h）")
    ap.add_argument("--warmup-min", type=float, default=30.0, help="拟合斜率时跳过开头多少分钟（默认 30）")
    ap.add_argument("--flag-slope-mb-h", type=float, default=2.0, help="疑似增长提示：内存斜率阈值 MB/h")
    ap.add_argument("--flag-r2", type=float, default=0.7, help="疑似增长提示：拟合优度阈值")
    ap.add_argument("--flag-min-hours", type=float, default=2.0, help="疑似增长提示：拟合跨度至少多少小时")
    ap.add_argument("--rules", help="追加分组规则 JSON，格式同 nbt res nav")
    ap.add_argument("--ancestor-any", action="store_true", help="同 nbt res nav")
    ap.add_argument("-v", "--verbose", action="store_true", help="进程启动事件也打印")
    a = ap.parse_args(argv)

    if a.report:
        a.out = a.report
        meta = {}
        mp = os.path.join(a.out, "run_meta.json")
        if os.path.exists(mp):
            with open(mp) as f:
                meta = json.load(f)
        res = build_summary(a, meta.get("label", ""))
        if res:
            print(open(os.path.join(a.out, "summary.md")).read())
        return
    if not a.out:
        a.out = os.path.expanduser(time.strftime("~/stb_eval/stb_%Y%m%d_%H%M%S"))
    rules = list(nrm.DEFAULT_RULES)
    if a.rules:
        with open(a.rules) as f:
            rules = [tuple(x) for x in json.load(f)] + rules
    a.disk = [os.path.expanduser(p) for p in a.disk]
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "run_meta.json"), "w") as f:
        json.dump({"label": a.label, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a)}, f,
                  ensure_ascii=False, indent=2)

    mon = Monitor(a, nrm.Classifier(rules, a.ancestor_any))
    stop = {"flag": False}

    def on_signal(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print(f"长跑资源监控开始：输出 {a.out}，采样 {a.interval} s，窗口 {a.window} / {a.proc_window} s", flush=True)
    mon.sample(first=True)
    t_start = time.time()
    next_t = t_start + a.interval
    while not stop["flag"]:
        time.sleep(max(0.0, min(1.0, next_t - time.time())))
        if time.time() < next_t:
            continue
        next_t += a.interval
        mon.sample()
        now = time.time()
        if now - mon.win["t_start"] >= a.window:
            mon.flush_window()
        if now - mon.pwin["t_start"] >= a.proc_window:
            mon.flush_proc_window()
        if now - mon.last_summary >= a.summary_every:
            build_summary(a, a.label)
            mon.last_summary = now
            print(f"[{time.strftime('%H:%M:%S')}] 汇总已更新（运行 {mon.elapsed_h():.2f} h）", flush=True)
        if a.duration and now - t_start >= a.duration:
            break
    mon.close()
    if mon.transient:
        print(f"短命进程 {mon.transient} 个（活不到 {a.transient_sec} s，未记事件）", flush=True)
    res = build_summary(a, a.label)
    if res:
        print(open(os.path.join(a.out, "summary.md")).read())


if __name__ == "__main__":
    main()

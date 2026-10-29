"""速度曲线叠加图与各路段实测速度（speed analyze 调用）。

每个任务、每一圈一条曲线，画在同一张图里看执行差异；横轴可切「沿实走路线的路程 / 任务内时间」，
纵轴可切里程计、跟踪器、平滑器、底盘指令四级速度。关键路段单独一张（横轴为沿路段的距离）。
各路段实测速度：按车依次经过点序列的时刻把任务切成路段，统计每段的最高速度和在目标 ±5% 内的持续时间，
用来确认哪些路段真的能跑到目标速度。
"""
import json
import math
import os

from nav_benchmark_tools import navlog as nl
from nav_benchmark_tools import pathplan

LEVEL_KEYS = ["odom", "follower", "smoother", "mux"]


def pass_times(pose, t0, t1, names, targets, near=0.6):
    """车依次经过点序列各点的时刻：首点 = t0，末点 = t1；中间点取窗口内依次第一次最靠近（< near m）的时刻。
    缺点位坐标或没经过时返回 None。"""
    seq = nl.window(pose, t0, t1)
    times, k = [t0], 0
    for name in names[1:-1]:
        tg = targets.get(name)
        if tg is None:
            return None
        best, best_d = None, near
        for i in range(k, len(seq)):
            d = math.hypot(seq[i][1] - tg["x"], seq[i][2] - tg["y"])
            if d < best_d:
                best, best_d = i, d
            elif best is not None and d > best_d + 0.1:
                break
        if best is None:
            return None
        times.append(seq[best][0])
        k = best
    times.append(t1)
    return times


def leg_rows(d, names, t0, t1, targets, target, band_time):
    """按路段统计：[{from, to, max_v, steady, in_band}]；切不出来返回 []。"""
    ts = pass_times(d["pose"], t0, t1, names, targets)
    if not ts:
        return []
    out = []
    for (a, b), ta, tb in zip(zip(names, names[1:]), ts, ts[1:]):
        od = nl.window(d["odom"], ta, tb)
        best, total = band_time(od, target) if target else (None, None)
        out.append({"from": a, "to": b, "max_v": max((abs(o[1]) for o in od), default=None), "steady": best,
                    "in_band": total, "dt": tb - ta})
    return out


def aggregate_legs(rows, target):
    """各圈的路段行 → 每个路段一行：样本、最高速度 均值 / 最小、±5% 最长连续 均值 / 最小、达到目标的圈数。"""
    by = {}
    for r in rows:
        by.setdefault((r["from"], r["to"]), []).append(r)
    out = []
    for (a, b), rs in by.items():
        mv = [r["max_v"] for r in rs if r["max_v"] is not None]
        st = [r["steady"] for r in rs if r["steady"] is not None]
        reach = sum(1 for v in mv if target and v >= target * 0.95)
        hold = sum(1 for v in st if v >= 2.0)
        out.append({"from": a, "to": b, "n": len(rs), "max_v_mean": sum(mv) / len(mv) if mv else None,
                    "max_v_min": min(mv) if mv else None, "steady_mean": sum(st) / len(st) if st else None,
                    "steady_min": min(st) if st else None, "reach": reach, "hold": hold})
    return sorted(out, key=lambda x: -(x["steady_mean"] or 0.0))


def _step_sample(series, grid):
    """series [(t, v, ...)] 按时间取不晚于 grid 各点的最近值（零阶保持）。"""
    out, i, last = [], 0, None
    for g in grid:
        while i < len(series) and series[i][0] <= g:
            last = series[i][1]
            i += 1
        out.append(round(abs(last), 4) if last is not None else None)
    return out


def curve_series(d, t0, t1, hz=10.0):
    """任务窗内 10 Hz 的曲线数据：t（相对受理）、s（沿实走路线的累计路程）、四级速度（取绝对值）。"""
    n = max(2, int((t1 - t0) * hz) + 1)
    grid = [t0 + k / hz for k in range(n)]
    pose = nl.window(d["pose"], t0, t1)
    cum, acc, prev = [], 0.0, None
    for p in pose:
        if prev is not None:
            acc += math.hypot(p[1] - prev[1], p[2] - prev[2])
        cum.append((p[0], acc))
        prev = p
    s = _step_sample(cum, grid) if cum else [None] * n
    return {"t": [round(g - t0, 2) for g in grid], "s": [round(x, 3) if x is not None else None for x in s],
            "v": {k: _step_sample(nl.window(d["levels"][k], t0 - 0.5, t1), grid) for k in LEVEL_KEYS}}


def segment_series(d, ti, to, a, b, hz=10.0):
    """关键路段窗内的曲线：s 为沿 a→b 的投影距离（从路段起点算）。"""
    c = curve_series(d, ti, to, hz)
    L = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) or 1.0
    ux, uy = (b["x"] - a["x"]) / L, (b["y"] - a["y"]) / L
    proj = [(p[0], (p[1] - a["x"]) * ux + (p[2] - a["y"]) * uy) for p in nl.window(d["pose"], ti, to)]
    grid = [ti + k / hz for k in range(len(c["t"]))]
    s, i, last = [], 0, None
    for g in grid:
        while i < len(proj) and proj[i][0] <= g:
            last = proj[i][1]
            i += 1
        s.append(round(last, 3) if last is not None else None)
    c["s"] = s
    return c


def curves_template_path():
    p = pathplan.template_path()
    return os.path.join(os.path.dirname(p), "curves_template.html")


def render_html(data, template=None):
    with open(template or curves_template_path(), encoding="utf-8") as f:
        html = f.read()
    tag = '<script id="data" type="application/json">'
    i = html.index(tag) + len(tag)
    j = html.index("</script>", i)
    return html[:i] + json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/") + html[j:]


def write_pngs(data, out_dir):
    """可选：每个任务、关键路段各一张 PNG（里程计速度 - 路程），给报告用；没装 matplotlib 返回 None。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    for fam in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei", "SimHei"):
        try:
            from matplotlib import font_manager
            font_manager.findfont(fam, fallback_to_default=False)
            plt.rcParams["font.sans-serif"] = [fam]
            break
        except Exception:  # noqa: BLE001  没这个字体就试下一个
            continue
    plt.rcParams["axes.unicode_minus"] = False
    files, target = [], data.get("target")
    charts = [(f"任务 {t['id']}：{' → '.join(t['points'])}", t["runs"], f"curves_{t['id']}.png") for t in data["tasks"]]
    for k, sg in enumerate(data.get("segments") or []):
        charts.append((f"关键路段 {sg['from']} → {sg['to']}", sg["runs"], f"curves_segment_{k + 1}.png"))
    for title, runs, name in charts:
        fig, ax = plt.subplots(figsize=(10, 4))
        n = max(1, len(runs) - 1)
        for k, r in enumerate(runs):
            xs = [x for x, v in zip(r["s"], r["v"]["odom"]) if x is not None and v is not None]
            vs = [v for x, v in zip(r["s"], r["v"]["odom"]) if x is not None and v is not None]
            ax.plot(xs, vs, color=plt.cm.viridis(k / n), lw=1.0, alpha=0.7,
                    ls="--" if r.get("mark") in ("不合格", "受干扰") else "-")
        if target:
            ax.axhspan(target * 0.95, target * 1.05, color="green", alpha=0.12)
        ax.set_title(f"{title}（{len(runs)} 次，颜色由紫到黄为圈次先后）")
        ax.set_xlabel("路程 (m)")
        ax.set_ylabel("里程计速度 (m/s)")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        path = os.path.join(out_dir, name)
        fig.savefig(path, dpi=120)
        plt.close(fig)
        files.append(path)
    return files

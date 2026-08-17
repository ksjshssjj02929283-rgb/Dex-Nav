#!/usr/bin/env python3
"""Visualize preprocessed map: occupancy, distance field, 3-layer node graphs."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

try:
    from scipy.ndimage import distance_transform_edt as _edt
except ImportError:
    _edt = None


def expand_user(path: str) -> str:
    return os.path.expanduser(path)


def load_image_grid(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".png":
        try:
            from PIL import Image
        except ImportError as exc:
            raise ImportError("Install Pillow for PNG maps: pip install pillow") from exc
        img = Image.open(path).convert("L")
        return np.flipud(np.array(img, dtype=np.uint8))
    return load_pgm(path)


def load_map_yaml(map_yaml: Path) -> tuple[np.ndarray, dict]:
    with open(map_yaml, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img_path = map_yaml.parent / meta["image"]
    if not img_path.exists():
        img_path = Path(meta["image"])
    data = load_image_grid(img_path)
    return data, meta


def _read_pgm_token_line(f) -> bytes:
    """Read next non-empty PGM header line, skipping # comments."""
    while True:
        line = f.readline()
        if not line:
            raise EOFError("Unexpected EOF in PGM header")
        if b"#" in line:
            line = line.split(b"#", 1)[0]
        line = line.strip()
        if line:
            return line


def load_pgm(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        magic = _read_pgm_token_line(f)
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM magic: {magic!r}")
        w, h = map(int, _read_pgm_token_line(f).split()[:2])
        maxval = int(_read_pgm_token_line(f))
        if maxval <= 0 or maxval > 255:
            raise ValueError(f"Unsupported PGM max value: {maxval}")
        if magic == b"P5":
            raw = f.read(w * h)
            if len(raw) != w * h:
                raise ValueError(
                    f"PGM {path}: expected {w * h} bytes, got {len(raw)}"
                )
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
        else:
            arr = np.loadtxt(f, dtype=np.uint8, max_rows=h)
            arr = arr.reshape(h, w)
    return np.flipud(arr)


def load_distance_map(yaml_path: Path) -> tuple[np.ndarray, dict]:
    with open(yaml_path, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    pgm = yaml_path.parent / meta["image"]
    norm = load_pgm(pgm).astype(np.float32) / 254.0
    dist = norm * float(meta.get("max_distance_m", 2.0))
    return dist, meta


def load_node_map(yaml_path: Path) -> dict:
    with open(yaml_path, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    nodes = []
    if meta.get("adjacency_mode") == "grid8":
        mask_path = yaml_path.parent / meta.get("node_mask_image", "nodemap_raw_mask.pgm")
        mask = load_pgm(mask_path)
        h, w = mask.shape
        res = float(meta["resolution"])
        ox, oy = meta["origin"][:2]
        nid = 0
        for gy in range(h):
            for gx in range(w):
                if mask[gy, gx] < 128:
                    continue
                nodes.append({
                    "id": nid,
                    "gx": gx,
                    "gy": gy,
                    "x": ox + (gx + 0.5) * res,
                    "y": oy + (gy + 0.5) * res,
                })
                nid += 1
        meta["nodes"] = nodes
        meta["adjacency"] = []
    else:
        meta.setdefault("nodes", [])
        meta.setdefault("adjacency", [])
    return meta


def occupancy_probability(pgm: np.ndarray, meta: dict) -> np.ndarray:
    """Match nav2 map_io trinary: occ probability per pixel in [0, 1]."""
    shade = pgm.astype(np.float64) / 255.0
    if meta.get("negate", 0):
        return shade
    return 1.0 - shade


def classify_occupancy(pgm: np.ndarray, meta: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (occupied, free, unknown) boolean masks aligned with map_io trinary."""
    occ_prob = occupancy_probability(pgm, meta)
    occ_t = float(meta["occupied_thresh"])
    free_t = float(meta["free_thresh"])
    occupied = occ_prob > occ_t
    free = occ_prob < free_t
    unknown = ~occupied & ~free
    return occupied, free, unknown


def trinary_display_rgb(pgm: np.ndarray, meta: dict) -> np.ndarray:
    """RGB image: free=light, unknown=mid-gray, occupied=dark."""
    occupied, free, unknown = classify_occupancy(pgm, meta)
    rgb = np.zeros((*pgm.shape, 3), dtype=np.float32)
    rgb[free] = (0.92, 0.92, 0.92)
    rgb[unknown] = (0.62, 0.62, 0.62)
    rgb[occupied] = (0.12, 0.12, 0.12)
    return rgb


def obstacle_mask_for_distance(
    pgm: np.ndarray,
    meta: dict,
    treat_unknown_as_obstacle: bool,
) -> np.ndarray:
    occupied, _, unknown = classify_occupancy(pgm, meta)
    if treat_unknown_as_obstacle:
        return occupied | unknown
    return occupied


def compute_euclidean_clearance_m(
    pgm: np.ndarray,
    meta: dict,
    treat_unknown_as_obstacle: bool,
) -> np.ndarray:
    """Euclidean distance (m) from each cell center to nearest obstacle cell."""
    res = float(meta["resolution"])
    obstacle = obstacle_mask_for_distance(pgm, meta, treat_unknown_as_obstacle)
    if _edt is not None:
        return _edt(~obstacle) * res
    h, w = obstacle.shape
    out = np.full((h, w), np.nan, dtype=np.float32)
    occ_y, occ_x = np.where(obstacle)
    if occ_y.size == 0:
        return np.zeros((h, w), dtype=np.float32)
    for gy in range(h):
        for gx in range(w):
            d = np.hypot((occ_x - gx) * res, (occ_y - gy) * res)
            out[gy, gx] = float(np.min(d))
    return out


def node_clearance_at(
    node: dict,
    grid_dist: np.ndarray,
    euclid_dist: np.ndarray,
) -> tuple[float, float]:
    gx, gy = node["gx"], node["gy"]
    h, w = grid_dist.shape
    if not (0 <= gx < w and 0 <= gy < h):
        return float("nan"), float("nan")
    return float(grid_dist[gy, gx]), float(euclid_dist[gy, gx])


def draw_edges(ax, nodemap: dict, color: str, alpha: float, lw: float) -> None:
    nodes = nodemap.get("nodes", [])
    if not nodes:
        return
    if nodemap.get("adjacency_mode") == "grid8":
        grid = {}
        for n in nodes:
            grid[(n["gx"], n["gy"])] = n
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        drawn = set()
        for n in nodes:
            for dx, dy in moves:
                nb = grid.get((n["gx"] + dx, n["gy"] + dy))
                if nb is None:
                    continue
                key = tuple(sorted((n["id"], nb["id"])))
                if key in drawn:
                    continue
                drawn.add(key)
                ax.plot([n["x"], nb["x"]], [n["y"], nb["y"]], color=color, alpha=alpha, lw=lw)
        return
    adj = nodemap.get("adjacency", [])
    for i, nbs in enumerate(adj):
        if i >= len(nodes):
            break
        a = nodes[i]
        for j in nbs:
            if j >= len(nodes):
                continue
            b = nodes[j]
            ax.plot([a["x"], b["x"]], [a["y"], b["y"]], color=color, alpha=alpha, lw=lw)


def draw_nodes(ax, nodemap: dict, color: str, size: float, label: str) -> None:
    nodes = nodemap.get("nodes", [])
    if not nodes:
        return
    xs = [n["x"] for n in nodes]
    ys = [n["y"] for n in nodes]
    ax.scatter(xs, ys, c=color, s=size, label=label, alpha=0.9, edgecolors="k", linewidths=0.2)


def build_dashboard_figure(
    map_yaml: Path,
    occ: np.ndarray,
    occ_meta: dict,
    dist: np.ndarray,
    global_map: dict,
    local_map: dict,
    raw_map: dict,
    pre: Path,
    show_global: bool,
    show_local: bool,
    show_raw: bool,
) -> plt.Figure:
    res = float(occ_meta["resolution"])
    ox, oy = occ_meta["origin"][:2]
    h, w = occ.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f"Preprocessed map: {map_yaml.parent.name}", fontsize=14)

    ax = axes[0, 0]
    ax.imshow(occ, cmap="gray_r", origin="lower", extent=extent, alpha=0.9)
    ax.set_title("Original occupancy")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax = axes[0, 1]
    im = ax.imshow(dist, cmap="viridis", origin="lower", extent=extent, alpha=0.95)
    ax.set_title("Distance to obstacle [m]")
    plt.colorbar(im, ax=ax, fraction=0.046)

    ax = axes[1, 0]
    ax.imshow(occ, cmap="gray_r", origin="lower", extent=extent, alpha=0.35)
    layer_title_parts = []
    if show_global:
        draw_edges(ax, global_map, color="#1565C0", alpha=0.7, lw=1.2)
        draw_nodes(
            ax, global_map, color="#1976D2", size=18,
            label=f"L1 global ({len(global_map.get('nodes', []))})",
        )
        layer_title_parts.append("L1 blue")
    if show_local:
        draw_edges(ax, local_map, color="#2E7D32", alpha=0.5, lw=0.8)
        draw_nodes(
            ax, local_map, color="#43A047", size=8,
            label=f"L2 local ({len(local_map.get('nodes', []))})",
        )
        layer_title_parts.append("L2 green")
    ax.set_title(
        " + ".join(layer_title_parts) if layer_title_parts else "Node layers (all hidden)"
    )
    if layer_title_parts:
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    ax = axes[1, 1]
    ax.imshow(occ, cmap="gray_r", origin="lower", extent=extent, alpha=0.25)
    raw_nodes = raw_map.get("nodes", [])
    if show_raw:
        if raw_nodes and len(raw_nodes) <= 8000:
            draw_nodes(ax, raw_map, color="#E65100", size=2, label=f"L3 raw ({len(raw_nodes)})")
            ax.set_title(f"Layer 3 raw ({len(raw_nodes)} nodes)")
        else:
            mask_path = pre / raw_map.get("node_mask_image", "nodemap_raw_mask.pgm")
            if mask_path.exists():
                mask = load_pgm(mask_path)
                ax.imshow(mask, cmap="Oranges", origin="lower", extent=extent, alpha=0.6)
                ax.set_title(f"Layer 3 raw mask ({len(raw_nodes)} cells)")
            else:
                ax.set_title("Layer 3 raw (empty)")
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="upper right", fontsize=8)
    else:
        ax.set_title("Layer 3 raw (hidden)")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    fig.tight_layout()
    return fig


def build_interactive_nodemap_figure(
    occ: np.ndarray,
    occ_meta: dict,
    grid_dist: np.ndarray,
    euclid_dist: np.ndarray,
    layers: list[tuple[str, dict, str, float]],
    map_name: str,
) -> plt.Figure:
    """Second window: hover nodes to inspect clearance."""
    res = float(occ_meta["resolution"])
    ox, oy = occ_meta["origin"][:2]
    h, w = occ.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(12, 10))
    bg = trinary_display_rgb(occ, occ_meta)
    ax.imshow(bg, origin="lower", extent=extent, alpha=0.95)

    annotated: list[tuple[dict, str, float, float, float]] = []
    for layer_label, nodemap, color, size in layers:
        nodes = nodemap.get("nodes", [])
        if not nodes:
            continue
        min_c = float(nodemap.get("min_clearance_m", 0.0))
        draw_edges(ax, nodemap, color=color, alpha=0.55, lw=1.0)
        xs = [n["x"] for n in nodes]
        ys = [n["y"] for n in nodes]
        ax.scatter(
            xs, ys, c=color, s=size, label=f"{layer_label} ({len(nodes)})",
            alpha=0.95, edgecolors="k", linewidths=0.3, picker=5,
        )
        for n in nodes:
            g_clear, e_clear = node_clearance_at(n, grid_dist, euclid_dist)
            annotated.append((n, layer_label, min_c, g_clear, e_clear))

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.set_title(f"Interactive nodemap — {map_name}\n(hover a node)")

    info = ax.text(
        0.02, 0.02, "Move mouse over a node",
        transform=ax.transAxes, fontsize=10, va="bottom", ha="left",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    xs_all = np.array([item[0]["x"] for item in annotated], dtype=np.float64)
    ys_all = np.array([item[0]["y"] for item in annotated], dtype=np.float64)

    def pick_nearest(event) -> int | None:
        if event.inaxes is not ax or xs_all.size == 0:
            return None
        dx = xs_all - event.xdata
        dy = ys_all - event.ydata
        dist_w = np.hypot(dx, dy)
        idx = int(np.argmin(dist_w))
        # ~1.5 node radii in world units (rough hover tolerance).
        tol = max(res * 8.0, 0.12)
        if dist_w[idx] > tol:
            return None
        return idx

    def on_move(event) -> None:
        idx = pick_nearest(event)
        if idx is None:
            info.set_text("Move mouse over a node")
            fig.canvas.draw_idle()
            return
        node, layer_label, min_c, g_clear, e_clear = annotated[idx]
        pass_grid = g_clear >= min_c if not np.isnan(g_clear) else False
        pass_euclid = e_clear >= min_c if not np.isnan(e_clear) else False
        info.set_text(
            f"{layer_label}  node id={node['id']}\n"
            f"world: ({node['x']:.2f}, {node['y']:.2f})  "
            f"grid: ({node['gx']}, {node['gy']})\n"
            f"grid-path clearance: {g_clear:.3f} m  "
            f"({'PASS' if pass_grid else 'FAIL'} vs {min_c:.2f})\n"
            f"euclidean clearance: {e_clear:.3f} m  "
            f"({'PASS' if pass_euclid else 'FAIL'} vs {min_c:.2f})"
        )
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_move)
    return fig


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize 3-layer preprocessed map")
    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to original map.yaml (preprocessed/ must be alongside)",
    )
    parser.add_argument("--save", default="", help="Save figure to this path instead of showing")
    parser.add_argument(
        "--hide-global",
        action="store_true",
        help="Hide L1 global layer (nodes and edges)",
    )
    parser.add_argument(
        "--hide-local",
        action="store_true",
        help="Hide L2 local layer (nodes and edges)",
    )
    parser.add_argument(
        "--hide-raw",
        action="store_true",
        help="Hide L3 raw layer (nodes / mask overlay)",
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="Do not open the interactive nodemap window",
    )
    parser.add_argument(
        "--interactive-layer",
        choices=("global", "local", "all"),
        default="global",
        help="Which nodemap layer(s) to show in the interactive window (default: global)",
    )
    parser.add_argument(
        "--unknown-as-free",
        action="store_true",
        help="Treat unknown cells as free when computing euclidean clearance overlay",
    )
    args = parser.parse_args()

    show_global = not args.hide_global
    show_local = not args.hide_local
    show_raw = not args.hide_raw
    show_interactive = not args.no_interactive and not args.save

    map_yaml = Path(expand_user(args.map_yaml))
    pre = map_yaml.parent / "preprocessed"
    if not pre.exists():
        print(f"Missing preprocessed dir: {pre}", file=sys.stderr)
        return 1

    occ, occ_meta = load_map_yaml(map_yaml)
    dist, _ = load_distance_map(pre / "distance_map.yaml")
    global_map = load_node_map(pre / "nodemap_global.yaml")
    local_map = load_node_map(pre / "nodemap_local.yaml")
    raw_map = load_node_map(pre / "nodemap_raw.yaml")

    dashboard = build_dashboard_figure(
        map_yaml, occ, occ_meta, dist, global_map, local_map, raw_map, pre,
        show_global, show_local, show_raw,
    )

    if args.save:
        dashboard.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.save}")
    elif show_interactive:
        treat_unknown = not args.unknown_as_free
        euclid_dist = compute_euclidean_clearance_m(occ, occ_meta, treat_unknown)
        if _edt is None:
            print(
                "Note: install scipy for fast euclidean clearance "
                "(pip install scipy). Using slow fallback.",
                file=sys.stderr,
            )

        interactive_layers: list[tuple[str, dict, str, float]] = []
        if args.interactive_layer in ("global", "all"):
            interactive_layers.append(("L1 global", global_map, "#1976D2", 36.0))
        if args.interactive_layer in ("local", "all"):
            interactive_layers.append(("L2 local", local_map, "#43A047", 14.0))

        build_interactive_nodemap_figure(
            occ, occ_meta, dist, euclid_dist, interactive_layers, map_yaml.parent.name,
        )
        plt.show()
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

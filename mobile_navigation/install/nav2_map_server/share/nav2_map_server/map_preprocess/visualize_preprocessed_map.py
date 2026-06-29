#!/usr/bin/env python3
"""Visualize preprocessed map: occupancy, distance field, 3-layer node graphs."""

from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


def expand_user(path: str) -> str:
    return os.path.expanduser(path)


def load_map_yaml(map_yaml: Path) -> tuple[np.ndarray, dict]:
    with open(map_yaml, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img_path = map_yaml.parent / meta["image"]
    if not img_path.exists():
        img_path = Path(meta["image"])
    data = load_pgm(img_path)
    return data, meta


def load_pgm(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM magic: {magic}")
        dims = b""
        while len(dims.split()) < 2:
            dims += f.readline()
        w, h = map(int, dims.split()[:2])
        maxval = int(f.readline().strip())
        if magic == b"P5":
            raw = f.read(w * h)
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
        else:
            arr = np.loadtxt(f, dtype=np.uint8).reshape(h, w)
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


def world_to_map(x: float, y: float, origin, resolution: float) -> tuple[int, int]:
    gx = int((x - origin[0]) / resolution)
    gy = int((y - origin[1]) / resolution)
    return gx, gy


def draw_edges(ax, nodemap: dict, color: str, alpha: float, lw: float) -> None:
    nodes = nodemap.get("nodes", [])
    if not nodes:
        return
    origin = nodemap["origin"]
    res = nodemap["resolution"]
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize 3-layer preprocessed map")
    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to original map.yaml (preprocessed/ must be alongside)",
    )
    parser.add_argument("--save", default="", help="Save figure to this path instead of showing")
    args = parser.parse_args()

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
    draw_edges(ax, global_map, color="#1565C0", alpha=0.7, lw=1.2)
    draw_nodes(ax, global_map, color="#1976D2", size=18, label=f"L1 global ({len(global_map.get('nodes', []))})")
    draw_edges(ax, local_map, color="#2E7D32", alpha=0.5, lw=0.8)
    draw_nodes(ax, local_map, color="#43A047", size=8, label=f"L2 local ({len(local_map.get('nodes', []))})")
    ax.set_title("Layer 1 (blue) + Layer 2 (green)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    ax = axes[1, 1]
    raw_nodes = raw_map.get("nodes", [])
    if raw_nodes and len(raw_nodes) <= 8000:
        draw_nodes(ax, raw_map, color="#E65100", size=2, label=f"L3 raw ({len(raw_nodes)})")
    else:
        mask_path = pre / raw_map.get("node_mask_image", "nodemap_raw_mask.pgm")
        if mask_path.exists():
            mask = load_pgm(mask_path)
            ax.imshow(mask, cmap="Oranges", origin="lower", extent=extent, alpha=0.6)
            ax.set_title(f"Layer 3 raw mask ({len(raw_nodes)} cells)")
        else:
            ax.set_title("Layer 3 raw (empty)")
    ax.imshow(occ, cmap="gray_r", origin="lower", extent=extent, alpha=0.25)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

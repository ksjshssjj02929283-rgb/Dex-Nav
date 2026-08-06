#!/usr/bin/env python3
"""Private fixture builder used by the map_manager test entry."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from PIL import Image


SAMPLE_NAMES = (
    "mm_test_a",
    "mm_test_b",
    "mm_test_legacy",
    "mm_test_broken",
)
VERSION_TS = "20260725-000001"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def nav_point(name: str, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
    return {
        "name": name,
        "pose": {"x": x, "y": y, "yaw": yaw},
        "tolerance": 0.2,
        "type": "STATION",
    }


def asset_variant(
    baseline: dict[str, Any],
    *,
    points: list[dict[str, Any]],
    routes: list[dict[str, Any]],
    areas: list[dict[str, Any]],
) -> dict[str, Any]:
    result = copy.deepcopy(baseline)
    result["points"] = points
    result["routes"] = routes
    result["areas"] = areas
    return result


def mutate_boundary_unknown_pixel(source: Path, target: Path, offset: int) -> None:
    """Make one top-boundary unknown cell free for an observable map oracle."""
    with Image.open(source) as image:
        result = image.convert("L")
        x, y = offset, 0
        current = int(result.getpixel((x, y)))
        if current >= 254:
            raise ValueError(
                f"baseline boundary pixel ({x}, {y}) is already free"
            )
        result.putpixel((x, y), 255)
        target.parent.mkdir(parents=True, exist_ok=True)
        result.save(target, format="PNG")


def copy_slam_files(source_slam: Path, target_slam: Path) -> None:
    target_slam.mkdir(parents=True, exist_ok=True)
    for name in ("map.yaml", "map.png", "map.pbstream"):
        shutil.copy2(source_slam / name, target_slam / name)


def validate_source(source: Path) -> tuple[Path, Path, dict[str, Any]]:
    source_slam = source / "slam"
    source_assets = source / "assets/map_assets.json"
    required = [
        source_slam / "map.yaml",
        source_slam / "map.png",
        source_slam / "map.pbstream",
        source_assets,
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ValueError(f"baseline map is incomplete: {missing}")
    if (source_slam / "map.pbstream").stat().st_size < 1024:
        raise ValueError("baseline map.pbstream is suspiciously small")
    baseline_assets = json.loads(source_assets.read_text(encoding="utf-8"))
    for key in ("points", "routes", "areas"):
        if not isinstance(baseline_assets.get(key), list):
            raise ValueError(f"baseline asset field is not an array: {key}")
    return source_slam, source_assets, baseline_assets


def build(source: Path, output: Path) -> dict[str, Any]:
    source = source.resolve()
    output = output.resolve()
    source_slam, source_assets_path, baseline_assets = validate_source(source)
    maps_root = output / "maps"
    maps_root.mkdir(parents=True)

    map_a = maps_root / "mm_test_a"
    copy_slam_files(source_slam, map_a / "slam")
    a_assets = asset_variant(
        baseline_assets,
        points=[nav_point("AUTO_A_POINT", 0.0, 0.0)],
        routes=[],
        areas=[
            {
                "attribute": "map_manager_auto_canary_a",
                "charger_type": "",
                "manual_linked_point_names": [],
                "max_speed": 0.35,
                "name": "AUTO_A_AREA",
                "polygon": [
                    {"x": -2.0, "y": -2.0},
                    {"x": 2.0, "y": -2.0},
                    {"x": 2.0, "y": 2.0},
                    {"x": -2.0, "y": 2.0},
                ],
                "related_junction_ids": [],
                "related_nav_point_names": ["AUTO_A_POINT"],
                "stop_distance": 0.0,
                "type": "SPEED_LIMIT_AREA",
            }
        ],
    )
    write_json(map_a / "assets/map_assets.json", a_assets)

    map_backup = map_a / f"slam/backups/backup.{VERSION_TS}"
    copy_slam_files(source_slam, map_backup)
    mutate_boundary_unknown_pixel(
        source_slam / "map.png", map_backup / "map.png", 1
    )
    rollback_assets = asset_variant(
        baseline_assets,
        points=[
            nav_point("AUTO_ROLLBACK_POINT_1", -1.0, -1.0),
            nav_point("AUTO_ROLLBACK_POINT_2", -3.0, -1.0),
        ],
        routes=[
            {
                "name": "AUTO_ROLLBACK_ROUTE",
                "points": [
                    "AUTO_ROLLBACK_POINT_1",
                    "AUTO_ROLLBACK_POINT_2",
                ],
            }
        ],
        areas=[],
    )
    write_json(
        map_a / f"assets/backups/map_assets.{VERSION_TS}.json",
        rollback_assets,
    )

    map_b = maps_root / "mm_test_b"
    copy_slam_files(source_slam, map_b / "slam")
    mutate_boundary_unknown_pixel(
        source_slam / "map.png", map_b / "slam/map.png", 0
    )
    b_assets = asset_variant(
        baseline_assets,
        points=[
            nav_point("AUTO_B_POINT_1", 1.0, 1.0),
            nav_point("AUTO_B_POINT_2", 3.0, 1.0),
        ],
        routes=[
            {
                "name": "AUTO_B_ROUTE",
                "points": ["AUTO_B_POINT_1", "AUTO_B_POINT_2"],
            }
        ],
        areas=[],
    )
    write_json(map_b / "assets/map_assets.json", b_assets)

    legacy = maps_root / "mm_test_legacy"
    legacy.mkdir()
    for name in ("map.yaml", "map.png", "map.pbstream"):
        shutil.copy2(source_slam / name, legacy / name)
    legacy_assets = asset_variant(
        baseline_assets,
        points=[nav_point("AUTO_LEGACY_POINT", 5.0, 5.0)],
        routes=[],
        areas=[],
    )
    write_json(legacy / "map_assets.json", legacy_assets)

    broken = maps_root / "mm_test_broken"
    broken.mkdir()
    for name in ("map.png", "map.pbstream"):
        shutil.copy2(source_slam / name, broken / name)

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source": {
            "name": source.name,
            "path": str(source),
            "sha256": {
                "slam/map.yaml": sha256(source_slam / "map.yaml"),
                "slam/map.png": sha256(source_slam / "map.png"),
                "slam/map.pbstream": sha256(source_slam / "map.pbstream"),
                "assets/map_assets.json": sha256(source_assets_path),
            },
        },
        "samples": {
            "map_a": {
                "name": "mm_test_a",
                "asset_counts": {"points": 1, "routes": 0, "areas": 1},
                "canaries": ["AUTO_A_POINT", "AUTO_A_AREA"],
                "rollback": {
                    "map_version_ts": VERSION_TS,
                    "assets_version_ts": VERSION_TS,
                    "asset_counts": {"points": 2, "routes": 1, "areas": 0},
                    "canaries": [
                        "AUTO_ROLLBACK_POINT_1",
                        "AUTO_ROLLBACK_POINT_2",
                        "AUTO_ROLLBACK_ROUTE",
                    ],
                },
            },
            "map_b": {
                "name": "mm_test_b",
                "asset_counts": {"points": 2, "routes": 1, "areas": 0},
                "canaries": [
                    "AUTO_B_POINT_1",
                    "AUTO_B_POINT_2",
                    "AUTO_B_ROUTE",
                ],
            },
            "legacy_flat": {
                "name": "mm_test_legacy",
                "asset_counts": {"points": 1, "routes": 0, "areas": 0},
                "canaries": ["AUTO_LEGACY_POINT"],
            },
            "broken_legacy": {
                "name": "mm_test_broken",
                "expected_failure": "missing map.yaml",
            },
        },
    }
    manifest["sha256"] = {
        str(path.relative_to(output)): sha256(path)
        for path in sorted(maps_root.rglob("*"))
        if path.is_file()
    }
    write_json(output / "manifest.json", manifest)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_map", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    if args.output.exists():
        print(f"ERROR: output already exists: {args.output}", file=sys.stderr)
        return 2
    try:
        manifest = build(args.source_map, args.output)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "samples": sorted(
                    spec["name"] for spec in manifest["samples"].values()
                ),
                "checksums": len(manifest["sha256"]),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

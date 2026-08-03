#!/usr/bin/env python3
"""Private data-driven checks used by the map_manager test entry."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from map_manager_auto_test import Report, RosHarness, execute_case
from map_manager_test_paths import (
    DEFAULT_MAPS_ROOT,
    DEFAULT_RESULT_ROOT,
    default_manifest_path,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tree_hashes(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def tree_layout(root: Path) -> dict[str, str]:
    layout: dict[str, str] = {}
    for path in sorted(root.rglob("*")):
        relative = str(path.relative_to(root))
        if path.is_dir():
            layout[relative] = "dir"
        elif path.is_file():
            layout[relative] = "file"
        else:
            layout[relative] = "other"
    return layout


_UNORDERED_ASSET_ARRAY_PATHS = {
    ("points",),
    ("routes",),
    ("areas",),
    ("route_graph", "junctions"),
    ("route_graph", "edges"),
    ("dsg", "layers", "ROOMS"),
    ("dsg", "layers", "PLACES"),
    ("dsg", "layers", "FLOORS"),
    ("dsg", "layers", "OBJECTS"),
    ("dsg", "layers", "BUILDINGS"),
    ("dsg", "edges"),
}
_UNORDERED_ASSET_MEMBER_KEYS = {
    "additional_area_names",
    "manual_anchor_point_names",
    "manual_linked_point_names",
    "related_junction_ids",
    "related_nav_point_names",
}


def canonicalize_asset_json(value: Any, path: tuple[str, ...] = ()) -> Any:
    if isinstance(value, dict):
        return {
            key: canonicalize_asset_json(item, path + (key,))
            for key, item in value.items()
        }
    if isinstance(value, list):
        normalized = [
            canonicalize_asset_json(item, path + ("*",))
            for item in value
        ]
        if (
            path in _UNORDERED_ASSET_ARRAY_PATHS
            or (path and path[-1] in _UNORDERED_ASSET_MEMBER_KEYS)
        ):
            return sorted(
                normalized,
                key=lambda item: json.dumps(
                    item,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ),
            )
        return normalized
    return value


def asset_semantic_digest(path: Path) -> str:
    value = json.loads(path.read_text(encoding="utf-8"))
    normalized = canonicalize_asset_json(value)
    payload = json.dumps(
        normalized,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def grid_digest(grid: Any) -> str:
    digest = hashlib.sha256()
    metadata = (
        grid.info.width,
        grid.info.height,
        grid.info.resolution,
        grid.info.origin.position.x,
        grid.info.origin.position.y,
    )
    digest.update(repr(metadata).encode())
    digest.update(bytes((value + 256) % 256 for value in grid.data))
    return digest.hexdigest()


class DataRunner:
    def __init__(
        self,
        harness: RosHarness,
        report: Report,
        maps_root: Path,
        manifest: dict[str, Any],
        rollback_service: str,
    ) -> None:
        from nav_interfaces.msg import MapState
        from nav_interfaces.srv import (
            GetAllAreas,
            GetAllNavPoints,
            GetAllRoutes,
            LoadMap,
            RenameMap,
            RollbackMap,
        )
        from nav_msgs.msg import OccupancyGrid

        self.harness = harness
        self.report = report
        self.maps_root = maps_root.resolve()
        self.manifest = manifest
        self.rollback_service = rollback_service
        samples = manifest["samples"]
        self.map_a = samples["map_a"]["name"]
        self.map_b = samples["map_b"]["name"]
        self.legacy = samples["legacy_flat"]["name"]
        self.broken = samples["broken_legacy"]["name"]
        self.MapState = MapState
        self.OccupancyGrid = OccupancyGrid
        self.LoadMap = LoadMap
        self.RenameMap = RenameMap
        self.RollbackMap = RollbackMap
        self.GetAllNavPoints = GetAllNavPoints
        self.GetAllRoutes = GetAllRoutes
        self.GetAllAreas = GetAllAreas

    def state(self, after_epoch: int = -1) -> Any:
        return self.harness.wait_message(
            self.MapState,
            "/map_manager/map_state",
            transient_local=True,
            predicate=lambda msg: msg.epoch > after_epoch,
        )

    def grid(self) -> Any:
        return self.harness.wait_message(
            self.OccupancyGrid,
            "/map",
            transient_local=True,
        )

    def load(self, name: str, tc: str = "") -> tuple[Any, Any]:
        if tc:
            self.report.action(
                tc,
                "读取变更前 MapState",
                f"target={name}",
            )
        before = self.state()
        request = self.LoadMap.Request()
        request.map_name = name
        if tc:
            self.report.action(
                tc,
                "调用 load_map",
                f"service=/map_manager/load_map, map_name={name}",
            )
        response = self.harness.call(
            self.LoadMap, "/map_manager/load_map", request
        )
        if tc:
            self.report.action(
                tc,
                "读取 load_map 响应",
                (
                    f"success={response.success}, "
                    f"message={getattr(response, 'message', '')}"
                ),
            )
        if tc and response.success:
            self.report.action(
                tc,
                "等待新 MapState",
                f"predicate=epoch>{before.epoch}",
            )
        after = self.state(before.epoch) if response.success else self.state()
        return response, after

    def asset_names(self) -> tuple[set[str], set[str], set[str]]:
        points = self.harness.call(
            self.GetAllNavPoints,
            "/map_asset_manager/get_all_nav_points",
            self.GetAllNavPoints.Request(),
        )
        routes = self.harness.call(
            self.GetAllRoutes,
            "/map_asset_manager/get_all_routes",
            self.GetAllRoutes.Request(),
        )
        areas = self.harness.call(
            self.GetAllAreas,
            "/map_asset_manager/get_all_areas",
            self.GetAllAreas.Request(),
        )
        if not points.success or not routes.success or not areas.success:
            raise RuntimeError("one or more map_asset_manager queries failed")
        return (
            {item.name for item in points.points},
            {item.name for item in routes.routes},
            {item.name for item in areas.area_info_list},
        )

    def check_canaries(self, tc: str, spec: dict[str, Any]) -> None:
        self.report.action(
            tc,
            "查询地图资产",
            "get_all_nav_points/get_all_routes/get_all_areas",
        )
        points, routes, areas = self.asset_names()
        actual = points | routes | areas
        expected = set(spec.get("canaries", []))
        counts = spec["asset_counts"]
        passed = (
            expected <= actual
            and len(points) == counts["points"]
            and len(routes) == counts["routes"]
            and len(areas) == counts["areas"]
        )
        self.report.add(
            tc,
            "资产 canary 与计数",
            passed,
            (
                f"points={sorted(points)}, routes={sorted(routes)}, "
                f"areas={sorted(areas)}"
            ),
        )

    def tc05(self) -> None:
        legacy = self.maps_root / self.legacy
        self.report.action(
            "TC-05",
            "检查迁移前目录",
            f"root={legacy}, flat_map_yaml={(legacy / 'map.yaml').is_file()}",
        )
        response, state = self.load(self.legacy, "TC-05")
        migrated = (
            response.success
            and state.map_name == self.legacy
            and (legacy / "slam/map.yaml").is_file()
            and (legacy / "slam/map.png").is_file()
            and (legacy / "slam/map.pbstream").is_file()
            and (legacy / "assets/map_assets.json").is_file()
            and not (legacy / "map.yaml").exists()
            and not (legacy / "map_assets.json").exists()
        )
        self.report.add(
            "TC-05",
            "旧扁平地图首次加载自动迁移",
            migrated,
            f"load={response.success}, state={state.map_name}, root={legacy}",
        )

    def tc06(self) -> None:
        legacy = self.maps_root / self.legacy
        self.report.action("TC-06", "记录迁移后目录哈希", f"root={legacy}")
        before = tree_hashes(legacy)
        response2, state2 = self.load(self.legacy, "TC-06")
        after = tree_hashes(legacy)
        self.report.add(
            "TC-06",
            "迁移幂等",
            response2.success and state2.map_name == self.legacy and before == after,
            f"before_files={len(before)}, after_files={len(after)}",
        )

    def tc07(self) -> None:
        map_dir = self.maps_root / self.map_a
        self.report.action(
            "TC-07",
            "记录加载前目录布局与 slam 哈希",
            f"root={map_dir}",
        )
        before_layout = tree_layout(map_dir)
        before_hashes = tree_hashes(map_dir)
        before_slam = {
            path: digest
            for path, digest in before_hashes.items()
            if path.startswith("slam/")
        }
        response, state = self.load(self.map_a, "TC-07")
        self.report.action(
            "TC-07",
            "记录加载后目录布局与 slam 哈希",
            f"root={map_dir}",
        )
        after_layout = tree_layout(map_dir)
        after_hashes = tree_hashes(map_dir)
        after_slam = {
            path: digest
            for path, digest in after_hashes.items()
            if path.startswith("slam/")
        }
        added = sorted(after_layout.keys() - before_layout.keys())
        removed = sorted(before_layout.keys() - after_layout.keys())
        type_changed = sorted(
            path
            for path in before_layout.keys() & after_layout.keys()
            if before_layout[path] != after_layout[path]
        )
        modified = sorted(
            path
            for path in before_hashes.keys() & after_hashes.keys()
            if before_hashes[path] != after_hashes[path]
        )
        slam_modified = sorted(
            path
            for path in before_slam.keys() & after_slam.keys()
            if before_slam[path] != after_slam[path]
        )
        self.report.add(
            "TC-07",
            "新分层地图布局与 slam 不受扰动",
            (
                response.success
                and state.map_name == self.map_a
                and before_layout == after_layout
                and before_slam == after_slam
            ),
            (
                f"state={state.map_name}, layout_match="
                f"{before_layout == after_layout}, slam_match="
                f"{before_slam == after_slam}, added={added}, removed={removed}, "
                f"type_changed={type_changed}, slam_modified={slam_modified}, "
                f"recorded_non_slam_modified="
                f"{sorted(set(modified) - set(slam_modified))}"
            ),
        )

    def tc08(self) -> None:
        response, state = self.load(self.map_a, "TC-08")
        if not response.success:
            self.report.add(
                "TC-08",
                "准备 map_a",
                False,
                f"message={getattr(response, 'message', '')}",
            )
            return
        epoch = state.epoch
        response2, state2 = self.load(self.map_a, "TC-08")
        self.report.add(
            "TC-08",
            "同名重载 epoch + 1",
            (
                response2.success
                and state2.map_name == self.map_a
                and state2.epoch == epoch + 1
            ),
            f"epoch={epoch}->{state2.epoch}, name={state2.map_name}",
        )
        self.check_canaries("TC-08", self.manifest["samples"]["map_a"])

    def tc09(self) -> None:
        response, state = self.load(self.map_a, "TC-09")
        if not response.success:
            self.report.add(
                "TC-09",
                "准备 map_a",
                False,
                f"message={getattr(response, 'message', '')}",
            )
            return
        self.report.action(
            "TC-09",
            "扫描旧定位 push 服务",
            "查找 */set_map_changed",
        )
        services = {name for name, _ in self.harness.node.get_service_names_and_types()}
        legacy_push_services = sorted(
            name for name in services if name.endswith("/set_map_changed")
        )
        start = time.monotonic()
        response3, state3 = self.load(self.map_a, "TC-09")
        elapsed = time.monotonic() - start
        self.report.add(
            "TC-09",
            "仅 MapState 触发且无旧 localization push",
            (
                response3.success
                and state3.epoch == state.epoch + 1
                and not legacy_push_services
                and elapsed < 2.0
            ),
            (
                f"elapsed={elapsed:.3f}s, epoch={state.epoch}->{state3.epoch}, "
                f"legacy_push_services={legacy_push_services}"
            ),
        )

    def tc10(self) -> None:
        _, before = self.load(self.map_a, "TC-10")
        rollback = self.manifest["samples"]["map_a"]["rollback"]
        request = self.RollbackMap.Request()
        request.map_version_ts = rollback["map_version_ts"]
        request.assets_version_ts = rollback["assets_version_ts"]
        self.report.action(
            "TC-10",
            "调用组合回退服务",
            (
                f"service={self.rollback_service}, "
                f"map_version={request.map_version_ts}, "
                f"assets_version={request.assets_version_ts}"
            ),
        )
        response = self.harness.call(
            self.RollbackMap, self.rollback_service, request
        )
        self.report.action(
            "TC-10",
            "读取回退响应",
            (
                f"success={response.success}, "
                f"message={getattr(response, 'message', '')}"
            ),
        )
        after = None
        state_error = ""
        if response.success:
            self.report.action(
                "TC-10",
                "等待回退后的 MapState",
                f"predicate=epoch>{before.epoch}",
            )
            try:
                after = self.state(before.epoch)
            except TimeoutError as exc:
                state_error = str(exc)
        map_dir = self.maps_root / self.map_a
        map_backup = (
            map_dir / f"slam/backups/backup.{rollback['map_version_ts']}"
        )
        map_matches = {
            name: sha256(map_dir / "slam" / name)
            == sha256(map_backup / name)
            for name in ("map.pbstream", "map.png", "map.yaml")
        }
        map_match = all(map_matches.values())
        current_assets = map_dir / "assets/map_assets.json"
        backup_assets = map_dir / (
            "assets/backups/map_assets."
            f"{rollback['assets_version_ts']}.json"
        )
        self.report.action(
            "TC-10",
            "比较完整资产 JSON 语义",
            "规范化对象键、无序资产集合和集合型关联字段",
        )
        assets_semantic_error = ""
        current_assets_digest = ""
        backup_assets_digest = ""
        try:
            current_assets_digest = asset_semantic_digest(current_assets)
            backup_assets_digest = asset_semantic_digest(backup_assets)
            assets_semantic_match = (
                current_assets_digest == backup_assets_digest
            )
        except (OSError, ValueError) as exc:
            assets_semantic_match = False
            assets_semantic_error = str(exc)
        assets_byte_match = sha256(current_assets) == sha256(backup_assets)
        self.report.add(
            "TC-10",
            "底图字节与完整资产语义组合回退",
            (
                response.success
                and after is not None
                and after.map_name == self.map_a
                and after.epoch == before.epoch + 1
                and map_match
                and assets_semantic_match
            ),
            (
                f"epoch={before.epoch}->"
                f"{after.epoch if after is not None else 'TIMEOUT'}, "
                f"map_matches={map_matches}, "
                f"assets_semantic_match={assets_semantic_match}, "
                f"assets_byte_match_record_only={assets_byte_match}, "
                f"asset_semantic_digests="
                f"{current_assets_digest[:12]}/{backup_assets_digest[:12]}, "
                f"asset_semantic_error={assets_semantic_error}, "
                f"message={getattr(response, 'message', '')}, "
                f"state_error={state_error}"
            ),
        )
        self.check_canaries("TC-10", rollback)

    def tc11(self) -> None:
        _, state_a = self.load(self.map_a, "TC-11")
        self.report.action("TC-11", "读取地图 A 栅格", "计算 OccupancyGrid digest")
        grid_a = grid_digest(self.grid())
        response_b, state_b = self.load(self.map_b, "TC-11")
        self.report.action("TC-11", "读取地图 B 栅格", "计算 OccupancyGrid digest")
        grid_b = grid_digest(self.grid())
        self.report.add(
            "TC-11",
            "A 切换到 B",
            (
                response_b.success
                and state_b.map_name == self.map_b
                and state_b.epoch == state_a.epoch + 1
                and grid_a != grid_b
            ),
            f"epoch={state_a.epoch}->{state_b.epoch}, grid_changed={grid_a != grid_b}",
        )
        self.check_canaries("TC-11", self.manifest["samples"]["map_b"])

    def tc12(self) -> None:
        _, state_b = self.load(self.map_b, "TC-12")
        self.report.action("TC-12", "读取地图 B 栅格", "计算 OccupancyGrid digest")
        grid_b = grid_digest(self.grid())
        response_a, state_a2 = self.load(self.map_a, "TC-12")
        self.report.action("TC-12", "读取地图 A 栅格", "计算 OccupancyGrid digest")
        grid_a2 = grid_digest(self.grid())
        self.report.add(
            "TC-12",
            "B 切回 A",
            (
                response_a.success
                and state_a2.map_name == self.map_a
                and state_a2.epoch == state_b.epoch + 1
                and grid_a2 != grid_b
            ),
            (
                f"epoch={state_b.epoch}->{state_a2.epoch}, "
                f"grid_changed={grid_a2 != grid_b}"
            ),
        )
        self.check_canaries("TC-12", self.manifest["samples"]["map_a"])

    def tc13(self) -> None:
        _, state_a2 = self.load(self.map_a, "TC-13")
        start = state_a2.epoch
        states = [
            self.load(name, "TC-13")[1]
            for name in (self.map_a, self.map_b, self.map_a)
        ]
        self.report.add(
            "TC-13",
            "快速连续 A→B→A 收敛",
            (
                [item.epoch for item in states]
                == [start + 1, start + 2, start + 3]
                and states[-1].map_name == self.map_a
            ),
            f"epochs={[item.epoch for item in states]}, final={states[-1].map_name}",
        )

    def tc14(self) -> None:
        _, before = self.load(self.map_a, "TC-14")
        self.report.action("TC-14", "读取失败前栅格", "计算 OccupancyGrid digest")
        before_grid = grid_digest(self.grid())
        request = self.LoadMap.Request()
        request.map_name = self.broken
        self.report.action(
            "TC-14",
            "调用损坏地图 load_map",
            f"map_name={self.broken}",
        )
        response = self.harness.call(
            self.LoadMap, "/map_manager/load_map", request
        )
        self.report.action(
            "TC-14",
            "读取失败后的状态与栅格",
            f"service_success={response.success}",
        )
        after = self.state()
        after_grid = grid_digest(self.grid())
        self.report.add(
            "TC-14",
            "缺 map.yaml 时失败且当前图不变",
            (
                not response.success
                and after.map_name == before.map_name
                and after.epoch == before.epoch
                and before_grid == after_grid
            ),
            (
                f"success={response.success}, state={after.map_name}, "
                f"epoch={before.epoch}->{after.epoch}, "
                f"grid_unchanged={before_grid == after_grid}"
            ),
        )

    def tc15(self) -> None:
        renamed = f"{self.map_a}_renamed_auto"
        if (self.maps_root / renamed).exists():
            raise RuntimeError(f"temporary rename target already exists: {renamed}")
        _, before = self.load(self.map_a, "TC-15")
        self.report.action("TC-15", "记录 rename 前目录哈希", f"map={self.map_a}")
        original_hashes = tree_hashes(self.maps_root / self.map_a)
        request = self.RenameMap.Request()
        request.old_name = self.map_a
        request.new_name = renamed
        self.report.action(
            "TC-15",
            "调用第一次 rename",
            f"{self.map_a}->{renamed}",
        )
        response = self.harness.call(
            self.RenameMap, "/map_manager/rename_map", request
        )
        renamed_state = self.state(before.epoch)
        request2 = self.RenameMap.Request()
        request2.old_name = renamed
        request2.new_name = self.map_a
        self.report.action(
            "TC-15",
            "调用恢复 rename",
            f"{renamed}->{self.map_a}",
        )
        response2 = self.harness.call(
            self.RenameMap, "/map_manager/rename_map", request2
        )
        restored_state = self.state(renamed_state.epoch)
        restored_hashes = tree_hashes(self.maps_root / self.map_a)
        self.report.add(
            "TC-15",
            "当前地图 rename 与回改",
            (
                response.success
                and response2.success
                and renamed_state.map_name == renamed
                and restored_state.map_name == self.map_a
                and renamed_state.epoch == before.epoch + 1
                and restored_state.epoch == renamed_state.epoch + 1
                and original_hashes == restored_hashes
            ),
            (
                f"epochs={before.epoch}->{renamed_state.epoch}->"
                f"{restored_state.epoch}, restored={original_hashes == restored_hashes}"
            ),
        )


def validate_live_files(
    maps_root: Path, manifest: dict[str, Any]
) -> list[str]:
    errors: list[str] = []
    for relative, expected in manifest["sha256"].items():
        parts = Path(relative).parts
        if not parts or parts[0] != "maps":
            errors.append(f"unsafe manifest path: {relative}")
            continue
        path = maps_root.joinpath(*parts[1:])
        if not path.is_file():
            errors.append(f"missing: {path}")
        elif sha256(path) != expected:
            errors.append(f"checksum mismatch: {path}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--maps-root",
        type=Path,
        default=DEFAULT_MAPS_ROOT,
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=default_manifest_path(),
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--confirm-mutations", action="store_true")
    parser.add_argument(
        "--rollback-service",
        default="auto",
        help=(
            "RollbackMap service name; auto detects /command_board/rollback_map "
            "or /command_board_manager/rollback_map"
        ),
    )
    args = parser.parse_args()
    if not args.confirm_mutations:
        parser.error(
            "this runner loads, migrates, rolls back and renames mm_test_* maps; "
            "pass --confirm-mutations"
        )

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    errors = validate_live_files(args.maps_root, manifest)
    if errors:
        parser.error("fixture precheck failed: " + "; ".join(errors))

    names = [spec["name"] for spec in manifest["samples"].values()]
    if any(not name.startswith("mm_test_") for name in names):
        parser.error("all mutable fixture names must start with mm_test_")

    artifact_dir = args.artifact_dir or (
        DEFAULT_RESULT_ROOT
        / f"data_{datetime.now().astimezone():%Y%m%d_%H%M%S}"
    )
    report = Report()
    harness: RosHarness | None = None
    original_map = ""
    rollback_service = args.rollback_service
    with tempfile.TemporaryDirectory(prefix="map_manager_fixture_backup_") as tmp:
        backup_root = Path(tmp)
        for name in names:
            shutil.copytree(args.maps_root / name, backup_root / name)
        try:
            harness = RosHarness(args.timeout)
            if rollback_service == "auto":
                services = {
                    name
                    for name, _ in harness.node.get_service_names_and_types()
                }
                candidates = (
                    "/command_board/rollback_map",
                    "/command_board_manager/rollback_map",
                )
                rollback_service = next(
                    (name for name in candidates if name in services),
                    "",
                )
                if not rollback_service:
                    raise RuntimeError(
                        "RollbackMap service unavailable; tried "
                        + ", ".join(candidates)
                    )
            original_map = DataRunner(
                harness,
                report,
                args.maps_root,
                manifest,
                rollback_service,
            ).state().map_name
            runner = DataRunner(
                harness,
                report,
                args.maps_root,
                manifest,
                rollback_service,
            )
            cases = (
                ("TC-05", "旧扁平地图首次加载自动迁移", runner.tc05),
                ("TC-06", "旧图迁移幂等", runner.tc06),
                ("TC-07", "新分层地图不迁移", runner.tc07),
                ("TC-08", "同名重载 epoch 与资产内容", runner.tc08),
                ("TC-09", "仅 MapState 触发定位重载", runner.tc09),
                ("TC-10", "底图与资产组合回退", runner.tc10),
            )
            for tc, title, method in cases:
                execute_case(report, tc, title, method)

            # TC-10 changes A current files; always restore pristine A before
            # running independent switch/failure/rename cases.
            report.start_case("RESTORE-A", "回退用例后的 map_a 恢复")
            try:
                report.action(
                    "RESTORE-A",
                    "恢复 map_a 底图与资产",
                    "从本轮临时备份复制 map.png/map_assets.json",
                )
                shutil.copy2(
                    backup_root / runner.map_a / "slam/map.png",
                    args.maps_root / runner.map_a / "slam/map.png",
                )
                shutil.copy2(
                    backup_root / runner.map_a / "assets/map_assets.json",
                    args.maps_root / runner.map_a / "assets/map_assets.json",
                )
                response, _ = runner.load(runner.map_a, "RESTORE-A")
                report.add(
                    "RESTORE-A",
                    "恢复后重新加载 map_a",
                    response.success,
                    f"success={response.success}",
                )
            except Exception as exc:
                report.error("RESTORE-A", "恢复 map_a", repr(exc))
            finally:
                report.finish_case("RESTORE-A")

            for tc, title, method in (
                ("TC-11", "地图 A 切换到 B", runner.tc11),
                ("TC-12", "地图 B 切回 A", runner.tc12),
                ("TC-13", "快速连续 A→B→A 收敛", runner.tc13),
                ("TC-14", "损坏地图失败保护", runner.tc14),
                ("TC-15", "当前地图 rename 与恢复", runner.tc15),
            ):
                execute_case(report, tc, title, method)
        except Exception as exc:
            report.error("RUNNER", "未处理异常", repr(exc))
        finally:
            renamed = args.maps_root / "mm_test_a_renamed_auto"
            if renamed.exists():
                shutil.rmtree(renamed)
            for name in names:
                target = args.maps_root / name
                if target.exists():
                    shutil.rmtree(target)
                shutil.copytree(backup_root / name, target)
            if harness is not None:
                if original_map:
                    try:
                        runner.load(original_map)
                    except Exception as exc:
                        report.error("RESTORE", "恢复原当前地图", repr(exc))
                harness.close()

    report.save(
        artifact_dir,
        {
            "executed_at": datetime.now().isoformat(),
            "mode": "data-driven",
            "maps_root": str(args.maps_root.resolve()),
            "manifest": str(args.manifest.resolve()),
            "original_map": original_map,
            "rollback_service": rollback_service,
        },
    )
    print(f"result: {artifact_dir}")
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

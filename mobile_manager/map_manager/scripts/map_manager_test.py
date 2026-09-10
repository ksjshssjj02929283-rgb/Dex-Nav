#!/usr/bin/env python3
"""Installed command dispatcher for map_manager fixture and acceptance tests."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

ENTRY_DIR = Path(__file__).resolve().parent
PRIVATE_SUPPORT_CANDIDATES = (
    ENTRY_DIR / "map_manager_test_support",
    ENTRY_DIR.parent / "test_support",
)
PRIVATE_SUPPORT_DIR = next(
    (path for path in PRIVATE_SUPPORT_CANDIDATES if path.is_dir()),
    PRIVATE_SUPPORT_CANDIDATES[0],
)
sys.path.insert(0, str(PRIVATE_SUPPORT_DIR))

from build_standard_map_data import SAMPLE_NAMES, build, sha256
from map_manager_test_paths import (
    DEFAULT_MAPS_ROOT,
    DEFAULT_RESULT_ROOT,
    DEFAULT_SOURCE_MAP_NAME,
    SCRIPT_DIR,
    default_manifest_path,
    detect_source_package,
    packaged_standard_map_path,
    runtime_manifest_path,
    runtime_state_path,
)
from validate_standard_map_data import Validator


MANAGED_MAP_NAMES = (DEFAULT_SOURCE_MAP_NAME, *SAMPLE_NAMES)
EXPECTED_VALIDATION_SUMMARY = {"pass": 87, "fail": 0, "pending": 0}


class FlowTrace:
    def __init__(self, flow_id: str, title: str) -> None:
        self.flow_id = flow_id
        self.title = title
        self.events: list[dict[str, str | int]] = []
        print(f"\n╭─ [{flow_id}] {title}", flush=True)

    def _append(
        self, kind: str, name: str, status: str, detail: str
    ) -> None:
        step = len(self.events) + 1
        self.events.append(
            {
                "step": step,
                "kind": kind,
                "name": name,
                "status": status,
                "detail": detail,
            }
        )
        if kind == "ACTION":
            marker = "▶"
        else:
            marker = "✓" if status == "PASS" else "✗"
        print(
            f"│ {step:02d} {marker} {kind} [{status}] {name}"
            + (f" — {detail}" if detail else ""),
            flush=True,
        )

    def action(self, name: str, detail: str = "") -> None:
        self._append("ACTION", name, "RUNNING", detail)

    def check(self, passed: bool, name: str, detail: str = "") -> bool:
        self._append("CHECK", name, "PASS" if passed else "FAIL", detail)
        return passed

    def finish(self, status: str) -> None:
        print(f"╰─ RESULT [{status}] {self.flow_id} {self.title}", flush=True)

    def save(self, path: Path) -> None:
        lines = [
            f"# {self.flow_id} {self.title}",
            "",
            "| 步骤 | 类型 | 动作 / 检查 | 结果 | 详细 |",
            "|---:|---|---|---|---|",
        ]
        for event in self.events:
            detail = str(event["detail"]).replace("|", "\\|")
            lines.append(
                f"| {event['step']} | {event['kind']} | {event['name']} | "
                f"{event['status']} | {detail} |"
            )
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_json(value: dict[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2))


def write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def timestamped_result_dir(prefix: str) -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    base = DEFAULT_RESULT_ROOT / f"{prefix}_{timestamp}"
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{suffix:02d}")
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def source_hashes(source_map: Path) -> dict[str, str]:
    return {
        str(path.relative_to(source_map)): sha256(path)
        for path in sorted(source_map.rglob("*"))
        if path.is_file()
    }


def validate_managed_target(maps_root: Path, name: str) -> Path:
    if name not in MANAGED_MAP_NAMES:
        raise ValueError(f"refusing unmanaged map name: {name}")
    target = maps_root / name
    if target.parent != maps_root:
        raise ValueError(f"unsafe managed map path: {target}")
    if target.is_symlink():
        raise ValueError(f"refusing symlink managed map: {target}")
    return target


def current_managed_map(timeout: float = 0.5) -> str | None:
    """Return the active managed map when a live map_manager is reachable."""
    try:
        from map_manager_auto_test import RosHarness
        from nav_interfaces.srv import GetMaps
    except ImportError:
        return None

    harness = None
    try:
        harness = RosHarness(timeout)
        response = harness.call(
            GetMaps,
            "/map_manager/get_maps",
            GetMaps.Request(),
        )
        if response.success and response.current_map_name in MANAGED_MAP_NAMES:
            return response.current_map_name
    except Exception:
        # An unavailable ROS graph is normal for local/offline self-tests.
        return None
    finally:
        if harness is not None:
            harness.close()
    return None


def read_managed_state(state_path: Path, maps_root: Path) -> dict[str, Any]:
    state = json.loads(state_path.read_text(encoding="utf-8"))
    if state.get("schema_version") != 1:
        raise ValueError("unsupported managed state schema")
    if Path(state.get("maps_root", "")).resolve() != maps_root:
        raise ValueError("managed state maps_root mismatch")
    names = tuple(state.get("managed_maps", []))
    if names != MANAGED_MAP_NAMES:
        raise ValueError(f"managed map allowlist mismatch: {names}")
    return state


def reusable_ready_dataset(
    *,
    maps_root: Path,
    state_path: Path,
    manifest_output: Path,
    package_source: Path,
    package_hashes: dict[str, str],
) -> tuple[dict[str, Any] | None, str]:
    """Return validation details only for an unchanged, complete ready dataset."""
    try:
        state = read_managed_state(state_path, maps_root)
        if state.get("status") != "ready":
            return None, f"managed state status={state.get('status')!r}, expected='ready'"
        if Path(state.get("package_standard_map", "")).resolve() != package_source:
            return None, "packaged standard map path changed"
        if state.get("standard_map_sha256") != package_hashes:
            return None, "packaged standard map checksum changed"

        state_manifest = Path(state.get("manifest", "")).resolve()
        if state_manifest != manifest_output:
            return None, (
                f"managed manifest mismatch: state={state_manifest}, "
                f"requested={manifest_output}"
            )
        if not state_manifest.is_file():
            return None, f"managed manifest missing: {state_manifest}"
        if source_hashes(maps_root / DEFAULT_SOURCE_MAP_NAME) != package_hashes:
            return None, "deployed map_standard checksum mismatch"
        for name in MANAGED_MAP_NAMES:
            if not validate_managed_target(maps_root, name).is_dir():
                return None, f"managed map directory missing: {name}"

        validation = Validator(
            Path("."),
            maps_root=maps_root,
            manifest_path=state_manifest,
        ).run()
        if validation["summary"] != EXPECTED_VALIDATION_SUMMARY:
            return None, f"deployed validation failed: {validation['summary']}"
        return validation, "ready state, checksums and 87 fixture checks are unchanged"
    except Exception as exc:
        return None, f"managed state is not reusable: {exc}"


def cleanup_managed_maps(maps_root: Path, *, emit: bool = True) -> int:
    maps_root = maps_root.resolve()
    state_path = runtime_state_path(maps_root).resolve()
    flow = FlowTrace("CLEANUP", "受管测试地图清理")
    flow.action("读取受管状态", str(state_path))
    if not state_path.is_file():
        flow.check(True, "受管状态不存在", "无需清理")
        flow.finish("SKIPPED")
        payload = {
            "status": "SKIPPED",
            "maps_root": str(maps_root),
            "reason": "managed state does not exist",
            "removed": [],
        }
        if emit:
            print_json(payload)
        return 0

    try:
        state = read_managed_state(state_path, maps_root)
        names = tuple(state["managed_maps"])
        active_map = current_managed_map()
        flow.action(
            "检查当前活动地图",
            f"active_managed_map={active_map}",
        )
        if active_map is not None:
            raise RuntimeError(
                f"refusing to remove active managed map: {active_map}; "
                "switch map_manager to a production map and retry cleanup"
            )

        removed: list[str] = []
        for name in names:
            target = validate_managed_target(maps_root, name)
            if not target.exists():
                flow.action("跳过不存在的受管地图", str(target))
                continue
            flow.action("删除受管测试地图", str(target))
            if target.is_dir():
                shutil.rmtree(target)
            else:
                target.unlink()
            removed.append(name)

        manifest = Path(state.get("manifest", "")).resolve()
        managed_data_root = (maps_root.parent / "map_manager_test").resolve()
        if manifest.parent == managed_data_root and manifest.is_file():
            flow.action("删除运行时 manifest", str(manifest))
            manifest.unlink()
        flow.action("删除受管状态", str(state_path))
        state_path.unlink()
    except Exception as exc:
        flow.check(False, "受管清理", repr(exc))
        flow.finish("FAIL")
        print(f"ERROR: managed map cleanup failed: {exc}", file=sys.stderr)
        return 1

    flow.check(True, "受管地图清理完成", f"removed={removed}")
    flow.finish("PASS")
    payload = {
        "status": "PASS",
        "maps_root": str(maps_root),
        "removed": removed,
        "preserved_results": str(DEFAULT_RESULT_ROOT),
    }
    if emit:
        print_json(payload)
    return 0


def prepare_dataset(args: argparse.Namespace, *, emit: bool = True) -> int:
    maps_root = args.maps_root.resolve()
    package_source = packaged_standard_map_path().resolve()
    manifest_output = (
        getattr(args, "manifest_output", None) or runtime_manifest_path(maps_root)
    ).resolve()
    state_path = runtime_state_path(maps_root).resolve()
    flow = FlowTrace("PREPARE", "标准地图与受管测试集准备")
    flow.action(
        "解析运行路径",
        (
            f"package_source={package_source}, maps_root={maps_root}, "
            f"manifest={manifest_output}"
        ),
    )

    if not flow.check(
        package_source.name == DEFAULT_SOURCE_MAP_NAME,
        "随包标准地图名称",
        f"actual={package_source.name}, expected={DEFAULT_SOURCE_MAP_NAME}",
    ):
        flow.finish("FAIL")
        print(
            f"ERROR: packaged standard map must be named "
            f"{DEFAULT_SOURCE_MAP_NAME}: {package_source}",
            file=sys.stderr,
        )
        return 2
    if not flow.check(
        package_source.is_dir(),
        "随包标准地图目录存在",
        str(package_source),
    ):
        flow.finish("FAIL")
        print(
            f"ERROR: packaged standard map does not exist: {package_source}",
            file=sys.stderr,
        )
        return 2

    flow.action("计算随包标准图哈希", "递归读取 4 个标准文件")
    package_hashes = source_hashes(package_source)
    if not flow.check(
        len(package_hashes) == 4,
        "标准图文件数量",
        f"actual={len(package_hashes)}, expected=4",
    ):
        flow.finish("FAIL")
        print(
            f"ERROR: packaged standard map must contain exactly 4 files: "
            f"{package_source}",
            file=sys.stderr,
        )
        return 1

    if state_path.exists():
        flow.action("检查上一轮受管测试集是否可复用", f"state={state_path}")
        validation, reuse_detail = reusable_ready_dataset(
            maps_root=maps_root,
            state_path=state_path,
            manifest_output=manifest_output,
            package_source=package_source,
            package_hashes=package_hashes,
        )
        if validation is not None:
            flow.check(True, "复用 ready 受管测试集", reuse_detail)
            result_dir = timestamped_result_dir("prepare")
            flow.finish("PASS")
            flow.save(result_dir / "flow.md")
            payload = {
                "status": "PASS",
                "reused_existing_dataset": True,
                "package_standard_map": str(package_source),
                "deployed_standard_map": str(
                    maps_root / DEFAULT_SOURCE_MAP_NAME
                ),
                "maps_root": str(maps_root),
                "managed_maps": list(MANAGED_MAP_NAMES),
                "manifest": str(manifest_output),
                "managed_state": str(state_path),
                "validation": validation["summary"],
                "package_source_files_unchanged": len(package_hashes),
                "flow": str(result_dir / "flow.md"),
                "result": str(result_dir),
            }
            write_json_atomic(result_dir / "summary.json", payload)
            if emit:
                print_json(payload)
            return 0

        flow.action("上一轮受管测试集不可复用，进入安全清理", reuse_detail)
        cleanup_rc = cleanup_managed_maps(maps_root, emit=False)
        if not flow.check(
            cleanup_rc == 0,
            "上一轮受管地图清理",
            f"returncode={cleanup_rc}",
        ):
            flow.finish("FAIL")
            return 1

    maps_root.mkdir(parents=True, exist_ok=True)
    current_standard = validate_managed_target(
        maps_root, DEFAULT_SOURCE_MAP_NAME
    )
    if current_standard.exists():
        flow.action(
            "检查地图目录中的既有 map_standard",
            str(current_standard),
        )
        if not current_standard.is_dir():
            flow.check(False, "既有 map_standard 类型", "不是目录")
            flow.finish("FAIL")
            print(
                f"ERROR: standard map target is not a directory: "
                f"{current_standard}",
                file=sys.stderr,
            )
            return 1
        same_standard = source_hashes(current_standard) == package_hashes
        if not flow.check(
            same_standard,
            "既有 map_standard 与随包标准图一致",
            f"path={current_standard}",
        ):
            flow.finish("FAIL")
            print(
                f"ERROR: refusing to replace a different existing "
                f"{DEFAULT_SOURCE_MAP_NAME}: {current_standard}",
                file=sys.stderr,
            )
            return 1

    with tempfile.TemporaryDirectory(
        prefix="map_manager_prepare_",
        dir=maps_root.parent,
    ) as temp_name:
        temp_root = Path(temp_name)
        staged_standard = temp_root / DEFAULT_SOURCE_MAP_NAME
        built = temp_root / "built"
        backup = temp_root / "backup"
        backup.mkdir()
        try:
            flow.action("复制标准图到临时区", str(staged_standard))
            shutil.copytree(package_source, staged_standard)
            flow.action("构建 4 套 mm_test_* 数据", str(built))
            build(package_source, built)
        except Exception as exc:
            flow.check(False, "临时测试集构建", repr(exc))
            flow.finish("FAIL")
            print(f"ERROR: fixture build failed: {exc}", file=sys.stderr)
            return 1

        if not flow.check(
            source_hashes(staged_standard) == package_hashes,
            "临时 map_standard 哈希",
            "与随包标准图比较",
        ):
            flow.finish("FAIL")
            print("ERROR: staged map_standard checksum mismatch", file=sys.stderr)
            return 1

        flow.action("执行临时测试集完整性校验", "预期 PASS=87")
        staged_result = Validator(built).run()
        staged_ok = staged_result["summary"] == EXPECTED_VALIDATION_SUMMARY
        if not flow.check(
            staged_ok,
            "临时测试集 87 项校验",
            str(staged_result["summary"]),
        ):
            flow.finish("FAIL")
            if emit:
                print_json({"stage": "staged_validation", **staged_result})
            return 1

        state = {
            "schema_version": 1,
            "status": "deploying",
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "maps_root": str(maps_root),
            "package_standard_map": str(package_source),
            "managed_maps": list(MANAGED_MAP_NAMES),
            "manifest": str(manifest_output),
            "standard_map_sha256": package_hashes,
        }
        flow.action("写入受管状态", f"status=deploying, path={state_path}")
        write_json_atomic(state_path, state)

        deployed: list[str] = []
        backed_up: list[str] = []
        deployment_sources = {
            DEFAULT_SOURCE_MAP_NAME: staged_standard,
            **{
                name: built / "maps" / name
                for name in SAMPLE_NAMES
            },
        }
        try:
            for name in MANAGED_MAP_NAMES:
                source = deployment_sources[name]
                target = validate_managed_target(maps_root, name)
                old = backup / name
                if target.exists():
                    flow.action("备份既有受管地图", f"{name}->{old}")
                    target.rename(old)
                    backed_up.append(name)
                flow.action("部署受管测试地图", f"{source}->{target}")
                source.rename(target)
                deployed.append(name)

            flow.action("执行已部署测试集校验", "预期 PASS=87")
            deployed_result = Validator(
                built,
                maps_root=maps_root,
                manifest_path=built / "manifest.json",
            ).run()
            deployed_ok = deployed_result["summary"] == EXPECTED_VALIDATION_SUMMARY
            if not flow.check(
                deployed_ok,
                "已部署测试集 87 项校验",
                str(deployed_result["summary"]),
            ):
                raise RuntimeError(
                    f"deployed validation failed: {deployed_result['summary']}"
                )
            if not flow.check(
                source_hashes(maps_root / DEFAULT_SOURCE_MAP_NAME)
                == package_hashes,
                "部署后 map_standard 哈希",
                str(maps_root / DEFAULT_SOURCE_MAP_NAME),
            ):
                raise RuntimeError("deployed map_standard checksum mismatch")
            if not flow.check(
                source_hashes(package_source) == package_hashes,
                "随包标准图保持只读不变",
                str(package_source),
            ):
                raise RuntimeError("packaged map_standard changed during preparation")

            flow.action("发布 manifest 与 ready 状态", str(manifest_output))
            manifest_output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(built / "manifest.json", manifest_output)
            state["status"] = "ready"
            write_json_atomic(state_path, state)
        except BaseException as exc:
            flow.action("回滚不完整部署", repr(exc))
            for name in reversed(deployed):
                target = validate_managed_target(maps_root, name)
                if target.exists():
                    if target.is_dir():
                        shutil.rmtree(target)
                    else:
                        target.unlink()
            for name in reversed(backed_up):
                old = backup / name
                if old.exists():
                    old.rename(maps_root / name)
            state_path.unlink(missing_ok=True)
            flow.finish("FAIL")
            print(f"ERROR: deployment rolled back: {exc}", file=sys.stderr)
            return 1

    result_dir = timestamped_result_dir("prepare")
    flow.finish("PASS")
    flow.save(result_dir / "flow.md")
    payload = {
        "status": "PASS",
        "reused_existing_dataset": False,
        "package_standard_map": str(package_source),
        "deployed_standard_map": str(maps_root / DEFAULT_SOURCE_MAP_NAME),
        "maps_root": str(maps_root),
        "managed_maps": list(MANAGED_MAP_NAMES),
        "manifest": str(manifest_output),
        "managed_state": str(state_path),
        "validation": EXPECTED_VALIDATION_SUMMARY,
        "package_source_files_unchanged": len(package_hashes),
        "flow": str(result_dir / "flow.md"),
        "result": str(result_dir),
    }
    write_json_atomic(result_dir / "summary.json", payload)
    if emit:
        print_json(payload)
    return 0


def validate_dataset(args: argparse.Namespace) -> int:
    maps_root = args.maps_root.resolve()
    manifest = (args.manifest or default_manifest_path(maps_root)).resolve()
    result = Validator(
        Path("."),
        maps_root=maps_root,
        manifest_path=manifest,
    ).run()
    print_json(result)
    return 0 if result["summary"]["fail"] == 0 else 1


def run_child(command: list[str]) -> int:
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return subprocess.run(command, env=env, check=False).returncode


def run_local_prepared(args: argparse.Namespace) -> int:
    maps_root = args.maps_root.resolve()
    source_map = maps_root / DEFAULT_SOURCE_MAP_NAME
    manifest = runtime_manifest_path(maps_root).resolve()
    command = [
        sys.executable,
        str(SCRIPT_DIR / "run_map_manager_local_self_test.py"),
        "--maps-root",
        str(maps_root),
        "--source-map",
        str(source_map),
        "--manifest",
        str(manifest),
    ]
    package_path = args.map_manager_package or detect_source_package()
    if package_path is not None:
        command.extend(["--map-manager-package", str(package_path)])
    if args.artifact_dir:
        command.extend(["--artifact-dir", str(args.artifact_dir)])
    return run_child(command)


def run_live_prepared(args: argparse.Namespace) -> int:
    maps_root = args.maps_root.resolve()
    manifest = runtime_manifest_path(maps_root).resolve()
    command = [
        sys.executable,
        str(SCRIPT_DIR / "run_map_manager_acceptance.py"),
        "--maps-root",
        str(maps_root),
        "--manifest",
        str(manifest),
        "--graph-timeout",
        str(args.graph_timeout),
        "--require-live",
    ]
    if args.confirm_mutations:
        command.append("--confirm-mutations")
    if args.artifact_dir:
        command.extend(["--artifact-dir", str(args.artifact_dir)])
    return run_child(command)


def run_managed_test(
    args: argparse.Namespace,
    runner: Callable[[argparse.Namespace], int],
) -> int:
    prepare_args = argparse.Namespace(
        maps_root=args.maps_root,
        manifest_output=runtime_manifest_path(args.maps_root),
    )
    prepare_rc = prepare_dataset(prepare_args, emit=False)
    if prepare_rc != 0:
        return prepare_rc

    cleanup_rc = 0
    try:
        test_rc = runner(args)
    finally:
        if args.keep_test_maps:
            print("test maps: KEPT (--keep-test-maps)")
        else:
            cleanup_rc = cleanup_managed_maps(args.maps_root, emit=False)
            print(
                "test maps: CLEANED"
                if cleanup_rc == 0
                else "test maps: CLEANUP_FAILED",
                file=sys.stdout if cleanup_rc == 0 else sys.stderr,
            )
    return test_rc if test_rc != 0 else cleanup_rc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Copy the packaged map_standard, prepare map_manager fixtures, "
            "and run local/live self-tests."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser(
        "prepare",
        help="copy packaged map_standard and deploy validated mm_test_* maps",
    )
    prepare.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    prepare.add_argument("--manifest-output", type=Path)
    prepare.set_defaults(handler=prepare_dataset)

    validate = subparsers.add_parser(
        "validate",
        help="validate deployed mm_test_* against the current manifest",
    )
    validate.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    validate.add_argument("--manifest", type=Path)
    validate.set_defaults(handler=validate_dataset)

    cleanup = subparsers.add_parser(
        "cleanup",
        help="remove only maps listed in the managed test state",
    )
    cleanup.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    cleanup.set_defaults(
        handler=lambda args: cleanup_managed_maps(args.maps_root)
    )

    local = subparsers.add_parser(
        "local",
        help="prepare, run local self-tests, and clean test maps",
    )
    local.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    local.add_argument("--map-manager-package", type=Path)
    local.add_argument("--artifact-dir", type=Path)
    local.add_argument("--keep-test-maps", action="store_true")
    local.set_defaults(
        handler=lambda args: run_managed_test(args, run_local_prepared)
    )

    live = subparsers.add_parser(
        "live",
        help="prepare, run full ROS2 acceptance, and clean test maps",
    )
    live.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    live.add_argument("--artifact-dir", type=Path)
    live.add_argument(
        "--graph-timeout",
        type=float,
        default=30.0,
        help="seconds to wait for the complete ROS graph after preflight",
    )
    live.add_argument("--confirm-mutations", action="store_true")
    live.add_argument("--keep-test-maps", action="store_true")
    live.set_defaults(
        handler=lambda args: run_managed_test(args, run_live_prepared)
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())

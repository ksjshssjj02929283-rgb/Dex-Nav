#!/usr/bin/env python3
"""Private local self-test implementation used by the public test entry.

This suite never calls live map mutation services. It validates the source build
when source is available, otherwise the installed package layout. It also checks
ROS interfaces, frozen fixtures, deterministic fixture rebuild, negative
checksum detection, the unified runner's offline contract, and pre/post hashes
of map_standard plus all mm_test_* fixtures.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from map_manager_test_paths import (
    DEFAULT_MAPS_ROOT,
    DEFAULT_RESULT_ROOT,
    DEFAULT_SOURCE_MAP_NAME,
    default_manifest_path,
    detect_source_package,
    installed_package_prefix,
    installed_package_share,
)

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SOURCE_MAP = DEFAULT_MAPS_ROOT / DEFAULT_SOURCE_MAP_NAME
DEFAULT_MANIFEST = default_manifest_path()
PROTECTED_MAPS = (
    DEFAULT_SOURCE_MAP_NAME,
    "mm_test_a",
    "mm_test_b",
    "mm_test_legacy",
    "mm_test_broken",
)


@dataclass
class Check:
    test_id: str
    name: str
    status: str
    detail: str
    evidence: str


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tree_hashes(maps_root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for name in PROTECTED_MAPS:
        root = maps_root / name
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if path.is_file():
                result[str(path.relative_to(maps_root))] = sha256(path)
    return result


def run_command(
    command: list[str],
    log_path: Path,
    *,
    cwd: Path | None = None,
    timeout: float = 300.0,
) -> tuple[int, str]:
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        output = completed.stdout or ""
        returncode = completed.returncode
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") + (exc.stderr or "")
        output += f"\nTIMEOUT after {timeout:.1f}s\n"
        returncode = 124
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(
        "$ " + " ".join(command) + "\n\n" + output,
        encoding="utf-8",
    )
    return returncode, output


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} top-level JSON is not an object")
    return value


def add(
    checks: list[Check],
    test_id: str,
    name: str,
    passed: bool,
    detail: str,
    evidence: Path | str,
) -> None:
    evidence_text = str(evidence)
    checks.append(
        Check(
            test_id,
            name,
            "PASS" if passed else "FAIL",
            detail,
            evidence_text,
        )
    )


def git_value(repo: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "unknown"


def git_repo_root(path: Path | None) -> Path | None:
    if path is None:
        return None
    root = git_value(path, "rev-parse", "--show-toplevel")
    return Path(root) if root != "unknown" else None


def installed_layout_check(
    private_script_names: list[str],
) -> tuple[bool, str, Path | None]:
    prefix = installed_package_prefix()
    package_share = installed_package_share()
    if prefix is None or package_share is None:
        return False, "map_manager install prefix not found", prefix

    executable_dir = prefix / "lib/map_manager"
    public_entry = executable_dir / "map_manager_test.py"
    map_manager_node = executable_dir / "map_manager_node"
    private_dir = executable_dir / "map_manager_test_support"
    required_files = [
        public_entry,
        map_manager_node,
        package_share / "package.xml",
        package_share / "test_data/standard_map_manifest.json",
        package_share / "test_data/map_standard/slam/map.yaml",
        *(private_dir / name for name in private_script_names),
    ]
    missing = [str(path) for path in required_files if not path.is_file()]
    executable_ok = all(
        path.is_file() and os.access(path, os.X_OK)
        for path in (public_entry, map_manager_node)
    )
    private_non_executable = all(
        path.is_file() and not os.access(path, os.X_OK)
        for path in (private_dir / name for name in private_script_names)
    )
    passed = not missing and executable_ok and private_non_executable
    detail = (
        f"prefix={prefix}, missing={len(missing)}, "
        f"public_executable={executable_ok}, "
        f"private_non_executable={private_non_executable}"
    )
    if missing:
        detail += "; missing_files=" + ",".join(missing)
    return passed, detail, prefix


def save_summary(
    artifact_dir: Path,
    checks: list[Check],
    metadata: dict[str, Any],
) -> dict[str, Any]:
    counts = {
        "pass": sum(check.status == "PASS" for check in checks),
        "fail": sum(check.status == "FAIL" for check in checks),
        "total": len(checks),
    }
    payload = {
        "metadata": metadata,
        "overall": "PASS" if counts["fail"] == 0 else "FAIL",
        "counts": counts,
        "checks": [asdict(check) for check in checks],
        "acceptance_boundary": {
            "local_suite": "本机只读/临时副本自测",
            "m5_runtime": "未替代 15 项完整 ROS2 业务栈功能 TC",
        },
    }
    (artifact_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# map_manager 第一版本机自测结果",
        "",
        "> 由 map_manager 自动测试脚本生成。",
        "",
        f"- 执行时间：{metadata['executed_at']}",
        f"- 本机自测：**{payload['overall']}**",
        f"- 检查数：PASS {counts['pass']} / FAIL {counts['fail']} / TOTAL {counts['total']}",
        f"- map_manager：`{metadata['map_manager_branch']}@{metadata['map_manager_commit']}`",
        f"- nav_interfaces：`{metadata['interfaces_branch']}@{metadata['interfaces_commit']}`",
        "",
        "| ID | 本机检查项 | 结果 | 结果摘要 | 证据 |",
        "|---|---|---|---|---|",
    ]
    for check in checks:
        detail = check.detail.replace("|", "\\|").replace("\n", "<br>")
        evidence = check.evidence.replace("|", "\\|")
        lines.append(
            f"| {check.test_id} | {check.name} | {check.status} | "
            f"{detail} | `{evidence}` |"
        )
    lines.extend(
        [
            "",
            "## 验收边界",
            "",
            "- 本结果证明本机代码构建、接口、标准数据、数据生成/校验工具及离线门禁可用。",
            "- 本结果不等于 M5 完整栈验收通过；15 项自动功能 TC 仍须在业务节点启动后执行。",
        ]
    )
    (artifact_dir / "summary.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    parser.add_argument("--source-map", type=Path, default=DEFAULT_SOURCE_MAP)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--map-manager-package",
        type=Path,
        default=detect_source_package(),
    )
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--command-timeout", type=float, default=300.0)
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="validate the installed package instead of rebuilding source",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    artifact_dir = args.artifact_dir or (
        DEFAULT_RESULT_ROOT / f"local_v1_{timestamp}"
    )
    artifact_dir.mkdir(parents=True, exist_ok=False)
    checks: list[Check] = []

    maps_root = args.maps_root.resolve()
    source_map = args.source_map.resolve()
    manifest_path = args.manifest.resolve()
    package_path = (
        args.map_manager_package.resolve()
        if args.map_manager_package is not None
        else None
    )
    map_repo = git_repo_root(package_path)
    nav_repo_override = os.environ.get("NAV_INTERFACES_SOURCE_REPO")
    nav_repo = git_repo_root(
        Path(nav_repo_override).expanduser() if nav_repo_override else None
    )

    before_hashes = tree_hashes(maps_root)
    (artifact_dir / "protected_maps.before.json").write_text(
        json.dumps(before_hashes, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    private_script_names = [
        "build_standard_map_data.py",
        "validate_standard_map_data.py",
        "map_manager_auto_test.py",
        "map_manager_data_test.py",
        "run_map_manager_acceptance.py",
        "run_map_manager_local_self_test.py",
        "map_manager_test_paths.py",
    ]
    public_entry_candidates = (
        SCRIPT_DIR.parent / "map_manager_test.py",
        SCRIPT_DIR.parent / "scripts/map_manager_test.py",
    )
    public_entry = next(
        (path for path in public_entry_candidates if path.is_file()),
        public_entry_candidates[0],
    )
    script_paths = [
        *(SCRIPT_DIR / name for name in private_script_names),
        public_entry,
    ]
    syntax_errors: list[str] = []
    for path in script_paths:
        try:
            ast.parse(
                path.read_text(encoding="utf-8"),
                filename=path.name,
            )
        except Exception as exc:
            syntax_errors.append(f"{path.name}: {exc}")
    add(
        checks,
        "LOCAL-01",
        "Python 自动化脚本语法",
        not syntax_errors,
        f"{len(script_paths)}/{len(script_paths)} AST 可解析"
        if not syntax_errors
        else "; ".join(syntax_errors),
        "scripts/map_manager_test.py + test_support/*.py",
    )

    try:
        manifest = load_json(manifest_path)
        manifest_ok = (
            manifest.get("schema_version") == 1
            and set(manifest.get("samples", {}))
            == {"map_a", "map_b", "legacy_flat", "broken_legacy"}
            and len(manifest.get("sha256", {})) == 18
        )
        manifest_detail = (
            f"schema={manifest.get('schema_version')}, "
            f"samples={len(manifest.get('samples', {}))}, "
            f"sha256={len(manifest.get('sha256', {}))}"
        )
    except Exception as exc:
        manifest = {}
        manifest_ok = False
        manifest_detail = str(exc)
    add(
        checks,
        "LOCAL-02",
        "冻结 manifest 结构",
        manifest_ok,
        manifest_detail,
        manifest_path,
    )

    try:
        from nav_interfaces.msg import MapState
        # v1.11（DEC-029）：查询合并 GetMaps，写操作合并 MapManagerCommand；旧 DeleteMap/GetMapInfo/GetMapList/GetString/LoadMap/RenameMap 已删
        from nav_interfaces.srv import GetMaps, MapManagerCommand, RollbackMap
        from nav_msgs.srv import GetMap

        del GetMaps, MapManagerCommand, RollbackMap, GetMap
        actual_fields = MapState.get_fields_and_field_types()
        expected_fields = {
            "header": "std_msgs/Header",
            "map_folder": "string",
            "map_name": "string",
            "epoch": "uint64",
        }
        interface_ok = actual_fields == expected_fields
        interface_detail = f"MapState={actual_fields}; required services importable"
    except Exception as exc:
        interface_ok = False
        interface_detail = str(exc)
    (artifact_dir / "ros_interface_contract.txt").write_text(
        interface_detail + "\n",
        encoding="utf-8",
    )
    add(
        checks,
        "LOCAL-03",
        "已安装 ROS2 接口契约",
        interface_ok,
        interface_detail,
        artifact_dir / "ros_interface_contract.txt",
    )

    deployed_json = artifact_dir / "deployed_fixture_validation.json"
    deployed_rc, _ = run_command(
        [
            sys.executable,
            str(SCRIPT_DIR / "validate_standard_map_data.py"),
            "--maps-root",
            str(maps_root),
            "--manifest",
            str(manifest_path),
            "--json-output",
            str(deployed_json),
        ],
        artifact_dir / "deployed_fixture_validation.log",
        timeout=args.command_timeout,
    )
    try:
        deployed_summary = load_json(deployed_json)["summary"]
    except Exception:
        deployed_summary = {"pass": 0, "fail": 1, "pending": 0}
    add(
        checks,
        "LOCAL-04",
        "已部署标准地图集完整性",
        deployed_rc == 0
        and deployed_summary == {"pass": 87, "fail": 0, "pending": 0},
        f"returncode={deployed_rc}, summary={deployed_summary}",
        deployed_json,
    )

    source_hashes = {
        str(path.relative_to(source_map)): sha256(path)
        for path in sorted(source_map.rglob("*"))
        if path.is_file()
    }
    expected_source_hashes = manifest.get("source", {}).get("sha256", {})
    source_ok = all(
        source_hashes.get(relative) == expected
        for relative, expected in expected_source_hashes.items()
    ) and len(expected_source_hashes) == 4
    (artifact_dir / "map_standard.source_hashes.json").write_text(
        json.dumps(source_hashes, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    add(
        checks,
        "LOCAL-05",
        "map_standard 冻结源哈希",
        source_ok,
        f"matched={sum(source_hashes.get(k) == v for k, v in expected_source_hashes.items())}/4",
        artifact_dir / "map_standard.source_hashes.json",
    )

    with tempfile.TemporaryDirectory(prefix="map_manager_local_v1_") as temp_name:
        temp_root = Path(temp_name)
        rebuilt = temp_root / "rebuilt"
        build_data_rc, _ = run_command(
            [
                sys.executable,
                str(SCRIPT_DIR / "build_standard_map_data.py"),
                str(source_map),
                str(rebuilt),
            ],
            artifact_dir / "fixture_rebuild.log",
            timeout=args.command_timeout,
        )
        try:
            rebuilt_manifest = load_json(rebuilt / "manifest.json")
            deterministic_ok = (
                build_data_rc == 0
                and rebuilt_manifest.get("samples") == manifest.get("samples")
                and rebuilt_manifest.get("sha256") == manifest.get("sha256")
                and rebuilt_manifest.get("source", {}).get("sha256")
                == manifest.get("source", {}).get("sha256")
            )
            deterministic_detail = (
                f"returncode={build_data_rc}, "
                f"checksums={len(rebuilt_manifest.get('sha256', {}))}/18"
            )
        except Exception as exc:
            rebuilt_manifest = {}
            deterministic_ok = False
            deterministic_detail = f"returncode={build_data_rc}, error={exc}"
        add(
            checks,
            "LOCAL-06",
            "从 map_standard 确定性重建测试地图集",
            deterministic_ok,
            deterministic_detail,
            artifact_dir / "fixture_rebuild.log",
        )

        rebuilt_json = artifact_dir / "rebuilt_fixture_validation.json"
        rebuilt_rc, _ = run_command(
            [
                sys.executable,
                str(SCRIPT_DIR / "validate_standard_map_data.py"),
                str(rebuilt),
                "--json-output",
                str(rebuilt_json),
            ],
            artifact_dir / "rebuilt_fixture_validation.log",
            timeout=args.command_timeout,
        )
        try:
            rebuilt_summary = load_json(rebuilt_json)["summary"]
        except Exception:
            rebuilt_summary = {"pass": 0, "fail": 1, "pending": 0}
        add(
            checks,
            "LOCAL-07",
            "重建数据集完整性",
            rebuilt_rc == 0
            and rebuilt_summary == {"pass": 87, "fail": 0, "pending": 0},
            f"returncode={rebuilt_rc}, summary={rebuilt_summary}",
            rebuilt_json,
        )

        tamper_target = rebuilt / "maps/mm_test_a/assets/map_assets.json"
        if tamper_target.is_file():
            tamper_target.write_text(
                tamper_target.read_text(encoding="utf-8") + "\n",
                encoding="utf-8",
            )
        tampered_json = artifact_dir / "tampered_fixture_validation.json"
        tampered_rc, _ = run_command(
            [
                sys.executable,
                str(SCRIPT_DIR / "validate_standard_map_data.py"),
                str(rebuilt),
                "--json-output",
                str(tampered_json),
            ],
            artifact_dir / "tampered_fixture_validation.log",
            timeout=args.command_timeout,
        )
        try:
            tampered_summary = load_json(tampered_json)["summary"]
        except Exception:
            tampered_summary = {"pass": 0, "fail": 0, "pending": 0}
        negative_ok = tampered_rc == 1 and tampered_summary.get("fail", 0) >= 1
        add(
            checks,
            "LOCAL-08",
            "冻结校验器故障注入拦截",
            negative_ok,
            f"expected_rc=1, actual_rc={tampered_rc}, summary={tampered_summary}",
            tampered_json,
        )

        offline_dir = temp_root / "offline_acceptance"
        offline_rc, _ = run_command(
            [
                sys.executable,
                str(SCRIPT_DIR / "run_map_manager_acceptance.py"),
                "--maps-root",
                str(maps_root),
                "--manifest",
                str(manifest_path),
                "--artifact-dir",
                str(offline_dir),
                "--skip-live",
                "--require-live",
            ],
            artifact_dir / "offline_acceptance.log",
            timeout=args.command_timeout,
        )
        try:
            offline = load_json(offline_dir / "summary.json")
            pending_count = sum(
                item.get("status") == "PENDING_ENV"
                for item in offline.get("automated_tests", [])
            )
            automated_only = set(offline) == {
                "metadata",
                "overall",
                "counts",
                "fixture_summary",
                "ros_graph",
                "stages",
                "automated_tests",
            } and set(offline.get("counts", {})) == {
                "automated_total",
                "automated_pass",
                "automated_fail",
                "automated_error",
                "automated_pending",
            }
            from run_map_manager_acceptance import (
                BASELINE_SERVICES,
                ROLLBACK_CANDIDATES,
                evaluate_ros_graph,
                graph_pending_detail,
                graph_probe_satisfied,
            )

            baseline_graph = evaluate_ros_graph(
                {"/map_manager/map_state"},
                set(BASELINE_SERVICES),
            )
            complete_graph = evaluate_ros_graph(
                {"/map_manager/map_state", "/map"},
                set(BASELINE_SERVICES)
                | {ROLLBACK_CANDIDATES[0]},
            )
            baseline_graph["timeout_sec"] = 30.0
            pending_detail = graph_pending_detail(
                baseline_graph,
                require_data=True,
            )
            staged_graph_wait_ok = (
                graph_probe_satisfied(
                    baseline_graph,
                    require_data=False,
                )
                and not graph_probe_satisfied(
                    baseline_graph,
                    require_data=True,
                )
                and graph_probe_satisfied(
                    complete_graph,
                    require_data=True,
                )
                and "/map (topic)" in pending_detail
                and "/map_asset_manager/" not in pending_detail
                and ROLLBACK_CANDIDATES[0] in pending_detail
            )
            offline_ok = (
                offline_rc == 2
                and offline.get("overall") == "INCOMPLETE"
                and offline.get("fixture_summary")
                == {"pass": 87, "fail": 0, "pending": 0}
                and pending_count == 15
                and automated_only
                and staged_graph_wait_ok
            )
            offline_detail = (
                f"returncode={offline_rc}, overall={offline.get('overall')}, "
                f"pending={pending_count}, automated_only={automated_only}, "
                f"staged_graph_wait={staged_graph_wait_ok}"
            )
            shutil.copy2(
                offline_dir / "summary.json",
                artifact_dir / "offline_acceptance.summary.json",
            )
            shutil.copy2(
                offline_dir / "summary.md",
                artifact_dir / "offline_acceptance.summary.md",
            )
        except Exception as exc:
            offline_ok = False
            offline_detail = f"returncode={offline_rc}, error={exc}"
        add(
            checks,
            "LOCAL-09",
            "统一 runner 离线门禁与分阶段 ROS 图就绪",
            offline_ok,
            offline_detail,
            artifact_dir / "offline_acceptance.summary.json",
        )

        if package_path is not None and not args.skip_build:
            colcon = shutil.which("colcon")
            if (
                not colcon
                or not (package_path / "package.xml").is_file()
            ):
                build_rc = 127
                build_detail = (
                    f"colcon={colcon}, package_xml="
                    f"{bool(package_path and (package_path / 'package.xml').is_file())}"
                )
                (artifact_dir / "map_manager_colcon_build.log").write_text(
                    build_detail + "\n",
                    encoding="utf-8",
                )
            else:
                build_rc, build_output = run_command(
                    [
                        colcon,
                        "--log-base",
                        str(temp_root / "colcon_log"),
                        "build",
                        "--base-paths",
                        str(package_path),
                        "--build-base",
                        str(temp_root / "colcon_build"),
                        "--install-base",
                        str(temp_root / "colcon_install"),
                        "--packages-select",
                        "map_manager",
                        "--event-handlers",
                        "console_direct+",
                    ],
                    artifact_dir / "map_manager_colcon_build.log",
                    timeout=args.command_timeout,
                )
                build_detail = (
                    f"returncode={build_rc}, "
                    f"finished={'Finished <<< map_manager' in build_output}"
                )
            add(
                checks,
                "LOCAL-10",
                "map_manager 隔离 colcon 构建",
                build_rc == 0,
                build_detail,
                artifact_dir / "map_manager_colcon_build.log",
            )
        else:
            install_ok, install_detail, _ = installed_layout_check(
                private_script_names
            )
            (artifact_dir / "map_manager_install_layout.log").write_text(
                install_detail + "\n",
                encoding="utf-8",
            )
            add(
                checks,
                "LOCAL-10",
                "map_manager 安装态完整性",
                install_ok,
                install_detail,
                artifact_dir / "map_manager_install_layout.log",
            )

    after_hashes = tree_hashes(maps_root)
    (artifact_dir / "protected_maps.after.json").write_text(
        json.dumps(after_hashes, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    add(
        checks,
        "LOCAL-11",
        "生产基准与测试地图零修改",
        before_hashes == after_hashes and bool(before_hashes),
        f"before_files={len(before_hashes)}, after_files={len(after_hashes)}",
        "protected_maps.before.json / protected_maps.after.json",
    )

    metadata = {
        "executed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "python": sys.version.split()[0],
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "maps_root": str(maps_root),
        "source_map": str(source_map),
        "manifest": str(manifest_path),
        "map_manager_repo": str(map_repo) if map_repo is not None else "installed",
        "map_manager_branch": (
            git_value(map_repo, "branch", "--show-current")
            if map_repo is not None
            else "installed"
        ),
        "map_manager_commit": (
            git_value(map_repo, "rev-parse", "--short", "HEAD")
            if map_repo is not None
            else "unknown"
        ),
        "map_manager_install_prefix": str(
            installed_package_prefix() or "unknown"
        ),
        "interfaces_repo": (
            str(nav_repo) if nav_repo is not None else "installed"
        ),
        "interfaces_branch": (
            git_value(nav_repo, "branch", "--show-current")
            if nav_repo is not None
            else "installed"
        ),
        "interfaces_commit": (
            git_value(nav_repo, "rev-parse", "--short", "HEAD")
            if nav_repo is not None
            else "unknown"
        ),
    }
    payload = save_summary(artifact_dir, checks, metadata)
    print(f"overall: {payload['overall']}")
    print(
        f"local checks: PASS={payload['counts']['pass']} "
        f"FAIL={payload['counts']['fail']} "
        f"TOTAL={payload['counts']['total']}"
    )
    print(f"result: {artifact_dir}")
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

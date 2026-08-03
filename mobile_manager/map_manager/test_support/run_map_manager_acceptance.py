#!/usr/bin/env python3
"""Private runner for all automated map_manager acceptance tests.

The unified entry point composes:

* frozen fixture validation;
* read-only live-stack preflight for TC-01~04;
* mutation-safe data-driven tests for TC-05~15;

Missing runtime infrastructure is reported as PENDING_ENV, never as a passing
functional test. The data runner still requires --confirm-mutations.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from map_manager_test_paths import (
    DEFAULT_MAPS_ROOT,
    DEFAULT_RESULT_ROOT,
    default_manifest_path,
)

SCRIPT_DIR = Path(__file__).resolve().parent

AUTOMATED_TCS = {
    "TC-01": "MapState、topic 与服务契约",
    "TC-02": "transient-local MapState late subscriber",
    "TC-03": "查询服务与当前地图一致",
    "TC-04": "MapState 发布状态记录",
    "TC-05": "旧扁平地图首次加载自动迁移",
    "TC-06": "旧图迁移幂等",
    "TC-07": "新分层地图不迁移",
    "TC-08": "同名重载 epoch 与资产内容",
    "TC-09": "仅 MapState 触发定位重载",
    "TC-10": "底图与资产组合回退",
    "TC-11": "地图 A 切换到 B",
    "TC-12": "地图 B 切回 A",
    "TC-13": "快速连续 A→B→A 收敛",
    "TC-14": "损坏地图失败保护",
    "TC-15": "当前地图 rename 与恢复",
}

PREFLIGHT_TCS = {"TC-01", "TC-02", "TC-03", "TC-04"}
DATA_TCS = set(AUTOMATED_TCS) - PREFLIGHT_TCS

TC_PLANS = {
    "TC-01": [
        "读取 MapState 字段并与接口契约比较",
        "读取 ROS topic 图并确认仅保留 map_state；发现旧 topic 时记录全部端点",
        "读取 map_manager 服务图并核对 8 个保留服务",
    ],
    "TC-02": [
        "创建 TRANSIENT_LOCAL late subscriber",
        "检查立即收到的 folder/name/epoch",
    ],
    "TC-03": [
        "读取当前 MapState",
        "调用当前地图 info/folder/name/list/data 服务",
        "逐字段比较服务结果、MapState 与栅格尺寸",
    ],
    "TC-04": [
        "读取 /map_manager/map_state 发布端点",
        "记录 publisher 节点；不检查订阅者数量或节点名称",
    ],
    "TC-05": [
        "检查旧扁平目录初始形态",
        "调用 load_map 加载 mm_test_legacy",
        "检查文件迁入 slam/assets 且 MapState 生效",
    ],
    "TC-06": [
        "记录已迁移目录哈希",
        "再次调用 load_map",
        "比较重载前后文件集合和哈希",
    ],
    "TC-07": [
        "记录 mm_test_a 加载前目录布局与 slam 哈希",
        "调用 load_map 加载新分层地图",
        "检查布局与 slam 零扰动；资产写回只记录不判失败",
    ],
    "TC-08": [
        "加载 mm_test_a 并记录 epoch",
        "同名再次调用 load_map",
        "检查 epoch 精确 +1",
        "查询资产服务并核对 canary 与数量",
    ],
    "TC-09": [
        "加载 mm_test_a 并记录 epoch",
        "扫描旧 set_map_changed push 服务",
        "仅调用 load_map 并检查 MapState +1 与耗时",
    ],
    "TC-10": [
        "加载 mm_test_a 并记录 epoch",
        "调用 command_board 组合回退服务",
        "记录服务响应并等待新 MapState",
        "比较回退底图字节、完整资产 JSON 语义并查询 canary",
    ],
    "TC-11": [
        "加载 A 并记录栅格 digest",
        "加载 B 并记录 MapState/栅格 digest",
        "核对 epoch、栅格差异和 B 资产 canary",
    ],
    "TC-12": [
        "加载 B 并记录栅格 digest",
        "加载 A 并检查 epoch、栅格差异和 A 资产 canary",
    ],
    "TC-13": [
        "加载 A 并记录起始 epoch",
        "连续执行 A→B→A",
        "检查 epoch 连续 +1 且最终收敛到 A",
    ],
    "TC-14": [
        "加载 A 并记录状态与栅格 digest",
        "调用 load_map 加载缺 map.yaml 的损坏图",
        "检查服务失败且状态/epoch/栅格不变",
    ],
    "TC-15": [
        "加载 A 并记录目录哈希",
        "执行 A→临时名 rename 并等待 MapState",
        "执行临时名→A rename",
        "检查 epoch、名称和目录哈希完全恢复",
    ],
}

BASELINE_SERVICES = {
    "/map_manager/get_current_map_data",
    "/map_manager/get_current_map_info",
    "/map_manager/get_map_folder",
    "/map_manager/get_map_name",
    "/map_manager/get_map_list",
    "/map_manager/load_map",
    "/map_manager/remove_map",
    "/map_manager/rename_map",
}

DATA_SERVICES = {
    "/map_asset_manager/get_all_nav_points",
    "/map_asset_manager/get_all_routes",
    "/map_asset_manager/get_all_areas",
}

ROLLBACK_CANDIDATES = (
    "/command_board/rollback_map",
    "/command_board_manager/rollback_map",
)


@dataclass
class TcResult:
    tc: str
    name: str
    kind: str
    status: str
    detail: str


@dataclass
class StageResult:
    name: str
    status: str
    detail: str
    command: list[str]
    returncode: int | None
    log: str


def run_command(
    name: str,
    command: list[str],
    log_path: Path,
    timeout: float,
) -> StageResult:
    print(f"\n╔═ STAGE {name}", flush=True)
    print(f"║ COMMAND {' '.join(command)}", flush=True)
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = completed.stdout
        if completed.stderr:
            output += ("\n" if output else "") + completed.stderr
        log_path.write_text(output, encoding="utf-8")
        if output:
            print(output.rstrip(), flush=True)
        result = StageResult(
            name=name,
            status="PASS" if completed.returncode == 0 else "FAIL",
            detail=f"returncode={completed.returncode}",
            command=command,
            returncode=completed.returncode,
            log=str(log_path),
        )
        print(
            f"╚═ STAGE RESULT [{result.status}] {result.detail}",
            flush=True,
        )
        return result
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") + ("\n" if exc.stdout else "") + (exc.stderr or "")
        log_path.write_text(output, encoding="utf-8")
        result = StageResult(
            name=name,
            status="ERROR",
            detail=f"timeout after {timeout:.1f}s",
            command=command,
            returncode=None,
            log=str(log_path),
        )
        print(
            f"╚═ STAGE RESULT [{result.status}] {result.detail}",
            flush=True,
        )
        return result
    except Exception as exc:
        log_path.write_text(repr(exc) + "\n", encoding="utf-8")
        result = StageResult(
            name=name,
            status="ERROR",
            detail=repr(exc),
            command=command,
            returncode=None,
            log=str(log_path),
        )
        print(
            f"╚═ STAGE RESULT [{result.status}] {result.detail}",
            flush=True,
        )
        return result


def evaluate_ros_graph(
    topics: set[str],
    services: set[str],
    nodes: set[str] | None = None,
) -> dict[str, Any]:
    """Evaluate one ROS graph snapshot without imposing a startup order."""
    rollback_service = next(
        (name for name in ROLLBACK_CANDIDATES if name in services),
        "",
    )
    missing_baseline_services = sorted(BASELINE_SERVICES - services)
    missing_baseline_requirements = list(missing_baseline_services)
    if "/map_manager/map_state" not in topics:
        missing_baseline_requirements.append("/map_manager/map_state (topic)")

    missing_data_services = sorted(DATA_SERVICES - services)
    missing_data_requirements = [
        *missing_baseline_requirements,
        *missing_data_services,
    ]
    if not rollback_service:
        missing_data_requirements.append(
            "one of "
            + ", ".join(ROLLBACK_CANDIDATES)
            + " (service)"
        )
    if "/map" not in topics:
        missing_data_requirements.append("/map (topic)")

    baseline_ready = not missing_baseline_requirements
    data_ready = not missing_data_requirements
    if data_ready:
        status = "READY"
    elif baseline_ready:
        status = "BASELINE_READY"
    else:
        status = "NOT_READY"
    return {
        "status": status,
        "baseline_ready": baseline_ready,
        "data_ready": data_ready,
        "rollback_service": rollback_service,
        "missing_baseline_services": missing_baseline_services,
        "missing_baseline_requirements": missing_baseline_requirements,
        "missing_data_services": missing_data_services,
        "missing_data_requirements": missing_data_requirements,
        "topics": sorted(topics),
        "services": sorted(services),
        "nodes": sorted(nodes or set()),
    }


def graph_probe_satisfied(
    graph: dict[str, Any],
    *,
    require_data: bool,
) -> bool:
    return bool(
        graph.get("data_ready")
        if require_data
        else graph.get("baseline_ready")
    )


def graph_pending_detail(
    graph: dict[str, Any],
    *,
    require_data: bool,
) -> str:
    if graph.get("error"):
        return f"ROS graph 探测异常: {graph['error']}"
    if graph.get("status") == "SKIPPED":
        return "ROS graph 探测已跳过"
    key = (
        "missing_data_requirements"
        if require_data
        else "missing_baseline_requirements"
    )
    missing = graph.get(key, [])
    wait_for = "完整数据测试依赖" if require_data else "map_manager 基础接口"
    timeout = float(graph.get("timeout_sec", 0.0))
    return (
        f"{wait_for}在 {timeout:.1f}s 内未就绪; "
        f"missing={missing}"
    )


def probe_ros_graph(
    timeout: float,
    *,
    require_data: bool,
) -> dict[str, Any]:
    from map_manager_auto_test import RosHarness

    harness: RosHarness | None = None
    started_at = time.monotonic()
    wait_mode = "data" if require_data else "baseline"
    try:
        harness = RosHarness(timeout)
        deadline = started_at + timeout
        topics: set[str] = set()
        services: set[str] = set()
        nodes: set[str] = set()
        graph = evaluate_ros_graph(topics, services, nodes)
        attempts = 0
        while time.monotonic() < deadline:
            harness.rclpy.spin_once(harness.node, timeout_sec=0.1)
            attempts += 1
            topics = {
                name for name, _ in harness.node.get_topic_names_and_types()
            }
            services = {
                name for name, _ in harness.node.get_service_names_and_types()
            }
            nodes = {
                f"{namespace.rstrip('/')}/{name}"
                if namespace != "/"
                else f"/{name}"
                for name, namespace in harness.node.get_node_names_and_namespaces()
            }
            graph = evaluate_ros_graph(topics, services, nodes)
            if graph_probe_satisfied(graph, require_data=require_data):
                break
        graph["wait_mode"] = wait_mode
        graph["timeout_sec"] = timeout
        graph["elapsed_sec"] = round(time.monotonic() - started_at, 3)
        graph["attempts"] = attempts
        return graph
    except Exception as exc:
        return {
            "status": "ERROR",
            "baseline_ready": False,
            "data_ready": False,
            "rollback_service": "",
            "error": repr(exc),
            "wait_mode": wait_mode,
            "timeout_sec": timeout,
            "elapsed_sec": round(time.monotonic() - started_at, 3),
        }
    finally:
        if harness is not None:
            harness.close()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def summarize_runner(
    result_path: Path,
    expected_tcs: set[str],
    tc_results: dict[str, TcResult],
) -> None:
    payload = load_json(result_path)
    checks = payload.get("checks", [])
    grouped: dict[str, list[dict[str, Any]]] = {}
    for check in checks:
        tc = str(check.get("tc", ""))
        if tc in expected_tcs:
            grouped.setdefault(tc, []).append(check)

    for tc in sorted(expected_tcs):
        items = grouped.get(tc, [])
        if not items:
            tc_results[tc] = TcResult(
                tc,
                AUTOMATED_TCS[tc],
                "AUTO",
                "ERROR",
                f"runner 未生成 {tc} 检查项",
            )
            continue
        statuses = {str(item.get("status", "ERROR")) for item in items}
        if statuses == {"PASS"}:
            status = "PASS"
        elif "ERROR" in statuses:
            status = "ERROR"
        else:
            status = "FAIL"
        details = "; ".join(
            (
                f"{item.get('name')}: {item.get('status')} "
                f"({item.get('detail', '')})"
            )
            for item in items
        )
        tc_results[tc] = TcResult(
            tc,
            AUTOMATED_TCS[tc],
            "AUTO",
            status,
            details,
        )


def escaped(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", "<br>")


def save_summary(
    artifact_dir: Path,
    metadata: dict[str, Any],
    stages: list[StageResult],
    tc_results: dict[str, TcResult],
    fixture_summary: dict[str, int],
    graph: dict[str, Any],
) -> dict[str, Any]:
    auto_results = [tc_results[tc] for tc in AUTOMATED_TCS]
    counts = {
        "automated_total": len(auto_results),
        "automated_pass": sum(item.status == "PASS" for item in auto_results),
        "automated_fail": sum(item.status == "FAIL" for item in auto_results),
        "automated_error": sum(item.status == "ERROR" for item in auto_results),
        "automated_pending": sum(
            item.status.startswith("PENDING") for item in auto_results
        ),
    }
    if counts["automated_fail"] or counts["automated_error"] or fixture_summary["fail"]:
        overall = "FAIL"
    elif counts["automated_pass"] == counts["automated_total"]:
        overall = "AUTOMATED_PASS"
    else:
        overall = "INCOMPLETE"

    payload = {
        "metadata": metadata,
        "overall": overall,
        "counts": counts,
        "fixture_summary": fixture_summary,
        "ros_graph": graph,
        "stages": [asdict(stage) for stage in stages],
        "automated_tests": [asdict(item) for item in auto_results],
    }
    (artifact_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# map_manager M5 自动化测试总结果",
        "",
        "> 由 map_manager 自动测试脚本生成。",
        "",
        f"- 执行时间：{metadata['executed_at']}",
        f"- 总体状态：**{overall}**",
        "- 完整可视化流程：`test_flow.md`",
        (
            f"- 自动 TC：总数 {counts['automated_total']} / "
            f"PASS {counts['automated_pass']} / "
            f"FAIL {counts['automated_fail']} / "
            f"ERROR {counts['automated_error']} / "
            f"PENDING {counts['automated_pending']}"
        ),
        (
            f"- 冻结数据检查：PASS {fixture_summary['pass']} / "
            f"FAIL {fixture_summary['fail']} / "
            f"PENDING {fixture_summary['pending']}"
        ),
        "",
        "## 自动化测试结果",
        "",
        "| TC | 测试项目 | 结果 | 证据/原因 |",
        "|---|---|---|---|",
    ]
    for item in auto_results:
        lines.append(
            f"| {item.tc} | {escaped(item.name)} | {item.status} | "
            f"{escaped(item.detail)} |"
        )
    lines.extend(
        [
            "",
            "## 阶段结果",
            "",
            "| 阶段 | 结果 | 详情 | 日志 |",
            "|---|---|---|---|",
        ]
    )
    for stage in stages:
        lines.append(
            f"| {stage.name} | {stage.status} | {escaped(stage.detail)} | "
            f"`{Path(stage.log).name}` |"
        )
    if overall == "INCOMPLETE":
        lines.extend(
            [
                "",
                "> 当前结果只证明冻结数据与自动化入口就绪。ROS2 完整栈未就绪时，"
                "PENDING_ENV 不得记为 TC 通过。",
            ]
        )
    (artifact_dir / "summary.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    save_test_flow(
        artifact_dir,
        metadata,
        stages,
        auto_results,
    )
    return payload


def save_test_flow(
    artifact_dir: Path,
    metadata: dict[str, Any],
    stages: list[StageResult],
    auto_results: list[TcResult],
) -> None:
    flow_events: dict[str, list[dict[str, Any]]] = {}
    for result_path in (
        artifact_dir / "preflight/result.json",
        artifact_dir / "data_driven/result.json",
    ):
        payload = load_json(result_path)
        for event in payload.get("flow", []):
            tc = str(event.get("tc", ""))
            if tc in AUTOMATED_TCS:
                flow_events.setdefault(tc, []).append(event)

    lines = [
        "# map_manager M5 测试流程可视化",
        "",
        f"- 执行时间：{metadata['executed_at']}",
        "- 本文由测试脚本生成，逐项展示计划动作、实际动作、检查和结果。",
        "",
        "## 总流程",
        "",
        "```mermaid",
        "flowchart TD",
        '    START(["开始"]) --> PREP["准备受管测试地图"]',
        '    PREP --> FIXTURE["冻结数据 87 项校验"]',
        '    FIXTURE --> GRAPH["ROS graph 就绪检查"]',
        '    GRAPH --> PREFLIGHT["TC-01~04 只读预检"]',
        '    PREFLIGHT --> DATA["TC-05~15 数据驱动测试"]',
        '    DATA --> RESTORE["恢复原地图与测试夹具"]',
        '    RESTORE --> END(["结束并回填"])',
        "```",
        "",
        "## 阶段执行结果",
        "",
        "| 阶段 | 结果 | 命令 | 详情 |",
        "|---|---|---|---|",
    ]
    for stage in stages:
        command = " ".join(stage.command)
        lines.append(
            f"| {stage.name} | {stage.status} | "
            f"`{escaped(command)}` | {escaped(stage.detail)} |"
        )

    for item in auto_results:
        lines.extend(
            [
                "",
                f"## {item.tc} {item.name}",
                "",
                f"**最终结果：{item.status}**",
                "",
                "**计划动作**",
                "",
            ]
        )
        for index, plan in enumerate(TC_PLANS[item.tc], 1):
            lines.append(f"{index}. {plan}")
        events = flow_events.get(item.tc, [])
        lines.extend(
            [
                "",
                "**实际执行流水**",
                "",
                "| 步骤 | 类型 | 动作 / 检查 | 结果 | 详细证据 |",
                "|---:|---|---|---|---|",
            ]
        )
        actual_events = [
            event
            for event in events
            if event.get("kind") not in {"CASE", "RESULT"}
        ]
        if not actual_events:
            lines.append(
                f"| — | NOT_RUN | 未产生执行流水 | {item.status} | "
                f"{escaped(item.detail)} |"
            )
        else:
            for event in actual_events:
                lines.append(
                    f"| {event.get('step', '—')} | "
                    f"{escaped(str(event.get('kind', '')))} | "
                    f"{escaped(str(event.get('name', '')))} | "
                    f"{escaped(str(event.get('status', '')))} | "
                    f"{escaped(str(event.get('detail', '')))} |"
                )

    (artifact_dir / "test_flow.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
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
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--graph-timeout",
        type=float,
        default=30.0,
        help="seconds to wait for baseline and then full data-test ROS graph",
    )
    parser.add_argument("--stage-timeout", type=float, default=300.0)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument(
        "--confirm-mutations",
        action="store_true",
        help="allow the data runner to load/migrate/rollback/rename mm_test_*",
    )
    parser.add_argument(
        "--skip-live",
        action="store_true",
        help="only validate fixtures and mark all automated TCs PENDING_ENV",
    )
    parser.add_argument(
        "--require-live",
        action="store_true",
        help="return exit code 2 when the full live stack is not ready",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    artifact_dir = args.artifact_dir or (
        DEFAULT_RESULT_ROOT / f"acceptance_{timestamp}"
    )
    artifact_dir.mkdir(parents=True, exist_ok=False)
    stages: list[StageResult] = []
    tc_results = {
        tc: TcResult(
            tc,
            name,
            "AUTO",
            "PENDING_ENV",
            "ROS2 完整栈尚未执行",
        )
        for tc, name in AUTOMATED_TCS.items()
    }

    fixture_json = artifact_dir / "fixture_validation.json"
    fixture_stage = run_command(
        "fixture_validation",
        [
            sys.executable,
            str(SCRIPT_DIR / "validate_standard_map_data.py"),
            "--maps-root",
            str(args.maps_root),
            "--manifest",
            str(args.manifest),
            "--json-output",
            str(fixture_json),
            "--console-flow",
        ],
        artifact_dir / "fixture_validation.stdout.log",
        args.stage_timeout,
    )
    stages.append(fixture_stage)
    fixture_payload = load_json(fixture_json)
    fixture_summary = fixture_payload.get(
        "summary", {"pass": 0, "fail": 1, "pending": 0}
    )

    initial_graph: dict[str, Any]
    if args.skip_live:
        initial_graph = {
            "status": "SKIPPED",
            "baseline_ready": False,
            "data_ready": False,
            "wait_mode": "baseline",
            "timeout_sec": 0.0,
        }
    else:
        initial_graph = probe_ros_graph(
            args.graph_timeout,
            require_data=False,
        )
    (artifact_dir / "ros_graph_initial.json").write_text(
        json.dumps(initial_graph, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    if initial_graph.get("baseline_ready"):
        preflight_dir = artifact_dir / "preflight"
        preflight_stage = run_command(
            "live_preflight",
            [
                sys.executable,
                str(SCRIPT_DIR / "map_manager_auto_test.py"),
                "preflight",
                "--timeout",
                str(args.timeout),
                "--artifact-dir",
                str(preflight_dir),
            ],
            artifact_dir / "preflight.stdout.log",
            args.stage_timeout,
        )
        stages.append(preflight_stage)
        summarize_runner(
            preflight_dir / "result.json",
            PREFLIGHT_TCS,
            tc_results,
        )
    else:
        detail = graph_pending_detail(
            initial_graph,
            require_data=False,
        )
        stages.append(
            StageResult(
                "live_preflight",
                "PENDING_ENV",
                str(detail),
                [],
                None,
                str(artifact_dir / "ros_graph_initial.json"),
            )
        )

    graph = initial_graph
    if (
        not args.skip_live
        and not fixture_summary.get("fail", 0)
        and initial_graph.get("baseline_ready")
        and not initial_graph.get("data_ready")
    ):
        graph = probe_ros_graph(
            args.graph_timeout,
            require_data=True,
        )
    (artifact_dir / "ros_graph.json").write_text(
        json.dumps(graph, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    if fixture_summary.get("fail", 0):
        for tc in DATA_TCS:
            tc_results[tc].status = "PENDING_DATA"
            tc_results[tc].detail = "冻结测试数据校验失败"
        stages.append(
            StageResult(
                "data_driven",
                "PENDING_DATA",
                "fixture validation failed",
                [],
                None,
                str(fixture_json),
            )
        )
    elif not graph.get("data_ready"):
        detail = graph_pending_detail(
            graph,
            require_data=True,
        )
        for tc in DATA_TCS:
            tc_results[tc].status = "PENDING_ENV"
            tc_results[tc].detail = detail
        stages.append(
            StageResult(
                "data_driven",
                "PENDING_ENV",
                str(detail),
                [],
                None,
                str(artifact_dir / "ros_graph.json"),
            )
        )
    elif not args.confirm_mutations:
        for tc in DATA_TCS:
            tc_results[tc].status = "PENDING_CONFIRMATION"
            tc_results[tc].detail = "需传 --confirm-mutations"
        stages.append(
            StageResult(
                "data_driven",
                "PENDING_CONFIRMATION",
                "pass --confirm-mutations to execute mm_test_* operations",
                [],
                None,
                str(artifact_dir / "ros_graph.json"),
            )
        )
    else:
        data_dir = artifact_dir / "data_driven"
        data_stage = run_command(
            "data_driven",
            [
                sys.executable,
                str(SCRIPT_DIR / "map_manager_data_test.py"),
                "--maps-root",
                str(args.maps_root),
                "--manifest",
                str(args.manifest),
                "--timeout",
                str(args.timeout),
                "--artifact-dir",
                str(data_dir),
                "--rollback-service",
                str(graph["rollback_service"]),
                "--confirm-mutations",
            ],
            artifact_dir / "data_driven.stdout.log",
            args.stage_timeout,
        )
        stages.append(data_stage)
        summarize_runner(
            data_dir / "result.json",
            DATA_TCS,
            tc_results,
        )

    metadata = {
        "executed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "maps_root": str(args.maps_root.resolve()),
        "manifest": str(args.manifest.resolve()),
        "confirm_mutations": args.confirm_mutations,
        "graph_timeout_sec": args.graph_timeout,
    }
    payload = save_summary(
        artifact_dir,
        metadata,
        stages,
        tc_results,
        fixture_summary,
        graph,
    )
    print("\n╔═ FINAL AUTOMATED TEST RESULTS")
    for item in payload["automated_tests"]:
        print(
            f"║ [{item['status']}] {item['tc']} {item['name']} — "
            f"{item['detail']}"
        )
    print("╚═ END AUTOMATED TEST RESULTS")
    print(f"overall: {payload['overall']}")
    print(
        "automated: "
        f"PASS={payload['counts']['automated_pass']} "
        f"FAIL={payload['counts']['automated_fail']} "
        f"ERROR={payload['counts']['automated_error']} "
        f"PENDING={payload['counts']['automated_pending']}"
    )
    print(
        "fixtures: "
        f"PASS={fixture_summary['pass']} "
        f"FAIL={fixture_summary['fail']} "
        f"PENDING={fixture_summary['pending']}"
    )
    print(f"result: {artifact_dir}")

    if payload["overall"] == "FAIL":
        return 1
    if args.require_live and payload["overall"] == "INCOMPLETE":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

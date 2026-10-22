#!/usr/bin/env python3
"""Private live-stack automatic checks.

Read-only preflight covers TC-01 through TC-04.
The optional smoke mode additionally checks the automatic part of TC-08 by
reloading the current map and asserting epoch + 1. It deliberately requires an
explicit confirmation flag because load_map mutates runtime state.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from map_manager_test_paths import DEFAULT_RESULT_ROOT


@dataclass
class Check:
    tc: str
    name: str
    status: str
    detail: str


@dataclass
class FlowEvent:
    sequence: int
    tc: str
    step: int
    kind: str
    name: str
    status: str
    detail: str


class Report:
    def __init__(self) -> None:
        self.checks: list[Check] = []
        self.flow: list[FlowEvent] = []
        self.case_titles: dict[str, str] = {}
        self.case_order: list[str] = []
        self.case_steps: dict[str, int] = {}
        self.active_cases: set[str] = set()

    def _event(
        self,
        tc: str,
        kind: str,
        name: str,
        status: str,
        detail: str,
        *,
        advance: bool = True,
    ) -> None:
        if advance:
            self.case_steps[tc] = self.case_steps.get(tc, 0) + 1
        event = FlowEvent(
            sequence=len(self.flow) + 1,
            tc=tc,
            step=self.case_steps.get(tc, 0),
            kind=kind,
            name=name,
            status=status,
            detail=detail,
        )
        self.flow.append(event)
        if kind == "ACTION":
            print(
                f"│ {event.step:02d} ▶ ACTION {name}"
                + (f" — {detail}" if detail else ""),
                flush=True,
            )
        elif kind == "CHECK":
            marker = "✓" if status == "PASS" else "✗"
            print(
                f"│ {event.step:02d} {marker} CHECK [{status}] {name}"
                + (f" — {detail}" if detail else ""),
                flush=True,
            )
        elif kind == "ERROR":
            print(
                f"│ {event.step:02d} ! ERROR {name}"
                + (f" — {detail}" if detail else ""),
                flush=True,
            )

    def start_case(self, tc: str, title: str) -> None:
        if tc not in self.case_titles:
            self.case_titles[tc] = title
            self.case_order.append(tc)
        self.active_cases.add(tc)
        print(f"\n╭─ [{tc}] {title}", flush=True)
        self._event(tc, "CASE", title, "RUNNING", "", advance=False)

    def action(self, tc: str, name: str, detail: str = "") -> None:
        if tc not in self.active_cases:
            self.start_case(tc, self.case_titles.get(tc, name))
        self._event(tc, "ACTION", name, "RUNNING", detail)

    def add(self, tc: str, name: str, passed: bool, detail: str) -> None:
        status = "PASS" if passed else "FAIL"
        self.checks.append(Check(tc, name, status, detail))
        self._event(tc, "CHECK", name, status, detail)

    def error(self, tc: str, name: str, detail: str) -> None:
        self.checks.append(Check(tc, name, "ERROR", detail))
        self._event(tc, "ERROR", name, "ERROR", detail)

    def case_status(self, tc: str) -> str:
        statuses = [check.status for check in self.checks if check.tc == tc]
        if not statuses or "ERROR" in statuses:
            return "ERROR"
        if "FAIL" in statuses:
            return "FAIL"
        return "PASS"

    def finish_case(self, tc: str) -> None:
        status = self.case_status(tc)
        title = self.case_titles.get(tc, tc)
        self._event(tc, "RESULT", title, status, "", advance=False)
        self.active_cases.discard(tc)
        print(f"╰─ RESULT [{status}] {tc} {title}", flush=True)

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(c.status == "PASS" for c in self.checks)

    def save(self, artifact_dir: Path, metadata: dict[str, Any]) -> None:
        artifact_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "metadata": metadata,
            "summary": {
                "passed": self.passed,
                "pass": sum(c.status == "PASS" for c in self.checks),
                "fail": sum(c.status == "FAIL" for c in self.checks),
                "error": sum(c.status == "ERROR" for c in self.checks),
            },
            "checks": [asdict(c) for c in self.checks],
            "flow": [asdict(event) for event in self.flow],
        }
        (artifact_dir / "result.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        lines = [
            "# map_manager 自动测试结果",
            "",
            f"- 执行时间：{metadata['executed_at']}",
            f"- 模式：{metadata['mode']}",
            f"- 总结：{'PASS' if self.passed else 'FAIL'}",
            "- 详细流程：`flow.md`",
            "",
            "| TC | 检查项 | 结果 | 证据 |",
            "|---|---|---|---|",
        ]
        for check in self.checks:
            detail = check.detail.replace("|", "\\|").replace("\n", "<br>")
            lines.append(f"| {check.tc} | {check.name} | {check.status} | {detail} |")
        (artifact_dir / "result.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        self._save_flow(artifact_dir, metadata)

    @staticmethod
    def _escaped(value: str) -> str:
        return value.replace("|", "\\|").replace("\n", "<br>")

    def _save_flow(
        self, artifact_dir: Path, metadata: dict[str, Any]
    ) -> None:
        lines = [
            "# map_manager 测试执行流程",
            "",
            f"- 执行时间：{metadata['executed_at']}",
            f"- 模式：{metadata['mode']}",
            "",
            "## 流程总览",
            "",
            "```mermaid",
            "flowchart TD",
            '    START(["开始"])',
        ]
        previous = "START"
        for index, tc in enumerate(self.case_order, 1):
            node = f"N{index}"
            status = self.case_status(tc)
            title = self.case_titles.get(tc, tc).replace('"', "'")
            lines.append(
                f'    {node}["{tc} {title}<br/>{status}"]'
            )
            lines.append(f"    {previous} --> {node}")
            lines.append(f"    class {node} {status.lower()}")
            previous = node
        lines.extend(
            [
                '    END(["结束"])',
                f"    {previous} --> END",
                "    classDef pass fill:#d7f5df,stroke:#238636,color:#111",
                "    classDef fail fill:#ffd8d5,stroke:#cf222e,color:#111",
                "    classDef error fill:#ffe8c5,stroke:#bf8700,color:#111",
                "```",
            ]
        )
        for tc in self.case_order:
            lines.extend(
                [
                    "",
                    f"## {tc} {self.case_titles.get(tc, tc)}",
                    "",
                    f"**最终结果：{self.case_status(tc)}**",
                    "",
                    "| 步骤 | 类型 | 动作 / 检查 | 结果 | 详细证据 |",
                    "|---:|---|---|---|---|",
                ]
            )
            for event in self.flow:
                if event.tc != tc or event.kind in {"CASE", "RESULT"}:
                    continue
                lines.append(
                    f"| {event.step} | {event.kind} | "
                    f"{self._escaped(event.name)} | {event.status} | "
                    f"{self._escaped(event.detail)} |"
                )
        (artifact_dir / "flow.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )


def execute_case(
    report: Report,
    tc: str,
    title: str,
    callback: Callable[[], Any],
) -> Any:
    report.start_case(tc, title)
    try:
        return callback()
    except Exception as exc:
        report.error(tc, "执行异常", repr(exc))
        return None
    finally:
        report.finish_case(tc)


class RosHarness:
    def __init__(self, timeout: float) -> None:
        import rclpy
        from rclpy.node import Node

        self.rclpy = rclpy
        self.timeout = timeout
        rclpy.init(args=None)
        self.node: Node = rclpy.create_node("map_manager_self_test")

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()

    def wait_message(
        self,
        msg_type: Any,
        topic: str,
        *,
        transient_local: bool = False,
        predicate: Optional[Callable[[Any], bool]] = None,
    ) -> Any:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        if transient_local:
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        received: list[Any] = []

        def callback(msg: Any) -> None:
            if predicate is None or predicate(msg):
                received.append(msg)

        subscription = self.node.create_subscription(msg_type, topic, callback, qos)
        deadline = time.monotonic() + self.timeout
        try:
            while not received and time.monotonic() < deadline:
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)
        if not received:
            raise TimeoutError(f"{topic} did not produce a matching message")
        return received[-1]

    def call(self, srv_type: Any, service: str, request: Any) -> Any:
        client = self.node.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise TimeoutError(f"service unavailable: {service}")
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=self.timeout
            )
            if not future.done():
                raise TimeoutError(f"service timeout: {service}")
            response = future.result()
            if response is None:
                raise RuntimeError(f"service returned no response: {service}")
            return response
        finally:
            self.node.destroy_client(client)


def qualified_name(name: str, namespace: str) -> str:
    if namespace == "/":
        return f"/{name}"
    return f"{namespace.rstrip('/')}/{name}"


def topic_endpoint_details(
    node: Any,
    topics: dict[str, list[str]],
    topic_names: list[str],
) -> dict[str, dict[str, Any]]:
    details: dict[str, dict[str, Any]] = {}
    for topic in topic_names:
        try:
            publishers = node.get_publishers_info_by_topic(topic)
            subscribers = node.get_subscriptions_info_by_topic(topic)
            details[topic] = {
                "types": topics[topic],
                "publishers": sorted(
                    {
                        (
                            qualified_name(
                                endpoint.node_name,
                                endpoint.node_namespace,
                            ),
                            endpoint.topic_type,
                        )
                        for endpoint in publishers
                    }
                ),
                "subscribers": sorted(
                    {
                        (
                            qualified_name(
                                endpoint.node_name,
                                endpoint.node_namespace,
                            ),
                            endpoint.topic_type,
                        )
                        for endpoint in subscribers
                        if endpoint.node_name != "map_manager_self_test"
                    }
                ),
            }
        except Exception as exc:  # noqa: BLE001 - graph may change mid-query
            details[topic] = {
                "types": topics[topic],
                "introspection_error": str(exc),
            }
    return details


def run_preflight(harness: RosHarness, report: Report) -> Any:
    from nav_interfaces.msg import MapState
    from nav_interfaces.srv import GetMaps
    from nav_msgs.srv import GetMap

    def tc01() -> None:
        report.action("TC-01", "读取 MapState 接口字段", "调用类型支持 introspection")
        fields = MapState.get_fields_and_field_types()
        expected_fields = {
            "header": "std_msgs/Header",
            "map_folder": "string",
            "map_name": "string",
            "epoch": "uint64",
        }
        report.add(
            "TC-01",
            "MapState 字段",
            fields == expected_fields,
            f"actual={fields}, expected={expected_fields}",
        )

        report.action("TC-01", "读取 ROS topic 图", "检查新旧地图状态 topic")
        topics = {
            name: types for name, types in harness.node.get_topic_names_and_types()
        }
        legacy_topics = sorted(
            {
                "/map_manager/map_folder",
                "/map_manager/map_name",
            }
            & topics.keys()
        )
        if legacy_topics:
            report.action(
                "TC-01",
                "解析旧 topic 端点",
                "记录 publisher/subscriber 节点和消息类型",
            )
        legacy_endpoints = topic_endpoint_details(
            harness.node,
            topics,
            legacy_topics,
        )
        topic_ok = "/map_manager/map_state" in topics and not legacy_topics
        report.add(
            "TC-01",
            "topic 形态",
            topic_ok,
            (
                f"map_state_present={'/map_manager/map_state' in topics}, "
                f"legacy_topics={legacy_topics}, "
                "legacy_endpoints="
                f"{json.dumps(legacy_endpoints, ensure_ascii=False, sort_keys=True)}"
            ),
        )

        report.action("TC-01", "读取 map_manager 服务图", "核对 3 个保留服务（v1.11：get_maps / map_command / get_current_map_data）+ 旧服务已删")
        services = {
            name: types for name, types in harness.node.get_service_names_and_types()
        }
        expected_services = {
            "/map_manager/get_current_map_data",
            "/map_manager/get_maps",
            "/map_manager/map_command",
        }
        retired_services = {
            "/map_manager/get_current_map_info",
            "/map_manager/get_map_folder",
            "/map_manager/get_map_name",
            "/map_manager/get_map_list",
            "/map_manager/load_map",
            "/map_manager/remove_map",
            "/map_manager/rename_map",
        }
        missing_services = sorted(expected_services - services.keys())
        leftover_services = sorted(retired_services & services.keys())
        report.add(
            "TC-01",
            "3 个 map_manager 服务且旧服务已删",
            not missing_services and not leftover_services,
            f"missing={missing_services}, leftover={leftover_services}",
        )

    execute_case(report, "TC-01", "MapState、topic 与服务契约", tc01)

    def tc02() -> Any:
        report.action(
            "TC-02",
            "创建 late subscriber",
            "TRANSIENT_LOCAL 订阅 /map_manager/map_state",
        )
        current = harness.wait_message(
            MapState, "/map_manager/map_state", transient_local=True
        )
        state_ok = bool(
            current.map_folder and current.map_name and current.epoch >= 1
        )
        report.add(
            "TC-02",
            "latched MapState",
            state_ok,
            (
                f"folder={current.map_folder}, name={current.map_name}, "
                f"epoch={current.epoch}"
            ),
        )
        return current

    state = execute_case(
        report,
        "TC-02",
        "transient-local MapState late subscriber",
        tc02,
    )

    def tc03() -> None:
        current = state
        if current is None:
            report.action(
                "TC-03",
                "重新读取当前 MapState",
                "TC-02 未提供可复用状态",
            )
            current = harness.wait_message(
                MapState, "/map_manager/map_state", transient_local=True
            )
        report.action(
            "TC-03",
            "调用 2 个查询服务",
            "get_maps（folder / current / list）+ get_current_map_data",
        )
        info = harness.call(GetMaps, "/map_manager/get_maps", GetMaps.Request())
        grid = harness.call(
            GetMap, "/map_manager/get_current_map_data", GetMap.Request()
        )
        names = [item.name for item in info.maps]
        query_ok = (
            info.success
            and info.map_folder == current.map_folder
            and info.current_map_name == current.map_name
            and current.map_name in names
            and grid.map.info.width > 0
            and grid.map.info.height > 0
            and len(grid.map.data)
            == grid.map.info.width * grid.map.info.height
        )
        report.add(
            "TC-03",
            "查询服务与 MapState 一致",
            query_ok,
            (
                f"name={info.current_map_name}, folder={info.map_folder}, "
                f"maps={len(info.maps)}, "
                f"grid={grid.map.info.width}x{grid.map.info.height}"
            ),
        )

    execute_case(report, "TC-03", "查询服务与当前地图一致", tc03)

    def tc04() -> None:
        report.action(
            "TC-04",
            "读取 MapState 发布端点",
            "只记录 publisher，不检查订阅者数量或节点名称",
        )
        state_endpoints = harness.node.get_publishers_info_by_topic(
            "/map_manager/map_state"
        )
        state_publishers = sorted(
            {
                qualified_name(endpoint.node_name, endpoint.node_namespace)
                for endpoint in state_endpoints
                if endpoint.node_name != "map_manager_self_test"
            }
        )
        report.add(
            "TC-04",
            "MapState 发布状态",
            bool(state_publishers),
            f"map_state_publishers={state_publishers}",
        )

    execute_case(report, "TC-04", "MapState 发布状态记录", tc04)
    return state


def run_same_map_smoke(
    harness: RosHarness, report: Report, current_state: Any, map_name: str
) -> None:
    from nav_interfaces.msg import MapState
    from nav_interfaces.srv import MapManagerCommand

    target = map_name or current_state.map_name
    report.action(
        "TC-08-A",
        "校验重载目标",
        f"current={current_state.map_name}, requested={target}",
    )
    if target != current_state.map_name:
        raise ValueError(
            "safe smoke mode only permits the current map; "
            f"current={current_state.map_name}, requested={target}"
        )
    request = MapManagerCommand.Request()
    request.op = MapManagerCommand.Request.OP_LOAD
    request.map_name = target
    report.action(
        "TC-08-A",
        "调用 map_command{op: LOAD}",
        f"service=/map_manager/map_command, op=1, map_name={target}",
    )
    started = time.monotonic()
    response = harness.call(MapManagerCommand, "/map_manager/map_command", request)
    elapsed = time.monotonic() - started
    report.action(
        "TC-08-A",
        "等待新 MapState",
        f"predicate=epoch>{current_state.epoch}",
    )
    after = harness.wait_message(
        MapState,
        "/map_manager/map_state",
        transient_local=True,
        predicate=lambda msg: msg.epoch > current_state.epoch,
    )
    passed = (
        response.success
        and after.epoch == current_state.epoch + 1
        and after.map_name == target
    )
    report.add(
        "TC-08-A",
        "同名 load_map 的 epoch/name 自动断言",
        passed,
        (
            f"success={response.success}, epoch={current_state.epoch}->{after.epoch}, "
            f"name={after.map_name}, service_elapsed={elapsed:.3f}s"
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode",
        choices=("preflight", "safe-all"),
        help="preflight is read-only; safe-all also reloads the current map",
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--map-name", default="")
    parser.add_argument(
        "--confirm-live-reload",
        action="store_true",
        help="required for safe-all because it calls load_map on the current map",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=None,
        help="default: $MAP_MANAGER_TEST_RESULT_ROOT/preflight_<timestamp>",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.mode == "safe-all" and not args.confirm_live_reload:
        print("ERROR: safe-all requires --confirm-live-reload", file=sys.stderr)
        return 2

    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    artifact_dir = args.artifact_dir or (
        DEFAULT_RESULT_ROOT / f"preflight_{timestamp}"
    )
    report = Report()
    metadata = {
        "executed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": args.mode,
        "timeout_sec": args.timeout,
    }
    harness: Optional[RosHarness] = None
    try:
        harness = RosHarness(args.timeout)
        state = run_preflight(harness, report)
        if args.mode == "safe-all":
            if state is None:
                report.start_case("TC-08-A", "当前地图同名安全重载")
                report.error(
                    "TC-08-A",
                    "前置状态不可用",
                    "TC-02 未取得当前 MapState",
                )
                report.finish_case("TC-08-A")
            else:
                execute_case(
                    report,
                    "TC-08-A",
                    "当前地图同名安全重载",
                    lambda: run_same_map_smoke(
                        harness, report, state, args.map_name
                    ),
                )
    except Exception as exc:
        report.error("RUNNER", "未处理异常", repr(exc))
    finally:
        if harness is not None:
            harness.close()
        report.save(artifact_dir, metadata)

    print(
        "\nSUMMARY "
        f"PASS={sum(c.status == 'PASS' for c in report.checks)} "
        f"FAIL={sum(c.status == 'FAIL' for c in report.checks)} "
        f"ERROR={sum(c.status == 'ERROR' for c in report.checks)}"
    )
    print(f"artifacts: {artifact_dir}")
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

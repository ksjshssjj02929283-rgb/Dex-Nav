#!/usr/bin/env python3
"""Run isolated data-service checks for map_asset_manager."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


RESULT_ROOT = Path.home() / ".dexe/navigation/map_asset_manager_test"
NAMESPACE = "/map_asset_manager_self_test"
MAP_NAME = "mm_asset_test"
EXPECTED_COUNTS = {
    "points": 3,
    "routes": 1,
    "areas": 1,
    "rooms": 1,
    "junctions": 1,
    "edges": 2,
}
DATA_SERVICES = (
    "get_all_nav_points", "get_point", "set_point",
    "get_route", "set_route", "set_area", "set_room", "get_route_graph",
    "set_route_junction", "set_route_edge", "get_assets", "set_assets",
)
MANAGEMENT_SERVICES = ("save_assets",)
COVERED_SERVICES = DATA_SERVICES + MANAGEMENT_SERVICES


def fixture_path() -> Path:
    script = Path(__file__).resolve()
    candidates = (
        script.parent.parent / "test_data/map_assets_standard.json",
        script.parents[2]
        / "share/map_asset_manager/test_data/map_assets_standard.json",
    )
    return next((path for path in candidates if path.is_file()), candidates[0])


def node_executable() -> Path:
    candidates: list[Path] = []
    override = os.environ.get("MAP_ASSET_MANAGER_NODE")
    if override:
        candidates.append(Path(override))
    candidates.append(Path(__file__).resolve().parent / "map_asset_manager_node")
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(
                Path(prefix) / "lib/map_asset_manager/map_asset_manager_node"
            )
    resolved = shutil.which("map_asset_manager_node")
    if resolved:
        candidates.append(Path(resolved))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise FileNotFoundError(
        "map_asset_manager_node not found; source install or set "
        "MAP_ASSET_MANAGER_NODE"
    )


def validate_fixture(value: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if value.get("schema_version") != "2.0":
        errors.append("schema_version must be 2.0")
    if value.get("schema_compatible_with") != "spark_dsg/1.2":
        errors.append("schema_compatible_with must be spark_dsg/1.2")
    layers = value.get("dsg", {}).get("layers", {})
    actual = {
        "points": len(value.get("points", [])),
        "routes": len(value.get("routes", [])),
        "areas": len(value.get("areas", [])),
        "rooms": len(layers.get("ROOMS", [])),
        "junctions": len(value.get("route_graph", {}).get("junctions", [])),
        "edges": len(value.get("route_graph", {}).get("edges", [])),
    }
    if actual != EXPECTED_COUNTS:
        errors.append(f"counts mismatch: actual={actual}, expected={EXPECTED_COUNTS}")
    names = {item.get("name") for item in value.get("points", [])}
    expected_names = {
        "AUTO_ASSET_POINT_A", "AUTO_ASSET_POINT_B", "AUTO_ASSET_POINT_C"
    }
    if names != expected_names:
        errors.append(
            f"point names mismatch: actual={sorted(names)}, "
            f"expected={sorted(expected_names)}"
        )
    endpoints = {
        ("nav", item.get("name")): (
            float(item.get("pose", {}).get("x", 0.0)),
            float(item.get("pose", {}).get("y", 0.0)),
        )
        for item in value.get("points", [])
    }
    endpoints.update({
        ("junction", item.get("id")): (
            float(item.get("x", 0.0)), float(item.get("y", 0.0))
        )
        for item in value.get("route_graph", {}).get("junctions", [])
    })
    for edge in value.get("route_graph", {}).get("edges", []):
        edge_id = edge.get("id", "<unnamed>")
        from_key = (
            edge.get("from", {}).get("type"), edge.get("from", {}).get("id")
        )
        to_key = (
            edge.get("to", {}).get("type"), edge.get("to", {}).get("id")
        )
        if from_key not in endpoints or to_key not in endpoints:
            errors.append(
                f"edge {edge_id} references missing endpoint: "
                f"from={from_key}, to={to_key}"
            )
            continue
        control_points = edge.get("control_points", [])
        if edge.get("edge_type") != "BEZIER":
            continue
        if len(control_points) < 2:
            errors.append(f"BEZIER edge {edge_id} needs at least 2 control points")
            continue
        if any(
            abs(float(actual) - expected) > 1e-5
            for actual, expected in zip(control_points[0], endpoints[from_key])
        ):
            errors.append(f"BEZIER edge {edge_id} start is not snapped")
        if any(
            abs(float(actual) - expected) > 1e-5
            for actual, expected in zip(control_points[-1], endpoints[to_key])
        ):
            errors.append(f"BEZIER edge {edge_id} end is not snapped")
    return errors


class Report:
    def __init__(self) -> None:
        self.checks: list[dict[str, str]] = []
        self.flow: list[dict[str, Any]] = []
        self.titles: dict[str, str] = {}
        self.order: list[str] = []
        self.steps: dict[str, int] = {}

    def start(self, tc: str, title: str) -> None:
        self.titles[tc] = title
        self.order.append(tc)
        self.steps[tc] = 0
        print(f"\n╭─ [{tc}] {title}", flush=True)

    def event(
        self, tc: str, kind: str, name: str, status: str, detail: str
    ) -> None:
        self.steps[tc] = self.steps.get(tc, 0) + 1
        event = {
            "sequence": len(self.flow) + 1,
            "tc": tc,
            "step": self.steps[tc],
            "kind": kind,
            "name": name,
            "status": status,
            "detail": detail,
        }
        self.flow.append(event)
        if kind == "ACTION":
            print(f"│ {event['step']:02d} ▶ ACTION {name} — {detail}", flush=True)
        else:
            marker = "✓" if status == "PASS" else "✗"
            print(
                f"│ {event['step']:02d} {marker} {kind} [{status}] "
                f"{name} — {detail}",
                flush=True,
            )

    def action(self, tc: str, name: str, detail: str = "") -> None:
        self.event(tc, "ACTION", name, "RUNNING", detail)

    def check(self, tc: str, name: str, passed: bool, detail: str) -> None:
        status = "PASS" if passed else "FAIL"
        self.checks.append(
            {"tc": tc, "name": name, "status": status, "detail": detail}
        )
        self.event(tc, "CHECK", name, status, detail)

    def error(self, tc: str, name: str, detail: str) -> None:
        self.checks.append(
            {"tc": tc, "name": name, "status": "ERROR", "detail": detail}
        )
        self.event(tc, "ERROR", name, "ERROR", detail)

    def case_status(self, tc: str) -> str:
        statuses = [item["status"] for item in self.checks if item["tc"] == tc]
        if not statuses or "ERROR" in statuses:
            return "ERROR"
        return "FAIL" if "FAIL" in statuses else "PASS"

    def finish(self, tc: str) -> None:
        print(
            f"╰─ RESULT [{self.case_status(tc)}] {tc} {self.titles[tc]}",
            flush=True,
        )

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(
            item["status"] == "PASS" for item in self.checks
        )

    def save(self, root: Path, metadata: dict[str, Any]) -> None:
        summary = {
            "metadata": metadata,
            "summary": {
                "passed": self.passed,
                "pass": sum(item["status"] == "PASS" for item in self.checks),
                "fail": sum(item["status"] == "FAIL" for item in self.checks),
                "error": sum(item["status"] == "ERROR" for item in self.checks),
            },
            "checks": self.checks,
            "flow": self.flow,
        }
        (root / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        lines = [
            "# map_asset_manager 自动测试结果", "",
            f"- 执行时间：{metadata['executed_at']}",
            f"- 隔离命名空间：{metadata['namespace']}",
            f"- 总结：{'PASS' if self.passed else 'FAIL'}", "",
            "| TC | 检查项 | 结果 | 证据 |",
            "|---|---|---|---|",
        ]
        for item in self.checks:
            detail = item["detail"].replace("|", "\\|").replace("\n", "<br>")
            lines.append(
                f"| {item['tc']} | {item['name']} | "
                f"{item['status']} | {detail} |"
            )
        (root / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        flow_lines = [
            "# map_asset_manager 测试执行流程", "",
            f"- 执行时间：{metadata['executed_at']}",
            f"- 隔离命名空间：{metadata['namespace']}",
        ]
        for tc in self.order:
            flow_lines.extend([
                "", f"## {tc} {self.titles[tc]}", "",
                f"**最终结果：{self.case_status(tc)}**", "",
                "| 步骤 | 类型 | 动作 / 检查 | 结果 | 详细证据 |",
                "|---:|---|---|---|---|",
            ])
            for event in self.flow:
                if event["tc"] != tc:
                    continue
                detail = event["detail"].replace("|", "\\|").replace("\n", "<br>")
                flow_lines.append(
                    f"| {event['step']} | {event['kind']} | {event['name']} | "
                    f"{event['status']} | {detail} |"
                )
        (root / "test_flow.md").write_text(
            "\n".join(flow_lines) + "\n", encoding="utf-8"
        )


def execute(
    report: Report, tc: str, title: str, callback: Callable[[], None]
) -> None:
    report.start(tc, title)
    try:
        callback()
    except Exception as exc:
        report.error(tc, "执行异常", repr(exc))
    report.finish(tc)


class RosHarness:
    def __init__(self, timeout: float) -> None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        self.timeout = timeout
        rclpy.init(args=None)
        self.node = rclpy.create_node("map_asset_manager_self_test_runner")
        self.latched_qos = QoSProfile(depth=1)
        self.latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self.latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()

    def call(self, service_type: Any, service: str, request: Any) -> Any:
        client = self.node.create_client(service_type, service)
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

    def wait_message(
        self, message_type: Any, topic: str, predicate: Callable[[Any], bool]
    ) -> Any:
        received: list[Any] = []

        def callback(message: Any) -> None:
            if predicate(message):
                received.append(message)

        sub = self.node.create_subscription(
            message_type, topic, callback, self.latched_qos
        )
        deadline = time.monotonic() + self.timeout
        try:
            while not received and time.monotonic() < deadline:
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        if not received:
            raise TimeoutError(f"no matching message on {topic}")
        return received[-1]


class AssetRunner:
    def __init__(
        self, harness: RosHarness, report: Report, maps_root: Path, fixture: Path
    ) -> None:
        from geometry_msgs.msg import Point, Pose2D
        from nav_interfaces.msg import MapState
        from nav_interfaces.srv import (
            SetArea, SetNavPoint, SetRoom, SetRoute, SetRouteEdge,
            SetRouteJunction, GetAssets, GetAllNavPoints, GetNavPoint,
            GetRoute, GetRouteGraph, SaveAssets,
        )
        from std_msgs.msg import String

        self.harness = harness
        self.report = report
        self.maps_root = maps_root
        self.fixture = fixture
        self.prefix = f"{NAMESPACE}/map_asset_manager"
        self.state_topic = f"{NAMESPACE}/map_manager/map_state"
        self.status_topic = f"{self.prefix}/status"
        self.epoch = 0
        self.Pose2D = Pose2D
        self.Point = Point
        self.MapState = MapState
        self.String = String
        service_types = (
            SetArea, SetNavPoint, SetRoom, SetRoute, SetRouteEdge,
            SetRouteJunction, GetAssets, GetAllNavPoints, GetNavPoint,
            GetRoute, GetRouteGraph, SaveAssets,
        )
        for service_type in service_types:
            setattr(self, service_type.__name__, service_type)
        self.publisher = harness.node.create_publisher(
            MapState, self.state_topic, harness.latched_qos
        )

    @staticmethod
    def close(actual: float, expected: float) -> bool:
        return abs(float(actual) - float(expected)) <= 1e-5

    def assets_req(self, kinds: int) -> Any:
        """GetAssets 请求；kinds 位掩码：1 points / 2 areas / 4 rooms / 8 routes / 16 junctions / 32 edges，0 = 全部"""
        req = self.GetAssets.Request()
        req.kinds = kinds
        return req

    @staticmethod
    def find_room(resp: Any, name: str) -> Any:
        return next((room for room in resp.rooms if room.name == name), None)

    def service(self, name: str) -> str:
        return f"{self.prefix}/{name}"

    def pose(self, x: float, y: float, theta: float = 0.0) -> Any:
        result = self.Pose2D()
        result.x, result.y, result.theta = x, y, theta
        return result

    def point(self, x: float, y: float) -> Any:
        result = self.Point()
        result.x, result.y = x, y
        return result

    @staticmethod
    def status_fields(value: str) -> dict[str, str]:
        return {
            key: item
            for token in value.split()
            if "=" in token
            for key, item in (token.split("=", 1),)
        }

    def prepare(self, tc: str) -> None:
        target = self.maps_root / MAP_NAME / "assets/map_assets.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.fixture, target)
        self.epoch += 1
        state = self.MapState()
        state.map_folder = str(self.maps_root)
        state.map_name = MAP_NAME
        state.epoch = self.epoch
        self.report.action(
            tc, "恢复标准资产并发布 MapState",
            f"map={MAP_NAME}, epoch={self.epoch}, topic={self.state_topic}",
        )
        deadline = time.monotonic() + self.harness.timeout
        while (
            self.publisher.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            self.harness.rclpy.spin_once(self.harness.node, timeout_sec=0.1)
        if self.publisher.get_subscription_count() < 1:
            raise TimeoutError("isolated node did not subscribe to MapState")
        self.publisher.publish(state)

        def ready(message: Any) -> bool:
            fields = self.status_fields(message.data)
            return (
                fields.get("map") == MAP_NAME
                and fields.get("points") == "3"
                and fields.get("routes") == "1"
                and fields.get("areas") == "1"
                and fields.get("rooms") == "1"
            )

        status = self.harness.wait_message(self.String, self.status_topic, ready)
        self.report.check(tc, "标准资产装载完成", True, status.data)

    def tc01(self) -> None:
        tc = "TC-01"
        self.prepare(tc)
        all_result = self.harness.call(
            self.GetAllNavPoints, self.service("get_all_nav_points"),
            self.GetAllNavPoints.Request(),
        )
        names = {item.name for item in all_result.points}
        get_request = self.GetNavPoint.Request()
        get_request.name = "AUTO_ASSET_POINT_A"
        one = self.harness.call(
            self.GetNavPoint, self.service("get_point"), get_request
        )
        self.report.check(
            tc, "导航点全量与单点查询",
            (
                all_result.success
                and names == {
                    "AUTO_ASSET_POINT_A", "AUTO_ASSET_POINT_B",
                    "AUTO_ASSET_POINT_C",
                }
                and one.success
                and one.point.type == "STATION"
                and self.close(one.point.x, -1.0)
                and self.close(one.point.theta, 0.1)
            ),
            f"names={sorted(names)}, point=({one.point.name},{one.point.type})",
        )
        add = self.SetNavPoint.Request()
        add.point.name, add.point.type = "AUTO_TMP_POINT", "PARKING"
        add.point.x, add.point.y = 10.0, 11.0
        added = self.harness.call(
            self.SetNavPoint, self.service("set_point"), add
        )
        update = self.SetNavPoint.Request()
        update.mode = 2  # MODE_UPDATE_ONLY
        update.point.name, update.point.type = "AUTO_TMP_POINT", "DOORWAIT"
        update.point.x, update.point.y = 12.0, 13.0
        updated = self.harness.call(
            self.SetNavPoint, self.service("set_point"), update
        )
        query = self.GetNavPoint.Request()
        query.name = "AUTO_TMP_POINT"
        current = self.harness.call(
            self.GetNavPoint, self.service("get_point"), query
        )
        remove = self.SetNavPoint.Request()
        remove.mode = 3  # MODE_REMOVE
        remove.point.name = "AUTO_TMP_POINT"
        removed = self.harness.call(
            self.SetNavPoint, self.service("set_point"), remove
        )
        absent = self.harness.call(
            self.GetNavPoint, self.service("get_point"), query
        )
        self.report.check(
            tc, "导航点新增/更新/删除闭环",
            (
                added.success and updated.success and current.success
                and current.point.type == "DOORWAIT"
                and self.close(current.point.x, 12.0)
                and removed.success and not absent.success
            ),
            f"add={added.success}, update={updated.success}, "
            f"remove={removed.success}, query_after_remove={absent.success}",
        )

    def tc02(self) -> None:
        tc = "TC-02"
        self.prepare(tc)
        all_result = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(8),
        )
        route_request = self.GetRoute.Request()
        route_request.name = "AUTO_ASSET_ROUTE"
        route = self.harness.call(
            self.GetRoute, self.service("get_route"), route_request
        )
        self.report.check(
            tc, "路线全量与单条查询",
            (
                all_result.success and len(all_result.routes) == 1
                and route.success
                and list(route.point_names) == [
                    "AUTO_ASSET_POINT_A", "AUTO_ASSET_POINT_B",
                    "AUTO_ASSET_POINT_C",
                ]
            ),
            f"routes={len(all_result.routes)}, points={list(route.point_names)}",
        )
        add = self.SetRoute.Request()
        add.mode = 1  # MODE_ADD_ONLY
        add.route.name = "AUTO_TMP_ROUTE"
        add.route.point_names = ["AUTO_ASSET_POINT_A", "AUTO_ASSET_POINT_B"]
        added = self.harness.call(
            self.SetRoute, self.service("set_route"), add
        )
        temp = self.GetRoute.Request()
        temp.name = add.route.name
        found = self.harness.call(
            self.GetRoute, self.service("get_route"), temp
        )
        remove = self.SetRoute.Request()
        remove.mode = 3  # MODE_REMOVE
        remove.route.name = add.route.name
        removed = self.harness.call(
            self.SetRoute, self.service("set_route"), remove
        )
        absent = self.harness.call(
            self.GetRoute, self.service("get_route"), temp
        )
        self.report.check(
            tc, "路线新增/查询/删除闭环",
            (
                added.success and found.success
                and list(found.point_names) == list(add.point_names)
                and removed.success and not absent.success
            ),
            f"add={added.success}, get={found.success}, "
            f"remove={removed.success}, query_after_remove={absent.success}",
        )

    def tc03(self) -> None:
        tc = "TC-03"
        self.prepare(tc)
        baseline = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(2),
        )
        area = baseline.areas[0] if baseline.areas else None
        self.report.check(
            tc, "面积资产全量查询",
            (
                baseline.success and area is not None
                and area.name == "AUTO_ASSET_AREA" and area.type == 3
                and self.close(area.max_speed, 0.35)
                and len(area.polygon) == 4
            ),
            f"count={len(baseline.areas)}, "
            f"name={getattr(area, 'name', None)}",
        )
        add = self.SetArea.Request()
        add.area_info.name = "AUTO_TMP_AREA"
        add.area_info.type = 3
        add.area_info.polygon = [
            self.pose(20.0, 20.0), self.pose(22.0, 20.0),
            self.pose(22.0, 22.0), self.pose(20.0, 22.0),
        ]
        added = self.harness.call(
            self.SetArea, self.service("set_area"), add
        )
        update = self.SetArea.Request()
        update.mode = 2  # MODE_UPDATE_ONLY
        update.area_info.name = add.area_info.name
        update.area_info.type = 5
        update.area_info.stop_distance = 0.45
        update.area_info.polygon = list(add.area_info.polygon)
        updated = self.harness.call(
            self.SetArea, self.service("set_area"), update
        )
        current_all = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(2),
        )
        current = next(
            (item for item in current_all.areas
             if item.name == add.area_info.name), None
        )
        remove = self.SetArea.Request()
        remove.mode = 3  # MODE_REMOVE
        remove.area_info.name = add.area_info.name
        removed = self.harness.call(
            self.SetArea, self.service("set_area"), remove
        )
        final_all = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(2),
        )
        final_names = {item.name for item in final_all.areas}
        self.report.check(
            tc, "面积资产新增/更新/删除闭环",
            (
                added.success and updated.success and current is not None
                and current.type == 5
                and self.close(current.stop_distance, 0.45)
                and removed.success and add.area_info.name not in final_names
            ),
            f"add={added.success}, update={updated.success}, "
            f"remove={removed.success}, final={sorted(final_names)}",
        )

    def tc04(self) -> None:
        tc = "TC-04"
        self.prepare(tc)
        baseline = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(4)
        )
        by_name = self.find_room(baseline, "AUTO_ASSET_ROOM")
        self.report.check(
            tc, "房间全量查询（get_assets kinds=ROOMS）+ 本地按名过滤",
            (
                baseline.success and len(baseline.rooms) == 1
                and by_name is not None and by_name.name == "AUTO_ASSET_ROOM"
                and len(by_name.polygon) >= 3
            ),
            f"count={len(baseline.rooms)}, by_name={by_name is not None}",
        )
        add = self.SetRoom.Request()
        add.room.name = "AUTO_TMP_ROOM"
        add.room.source = "manual"
        add.room.type = 5
        add.room.polygon = [
            self.pose(20.0, 20.0), self.pose(22.0, 20.0),
            self.pose(22.0, 22.0), self.pose(20.0, 22.0),
        ]
        added = self.harness.call(
            self.SetRoom, self.service("set_room"), add
        )
        update = self.SetRoom.Request()
        update.mode = 2  # MODE_UPDATE_ONLY
        update.room.name = add.room.name
        update.room.source = "manual"
        update.room.type = 8
        update.room.polygon = list(add.room.polygon)
        updated = self.harness.call(
            self.SetRoom, self.service("set_room"), update
        )
        current_all = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(4)
        )
        current = self.find_room(current_all, add.room.name)
        remove = self.SetRoom.Request()
        remove.mode = 3  # MODE_REMOVE
        remove.room.name = add.room.name
        removed = self.harness.call(
            self.SetRoom, self.service("set_room"), remove
        )
        final_all = self.harness.call(
            self.GetAssets, self.service("get_assets"), self.assets_req(4)
        )
        absent = self.find_room(final_all, add.room.name)
        self.report.check(
            tc, "房间新增/更新/按名查询/删除闭环",
            (
                added.success and updated.success and current_all.success
                and current is not None and current.type == 8
                and removed.success and final_all.success and absent is None
            ),
            f"add={added.success}, update={updated.success}, "
            f"found={current is not None}, remove={removed.success}, "
            f"found_after_remove={absent is not None}",
        )

    def tc05(self) -> None:
        tc = "TC-05"
        self.prepare(tc)
        baseline = self.harness.call(
            self.GetRouteGraph, self.service("get_route_graph"),
            self.GetRouteGraph.Request(),
        )
        junctions = {item.id for item in baseline.junctions}
        edges = {item.id for item in baseline.edges}
        nav_points = {item.name for item in baseline.nav_points}
        self.report.check(
            tc, "路网节点/边/引用点全量查询",
            (
                baseline.success
                and junctions == {"AUTO_ASSET_JUNCTION"}
                and edges == {"AUTO_ASSET_EDGE_A", "AUTO_ASSET_EDGE_B"}
                and nav_points == {
                    "AUTO_ASSET_POINT_A", "AUTO_ASSET_POINT_B"
                }
            ),
            f"junctions={sorted(junctions)}, edges={sorted(edges)}, "
            f"nav_points={sorted(nav_points)}",
        )
        add_junction = self.SetRouteJunction.Request()
        add_junction.junction.id = "AUTO_TMP_JUNCTION"
        add_junction.junction.x = 20.0
        add_junction.junction.y = 20.0
        junction_added = self.harness.call(
            self.SetRouteJunction, self.service("set_route_junction"),
            add_junction,
        )
        add_edge = self.SetRouteEdge.Request()
        add_edge.edge.id = "AUTO_TMP_EDGE"
        add_edge.edge.from_ep.id = "AUTO_ASSET_POINT_C"
        add_edge.edge.from_ep.type = "nav"
        add_edge.edge.to_ep.id = "AUTO_TMP_JUNCTION"
        add_edge.edge.to_ep.type = "junction"
        add_edge.edge.edge_type = "BEZIER"
        add_edge.edge.max_speed = 0.2
        add_edge.edge.control_points = [
            self.point(-99.0, -98.0),
            self.point(10.0, 11.0),
            self.point(98.0, 99.0),
        ]
        edge_added = self.harness.call(
            self.SetRouteEdge, self.service("set_route_edge"), add_edge
        )
        update_edge = self.SetRouteEdge.Request()
        update_edge.mode = 2  # MODE_UPDATE_ONLY
        update_edge.edge = add_edge.edge
        update_edge.edge.bidirectional = True
        update_edge.edge.max_speed = 0.15
        update_edge.edge.obs_policy = "DETOUR"
        edge_updated = self.harness.call(
            self.SetRouteEdge, self.service("set_route_edge"),
            update_edge,
        )
        current_graph = self.harness.call(
            self.GetRouteGraph, self.service("get_route_graph"),
            self.GetRouteGraph.Request(),
        )
        current_edge = next(
            (item for item in current_graph.edges if item.id == "AUTO_TMP_EDGE"),
            None,
        )
        snapped_control_points = (
            [(point.x, point.y) for point in current_edge.control_points]
            if current_edge else []
        )
        remove_edge = self.SetRouteEdge.Request()
        remove_edge.mode = 3  # MODE_REMOVE
        remove_edge.edge.id = "AUTO_TMP_EDGE"
        edge_removed = self.harness.call(
            self.SetRouteEdge, self.service("set_route_edge"), remove_edge
        )
        remove_junction = self.SetRouteJunction.Request()
        remove_junction.mode = 3  # MODE_REMOVE
        remove_junction.junction.id = "AUTO_TMP_JUNCTION"
        junction_removed = self.harness.call(
            self.SetRouteJunction, self.service("set_route_junction"),
            remove_junction,
        )
        final = self.harness.call(
            self.GetRouteGraph, self.service("get_route_graph"),
            self.GetRouteGraph.Request(),
        )
        self.report.check(
            tc, "路网节点与边新增/更新/删除闭环",
            (
                junction_added.success and edge_added.success
                and edge_updated.success and current_edge is not None
                and current_edge.bidirectional
                and self.close(current_edge.max_speed, 0.15)
                and current_edge.obs_policy == "DETOUR"
                and len(current_edge.control_points) == 3
                and self.close(current_edge.control_points[0].x, 3.0)
                and self.close(current_edge.control_points[0].y, 0.0)
                and self.close(current_edge.control_points[1].x, 10.0)
                and self.close(current_edge.control_points[1].y, 11.0)
                and self.close(current_edge.control_points[-1].x, 20.0)
                and self.close(current_edge.control_points[-1].y, 20.0)
                and edge_removed.success and junction_removed.success
                and {item.id for item in final.junctions}
                == {"AUTO_ASSET_JUNCTION"}
                and {item.id for item in final.edges}
                == {"AUTO_ASSET_EDGE_A", "AUTO_ASSET_EDGE_B"}
            ),
            f"add_junction={junction_added.success}, add_edge={edge_added.success}, "
            f"update_edge={edge_updated.success}, remove_edge={edge_removed.success}, "
            f"remove_junction={junction_removed.success}, "
            f"snapped_control_points={snapped_control_points}",
        )

    def tc06(self) -> None:
        tc = "TC-06"
        self.prepare(tc)
        assets_file = self.maps_root / MAP_NAME / "assets/map_assets.json"
        backups_dir = assets_file.parent / "backups"
        before = set(backups_dir.glob("map_assets.*.json"))
        request = self.SaveAssets.Request()
        request.map_name = MAP_NAME
        self.report.action(
            tc, "显式保存当前资产并生成 ring 备份",
            f"service={self.service('save_assets')}, map={MAP_NAME}",
        )
        response = self.harness.call(
            self.SaveAssets, self.service("save_assets"), request
        )
        backup_file = backups_dir / (
            f"map_assets.{response.backup_ts}.json"
            if response.backup_ts else "map_assets.INVALID.json"
        )
        after = set(backups_dir.glob("map_assets.*.json"))
        self.report.check(
            tc, "save_assets 响应与备份落盘",
            (
                response.success
                and Path(response.assets_path).resolve() == assets_file.resolve()
                and bool(response.backup_ts)
                and backup_file.is_file()
                and backup_file in after - before
            ),
            f"success={response.success}, assets_path={response.assets_path}, "
            f"backup_ts={response.backup_ts}, backup={backup_file}, "
            f"new_backups={sorted(path.name for path in after - before)}",
        )
        if not assets_file.is_file() or not backup_file.is_file():
            self.report.check(
                tc, "当前资产与备份内容一致",
                False,
                f"assets_exists={assets_file.is_file()}, "
                f"backup_exists={backup_file.is_file()}",
            )
            return
        current_bytes = assets_file.read_bytes()
        backup_bytes = backup_file.read_bytes()
        current_json = json.loads(current_bytes)
        backup_json = json.loads(backup_bytes)
        current_errors = validate_fixture(current_json)
        backup_errors = validate_fixture(backup_json)
        self.report.check(
            tc, "当前资产与备份内容一致",
            (
                current_bytes == backup_bytes
                and current_json == backup_json
                and not current_errors
                and not backup_errors
            ),
            f"byte_equal={current_bytes == backup_bytes}, "
            f"semantic_equal={current_json == backup_json}, "
            f"current_errors={current_errors}, backup_errors={backup_errors}",
        )


def run_local(args: argparse.Namespace, output: Path) -> int:
    report = Report()
    harness: RosHarness | None = None
    process: subprocess.Popen[Any] | None = None
    metadata = {
        "executed_at": datetime.now().astimezone().isoformat(),
        "mode": "isolated-local",
        "namespace": NAMESPACE,
        "fixture": str(args.fixture.resolve()),
        "case_count": 6,
        "covered_data_services": list(DATA_SERVICES),
        "covered_management_services": list(MANAGEMENT_SERVICES),
        "covered_services": list(COVERED_SERVICES),
        "boundary": (
            "25 data services plus explicit save_assets backup; asset rollback "
            "remains covered by map_manager/command_board TC-10; navigation "
            "actions, TF-dependent sample_point and RViz interaction are excluded"
        ),
    }
    try:
        with tempfile.TemporaryDirectory(prefix="map_asset_manager_test_") as tmp:
            maps_root = Path(tmp) / "maps"
            maps_root.mkdir()
            with (output / "map_asset_manager.log").open(
                "w", encoding="utf-8"
            ) as node_log:
                process = subprocess.Popen(
                    [
                        str(node_executable()), "--ros-args",
                        "-r", "__node:=map_asset_manager",
                        "-r", f"__ns:={NAMESPACE}",
                        "-p", "auto_save:=false",
                    ],
                    stdout=node_log,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                harness = RosHarness(args.timeout)
                runner = AssetRunner(harness, report, maps_root, args.fixture)
                cases = (
                    ("TC-01", "导航点资产数据接口全集", runner.tc01),
                    ("TC-02", "路线资产数据接口全集", runner.tc02),
                    ("TC-03", "面积资产数据接口全集", runner.tc03),
                    ("TC-04", "房间资产数据接口全集", runner.tc04),
                    ("TC-05", "路网资产数据接口全集", runner.tc05),
                    ("TC-06", "显式保存与资产 ring 备份", runner.tc06),
                )
                for tc, title, method in cases:
                    execute(report, tc, title, method)
                if process.poll() is not None:
                    report.error(
                        "RUNNER", "隔离节点异常退出",
                        f"returncode={process.returncode}",
                    )
    except Exception as exc:
        report.error("RUNNER", "未处理异常", repr(exc))
    finally:
        if harness is not None:
            harness.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        report.save(output, metadata)
    print(f"result: {output}")
    return 0 if report.passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="map_asset_manager isolated automatic self-test"
    )
    parser.add_argument("mode", choices=("validate", "local"))
    parser.add_argument("--fixture", type=Path, default=fixture_path())
    parser.add_argument("--result-root", type=Path, default=RESULT_ROOT)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    output = args.artifact_dir or args.result_root / f"{args.mode}_{stamp}"
    output.mkdir(parents=True, exist_ok=True)
    try:
        fixture = json.loads(args.fixture.read_text(encoding="utf-8"))
        errors = validate_fixture(fixture)
    except Exception as exc:
        errors = [f"fixture parse failed: {exc!r}"]
    validation = {
        "fixture": str(args.fixture.resolve()),
        "passed": not errors,
        "errors": errors,
        "expected_counts": EXPECTED_COUNTS,
        "case_count": 6,
        "covered_data_services": list(DATA_SERVICES),
        "covered_management_services": list(MANAGEMENT_SERVICES),
        "covered_services": list(COVERED_SERVICES),
    }
    (output / "fixture_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if errors:
        print("\n".join(errors), file=sys.stderr)
        print(f"result: {output}")
        return 1
    if args.mode == "validate":
        print("fixture: PASS")
        print(f"result: {output}")
        return 0
    return run_local(args, output)


if __name__ == "__main__":
    raise SystemExit(main())

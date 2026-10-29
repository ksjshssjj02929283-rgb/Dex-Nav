"""入口分发、录包预设、状态码含义、状态字段提取的单元测试（不需要运行中的节点）。"""
import math

from nav_benchmark_tools import cli, common, nav_cmd, record_cmd, watch_cmd


def test_usage_lists_all_commands(capsys):
    assert cli.main([]) == 0
    out = capsys.readouterr().out
    for k in list(cli.COMMANDS) + ["res nav", "res map", "res stb", "brake run", "brake analyze", "navlog convert",
                                   "speed profile", "speed analyze"]:
        assert k in out


def test_unknown_command_and_group():
    assert cli.main(["nope"]) == 2
    assert cli.main(["res", "nope"]) == 2
    assert cli.main(["res", "-h"]) == 0


def test_record_family_and_topics():
    assert record_cmd.family_of("G-ROUTE-03") == "G-ROUTE"
    assert record_cmd.family_of("s-tray-01") == "S-TRAY"
    assert record_cmd.family_of("G-STOP") == "G-STOP"
    topics, hidden, fam = record_cmd.topics_for("G-ROUTE-03", ["/plan", "/x"])
    assert fam == "G-ROUTE" and not hidden
    assert topics[:len(record_cmd.BASE)] == record_cmd.BASE
    assert "/planner_server/RouteGridPlanner/route_plan" in topics and topics[-1] == "/x"
    assert len(topics) == len(set(topics))
    assert record_cmd.topics_for("G-CHG-01")[1] is True
    topics, hidden, fam = record_cmd.topics_for("SPEED")
    assert fam == "SPEED" and hidden
    assert {"/cmd_vel_nav_path", "/navigate_through_graph_points/_action/status", "/feedback/joint"} <= set(topics)


def test_record_list(capsys):
    assert record_cmd.main(["--list"]) == 0
    assert "/cmd_vel_hard_stop" in capsys.readouterr().out


def test_code_text_from_installed_msgs():
    assert common.code_text(0) == "0"
    assert "E_NAV_GOAL_UNREACHABLE" in common.code_text(2201020)
    assert "E_MAP_SAVE_FAILED" in common.code_text(2101072)
    assert common.code_text(1234567) == "1234567"


def test_md_table_escapes_pipe():
    t = common.md_table(["a", "b"], [["x|y", None]])
    assert "x\\|y" in t and t.count("\n") == 2


def test_watch_extractors():
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_interfaces.msg import NavigationState
    m = NavigationState(state=4, error_code=2201123, target_type=1, target_name="P1")
    d = watch_cmd.fields_nav(m, False)
    assert d["state"] == "4 BLOCKED" and d["target"] == "1 TARGET_POINT P1" and "distance_remaining" not in d
    assert "distance_remaining" in watch_cmd.fields_nav(m, True)
    assert "E_NAV_ESTOP" in watch_cmd.value_text("error_code", 2201123)
    da = DiagnosticArray(status=[DiagnosticStatus(name="other")])
    assert watch_cmd.fields_safety(da, False) is None
    da.status.append(DiagnosticStatus(name="/Devices/System/PowerBoard/collector_node: PowerBoard Safety",
                                      values=[KeyValue(key="estop_state", value="true")]))
    assert watch_cmd.fields_safety(da, False)["estop_state"] == "true"


def test_yaw_quat():
    x, y, z, w = nav_cmd.yaw_quat(90)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9 and abs(w - math.cos(math.pi / 4)) < 1e-9

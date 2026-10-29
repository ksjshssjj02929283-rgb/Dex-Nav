"""nbt assets：读地图资产（/map_asset_manager/get_assets），打印点位、路线、路网、区域和失效清单。

  nbt assets                              # 当前地图全部资产
  nbt assets --kinds points,routes        # 只看点位和路线
  nbt assets --map office_5f --json a.json
测试前用它核对点位名、路线组成、路网边属性（模式 / 限速 / 车道宽 / 障碍策略），以及资产是否有失效项。
"""
import argparse
import json
import math
import sys

from nav_benchmark_tools import common

SERVICE = "/map_asset_manager/get_assets"
KINDS = {"points": 1, "areas": 2, "rooms": 4, "routes": 8, "junctions": 16, "edges": 32}
EDGE_MODE = {1: "差速", 2: "全向平移", 3: "全向平移+旋转"}
AREA_TYPE = {1: "虚拟墙", 2: "禁行区", 3: "限速区", 4: "绕行区", 5: "停障区", 6: "停车区", 7: "充电区",
             8: "电梯区", 9: "颠簸区"}


def to_dict(msg):
    from rosidl_runtime_py import message_to_ordereddict
    return message_to_ordereddict(msg)


def ep(e):
    return f"{e.type}:{e.id}" if e.type else e.id


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nbt assets", description="读地图资产")
    ap.add_argument("--kinds", default="", help=f"逗号分隔，可选 {','.join(KINDS)}；默认全部")
    ap.add_argument("--map", default="", help="地图名，默认当前已加载地图")
    ap.add_argument("--json", default="", help="把完整应答存成 JSON")
    ap.add_argument("--timeout", type=float, default=5.0)
    a = ap.parse_args(argv)
    mask = 0
    for k in [x.strip() for x in a.kinds.split(",") if x.strip()]:
        if k not in KINDS:
            ap.error(f"未知类型 {k}，可选 {list(KINDS)}")
        mask |= KINDS[k]

    import rclpy
    from nav_interfaces.srv import GetAssets
    rclpy.init()
    node = rclpy.create_node("nbt_assets")
    try:
        cli = node.create_client(GetAssets, SERVICE)
        if not cli.wait_for_service(timeout_sec=a.timeout):
            print(f"✗ {SERVICE} 不在线")
            return 2
        fut = cli.call_async(GetAssets.Request(kinds=mask, map_name=a.map))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=a.timeout)
        if not fut.done() or fut.result() is None:
            print(f"✗ {a.timeout:.0f} s 内无应答")
            return 2
        r = fut.result()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not r.success:
        print(f"✗ error_code={common.code_text(r.error_code)}  {r.message}")
        return 1
    edited = r.last_edited_at.sec + r.last_edited_at.nanosec * 1e-9
    print(f"# 地图资产：{r.map_name}{'（当前地图）' if r.is_current else ''}\n")
    print(common.md_table(["项", "值"], [
        ["revision", r.revision], ["map_epoch", r.map_epoch],
        ["资产最后编辑", common.now_text(edited) if edited > 0 else "—"],
        ["数量", f"点位 {len(r.points)} / 路线 {len(r.routes)} / 路网节点 {len(r.junctions)} / 路网边 {len(r.edges)}"
                 f" / 区域 {len(r.areas)} / 房间 {len(r.rooms)}"],
        ["失效项", len(r.issues)]]))
    if r.issues:
        print("\n## 失效项\n")
        print(common.md_table(["类型", "对象", "代码", "说明"], [[i.kind, i.key, i.code, i.detail] for i in r.issues]))
    if r.points:
        print("\n## 点位\n")
        print(common.md_table(["名称", "类型", "x", "y", "θ(°)", "容差"],
                              [[p.name, p.type, f"{p.x:.3f}", f"{p.y:.3f}", f"{math.degrees(p.theta):.1f}",
                                f"{p.tolerance:g}"] for p in r.points]))
    if r.routes:
        print("\n## 路线\n")
        print(common.md_table(["名称", "点数", "组成"], [[x.name, len(x.point_names), " → ".join(x.point_names)]
                                                    for x in r.routes]))
    if r.junctions:
        print("\n## 路网节点\n")
        print(common.md_table(["ID", "x", "y", "yaw(°)"], [[j.id, f"{j.x:.3f}", f"{j.y:.3f}", f"{math.degrees(j.yaw):.1f}"]
                                                           for j in r.junctions]))
    if r.edges:
        print("\n## 路网边\n")
        print(common.md_table(
            ["ID", "起点", "终点", "类型", "双向", "模式", "倒车", "限速", "车道宽", "padding", "障碍策略", "控制点"],
            [[e.id, ep(e.from_ep), ep(e.to_ep), e.edge_type, e.bidirectional, EDGE_MODE.get(e.mode, e.mode),
              e.backward, f"{e.max_speed:g}", f"{e.lane_width:g}", f"{e.padding:g}", e.obs_policy,
              len(e.control_points)] for e in r.edges]))
    if r.areas:
        print("\n## 区域\n")
        print(common.md_table(["名称", "类型", "顶点数", "限速", "停障距离", "关联点位"],
                              [[x.name, AREA_TYPE.get(x.type, x.type), len(x.polygon), f"{x.max_speed:g}",
                                f"{x.stop_distance:g}", "，".join(x.related_nav_point_names)] for x in r.areas]))
    if r.rooms:
        print("\n## 房间\n")
        print(common.md_table(["ID", "名称", "语义", "关联点位"],
                              [[x.id, x.name, x.semantic_label, "，".join(x.related_nav_point_names)] for x in r.rooms]))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(to_dict(r), f, ensure_ascii=False, indent=2, default=str)
        print(f"\n已保存 {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

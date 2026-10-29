"""nbt 入口：按子命令分发到各模块的 main(argv)。

  nbt env                 采版本、环境、关键参数
  nbt watch               盯状态变化
  nbt record --case ID    按用例录包
  nbt nav ...             下发导航任务 / 暂停 / 恢复 / 取消 / 限速
  nbt assets              读地图资产
  nbt serve               RViz 测试面板的后端
  nbt res nav|map|stb     资源占用（导航口径 / 建图 / 长跑趋势）
  nbt brake run|analyze   制动距离（实时触发 / 离线分析包）
  nbt pathnav ...         经调度规划器跑路网任务（path_navigation），判停稳、读导航反馈、录划线结果
  nbt navlog convert      路网任务录包 → nav_monitor 格式任务文件
  nbt speed profile|analyze  速度档切换（带门禁）/ 能力摸底速度测试离线分析
每个子命令加 -h 看参数。
"""
import importlib
import sys

COMMANDS = {
    "env": ("env_cmd", "采版本、环境、关键参数（通用约定 3.1）"),
    "watch": ("watch_cmd", "盯建图 / 定位 / 导航 / 系统 / 障碍 / 急停状态变化"),
    "record": ("record_cmd", "按用例录包（默认话题 + 用例族追加）"),
    "nav": ("nav_cmd", "经导航门面下发任务和控制命令"),
    "assets": ("assets_cmd", "读地图资产（点位 / 路线 / 路网 / 区域）"),
    "serve": ("serve_cmd", "RViz 测试面板的后端（常驻在机器人上）"),
    "pathnav": ("pathnav_cmd", "经调度规划器跑路网任务，判停稳、读导航反馈、录划线结果"),
}
GROUPS = {
    "res": {"nav": ("res_nav", "导航进程 CPU / 内存（纯算法、带驱动两个口径）"),
            "map": ("res_map", "建图进程 CPU / 峰值内存 / 地图生成耗时"),
            "stb": ("res_stb", "长跑资源趋势：内存增长、CPU、进程重启")},
    "brake": {"run": ("brake_run", "实时触发制动并测距离"),
              "analyze": ("brake_analyze", "离线分析包里的制动段")},
    "navlog": {"convert": ("navlog_convert", "路网任务录包 → nav_monitor 格式任务文件")},
    "speed": {"profile": ("speed_profile", "速度档：看 / 改（带门禁）/ 恢复"),
              "analyze": ("speed_analyze", "能力摸底速度测试离线分析：关键点位精度、关键路段速度、加减速")},
}


def usage():
    lines = ["用法：nbt <子命令> [参数]，子命令加 -h 看参数", ""]
    for k, (_, h) in COMMANDS.items():
        lines.append(f"  {k:<16}{h}")
    for g, subs in GROUPS.items():
        for k, (_, h) in subs.items():
            lines.append(f"  {g + ' ' + k:<16}{h}")
    return "\n".join(lines)


def run(mod, argv):
    m = importlib.import_module(f"nav_benchmark_tools.{mod}")
    rc = m.main(argv)
    return rc if isinstance(rc, int) else 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(usage())
        return 0
    cmd, rest = argv[0], argv[1:]
    if cmd in COMMANDS:
        return run(COMMANDS[cmd][0], rest)
    if cmd in GROUPS:
        subs = GROUPS[cmd]
        if not rest or rest[0] in ("-h", "--help") or rest[0] not in subs:
            print(f"用法：nbt {cmd} <{'|'.join(subs)}> [参数]")
            for k, (_, h) in subs.items():
                print(f"  {k:<10}{h}")
            return 0 if rest and rest[0] in ("-h", "--help") else 2
        return run(subs[rest[0]][0], rest[1:])
    print(f"未知子命令：{cmd}\n\n{usage()}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())

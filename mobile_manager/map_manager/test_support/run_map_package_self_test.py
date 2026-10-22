#!/usr/bin/env python3
"""地图导入导出（~/map_package 服务 + 应急命令行工具 map_package_tool）服务级自测。

用法（先 source 好 ROS 与本工作区的 install）：
    ros2 run map_manager run_map_package_self_test.py
    ros2 run map_manager run_map_package_self_test.py --keep

做什么：
  1. 建一个临时 HOME，把包内 map_standard 拷成三张图：install 配置的 default_map_name（启动即加载，当前图）、
     一张中文名的图、一张英文名的图
  2. 用临时 HOME 起 map_manager_node 与 map_asset_manager_node（直接起二进制，结束时确认退出）
  3. 服务：导出（中文名）→ 导入（自动改名 / 指定新名 / 拒绝 / 覆盖当前图 / 覆盖非当前图并可从回收站取回）、
     参数非法、文件不在 .imports、地图操作锁被别的进程占用
  4. 命令行工具：与服务互通（工具导出 → 服务导入、服务导出 → 工具导入）、JSON 字段与退出码、
     不加 --force 的覆盖、节点和工具同时导入同名包
  5. 停掉节点后，在没有 ROS 环境变量、C locale 下用命令行工具列图、导出、导入
  6. 统一名字规则在 map_command 上的效果：改名超长 / 含禁用字符被拒且节点不崩；删除「..」、恢复「../x」被拒
  7. 核对本机真实地图目录与 install 配置文件在测试前后未被改动

只改临时目录，不碰 ~/.dexe/data/maps。全部通过退出码 0。
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

E_MAP_OP_INVALID = 2202010
E_MAP_NAME_INVALID = 2202020
E_MAP_IN_USE = 2202040
E_MAP_NAME_EXISTS = 2202050
E_MAP_IO_FAILED = 2202082
E_MAP_IMPORT_FILE_INVALID = 2202140
E_MAP_IMPORT_INCOMPATIBLE = 2202150
RESPONSE_FIELDS = ["success", "error_code", "message", "map_name", "file_path", "size_bytes", "sha256",
                   "trashed_entry"]


def read_yaml_lines(path):
    return Path(path).read_text(encoding="utf-8").splitlines() if Path(path).exists() else []


def yaml_value(lines, key):
    for line in lines:
        if line.startswith(key + ":"):
            return line.split(":", 1)[1].split("#")[0].strip().strip('"').strip("'")
    return None


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot_mtimes(root):
    out = {}
    if root.exists():
        for p in root.rglob("*"):
            try:
                out[str(p)] = p.stat().st_mtime_ns
            except FileNotFoundError:
                pass
    return out


class Report:
    def __init__(self):
        self.rows = []

    def check(self, case, desc, ok, detail=""):
        self.rows.append((case, desc, bool(ok), detail))
        print(f"  {'✅' if ok else '❌'} [{case}] {desc}" + (f"  — {detail}" if detail and not ok else ""))

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


def main():
    parser = argparse.ArgumentParser(description="map package service-level self test")
    parser.add_argument("--domain", type=int, default=88, help="ROS_DOMAIN_ID（默认 88，避免与实机 / 其它测试串扰）")
    parser.add_argument("--keep", action="store_true", help="保留临时目录便于查看")
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from nav_interfaces.srv import GetAssets, GetMaps, GetTrashedMaps, MapManagerCommand, MapPackage

    mm_share = Path(get_package_share_directory("map_manager"))
    mm_prefix = Path(get_package_prefix("map_manager"))
    mm_bin = mm_prefix / "lib/map_manager/map_manager_node"
    tool = mm_prefix / "lib/map_manager/map_package_tool"
    am_bin = Path(get_package_prefix("map_asset_manager")) / "lib/map_asset_manager/map_asset_manager_node"
    config_yaml = mm_share / "config/config.yaml"
    default_map = yaml_value(read_yaml_lines(config_yaml), "default_map_name") or "map_pkg_current"
    src_map = mm_share / "test_data/map_standard"
    if not (src_map / "slam/map.pbstream").exists():
        sys.exit(f"test map missing: {src_map}")
    if not tool.exists():
        sys.exit(f"map_package_tool not built: {tool}")

    real_maps = Path.home() / ".dexe/data/maps"
    real_before = snapshot_mtimes(real_maps)
    config_before = {k: yaml_value(read_yaml_lines(config_yaml), k) for k in ("map_folder", "default_map_name")}

    tmp = Path(tempfile.mkdtemp(prefix="map_package_self_test_"))
    home = tmp / "home"
    maps = home / ".dexe/data/maps"
    cn_name, en_name = "一楼大厅", "pkg_src"
    for name in (default_map, cn_name, en_name):
        shutil.copytree(src_map, maps / name, ignore=shutil.ignore_patterns("backups", ".map.yaml.lock"))
    imports = maps / ".imports"
    print(f"temp HOME: {home}\ncurrent map: {default_map}")

    env = dict(os.environ, HOME=str(home))

    def start_nodes():
        return [subprocess.Popen([str(mm_bin), "--ros-args", "-r", "__node:=map_manager"], env=env,
                                 stdout=open(tmp / "map_manager.log", "a"), stderr=subprocess.STDOUT),
                subprocess.Popen([str(am_bin)], env=env,
                                 stdout=open(tmp / "map_asset_manager.log", "a"), stderr=subprocess.STDOUT)]

    def stop_nodes(procs):
        for p in procs:
            p.send_signal(signal.SIGINT)
        deadline = time.time() + 10
        for p in procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    def run_tool(argv, bare_env=False):
        # bare_env：没有任何 ROS 环境变量、C locale，模拟 map_manager 崩溃后在 SSH 里应急使用
        e = {"PATH": "/usr/bin:/bin", "HOME": str(home), "LC_ALL": "C"} if bare_env else env
        p = subprocess.run([str(tool)] + argv, env=e, capture_output=True, timeout=120)
        out = p.stdout.decode("utf-8", errors="replace").strip()
        try:
            j = json.loads(out.splitlines()[-1]) if out else {}
        except json.JSONDecodeError:
            j = {"_raw": out}
        return p.returncode, j

    def stage(zip_path, name=None):
        imports.mkdir(parents=True, exist_ok=True)
        dst = imports / (name or Path(zip_path).name)
        shutil.copy(zip_path, dst)
        return str(dst)

    rep = Report()
    procs = start_nodes()
    rclpy.init()
    node = rclpy.create_node("map_package_self_test")
    try:
        pkg = node.create_client(MapPackage, "/map_manager/map_package")
        cmd = node.create_client(MapManagerCommand, "/map_manager/map_command")
        gm = node.create_client(GetMaps, "/map_manager/get_maps")
        gt = node.create_client(GetTrashedMaps, "/map_manager/get_trashed_maps")
        ga = node.create_client(GetAssets, "/map_asset_manager/get_assets")
        for cli in (pkg, cmd, gm, gt, ga):
            if not cli.wait_for_service(timeout_sec=20.0):
                sys.exit(f"service not available: {cli.srv_name} (see logs in {tmp})")

        def call(cli, req, timeout=60.0):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
            if fut.result() is None:
                raise RuntimeError(f"call {cli.srv_name} timed out")
            return fut.result()

        def names():
            return [m.name for m in call(gm, GetMaps.Request()).maps]

        def export(name, content=0):
            t0 = time.time()
            r = call(pkg, MapPackage.Request(op=1, map_name=name, content=content))
            return r, time.time() - t0

        def import_(path, new_name="", conflict=0):
            return call(pkg, MapPackage.Request(op=2, file_path=path, new_name=new_name, on_conflict=conflict))

        # ── S1 导出中文名地图 ──
        print("▶ S1 export (service)")
        r, sec = export(cn_name)
        zip_cn = Path(r.file_path)
        rep.check("S1", "export Chinese-named map", r.success and zip_cn.exists(), r.message)
        rep.check("S1", "file under <map_folder>/.exports/ named <map>_<ts>.zip",
                  zip_cn.parent == maps / ".exports" and zip_cn.name.startswith(cn_name + "_"), str(zip_cn))
        rep.check("S1", "sha256 / size_bytes match the file", r.sha256 == sha256(zip_cn) and
                  r.size_bytes == zip_cn.stat().st_size)
        rep.check("S1", "response map_name echoes the source", r.map_name == cn_name)
        print(f"    export took {sec:.2f} s, {r.size_bytes} bytes")
        r_full, _ = export(cn_name, content=1)
        rep.check("S1", "full package export", r_full.success, r_full.message)

        # ── S2 导入：自动改名、指定新名、拒绝 ──
        print("▶ S2 import (service)")
        upload = stage(zip_cn)
        r = import_(upload)
        rep.check("S2", "RENAME on existing name -> <name>_copy1", r.success and r.map_name == cn_name + "_copy1",
                  r.message)
        rep.check("S2", "uploaded file removed after success", not Path(upload).exists())
        rep.check("S2", "imported map listed by get_maps", cn_name + "_copy1" in names())
        a = call(ga, GetAssets.Request(map_name=cn_name + "_copy1"))
        rep.check("S2", "map_asset_manager reads assets of the imported map", a.success, a.message)
        rep.check("S2", "imported files byte-identical", sha256(maps / cn_name / "slam/map.pbstream") ==
                  sha256(maps / (cn_name + "_copy1") / "slam/map.pbstream") and
                  sha256(maps / cn_name / "assets/map_assets.json") ==
                  sha256(maps / (cn_name + "_copy1") / "assets/map_assets.json"))
        r = import_(stage(zip_cn), new_name="二楼走廊", conflict=1)
        rep.check("S2", "import under a new Chinese name (REJECT, no conflict)", r.success and r.map_name == "二楼走廊",
                  r.message)
        upload = stage(zip_cn)
        r = import_(upload, conflict=1)
        rep.check("S2", "REJECT on existing name -> 2202050", r.error_code == E_MAP_NAME_EXISTS, r.message)
        rep.check("S2", "uploaded file kept on failure", Path(upload).exists())

        # ── S3 覆盖 ──
        print("▶ S3 overwrite")
        r_cur, _ = export(default_map)
        r = import_(stage(r_cur.file_path), conflict=2)
        rep.check("S3", "OVERWRITE the currently loaded map -> 2202040", r.error_code == E_MAP_IN_USE, r.message)
        (maps / en_name / "slam/marker.txt").write_text("old")
        r_en, _ = export(cn_name)
        r = import_(stage(r_en.file_path), new_name=en_name, conflict=2)
        rep.check("S3", "OVERWRITE a non-current map -> previous map moved to trash",
                  r.success and r.trashed_entry and not (maps / en_name / "slam/marker.txt").exists(), r.message)
        trashed = [t.entry_name for t in call(gt, GetTrashedMaps.Request()).entries]
        rep.check("S3", "trashed entry visible in get_trashed_maps", r.trashed_entry in trashed, str(trashed))
        rr = call(cmd, MapManagerCommand.Request(op=4, map_name=r.trashed_entry, new_name=en_name + "_old"))
        rep.check("S3", "previous map restorable with OP_RESTORE",
                  rr.success and (maps / (en_name + "_old") / "slam/marker.txt").exists(), rr.message)

        # ── S4 参数与文件 ──
        print("▶ S4 invalid requests")
        rep.check("S4", "op=0 -> 2202010", call(pkg, MapPackage.Request()).error_code == E_MAP_OP_INVALID)
        rep.check("S4", "content=5 -> 2202010",
                  call(pkg, MapPackage.Request(op=1, map_name=cn_name, content=5)).error_code == E_MAP_OP_INVALID)
        rep.check("S4", "on_conflict=7 -> 2202010", import_(stage(zip_cn), conflict=7).error_code == E_MAP_OP_INVALID)
        rep.check("S4", "file outside .imports -> 2202140",
                  import_(str(zip_cn)).error_code == E_MAP_IMPORT_FILE_INVALID)
        bad = imports / "not_a_package.zip"
        bad.write_bytes(b"PK\x03\x04garbage")
        rep.check("S4", "broken zip -> 2202140 / 2202150",
                  import_(str(bad)).error_code in (E_MAP_IMPORT_FILE_INVALID, E_MAP_IMPORT_INCOMPATIBLE))

        # ── S5 地图操作锁被别的进程占用 ──
        print("▶ S5 export while another process holds the map op lock")
        lock_path = maps / cn_name / "slam/.map_op.lock"
        with open(lock_path, "a+") as lf:
            fcntl.flock(lf, fcntl.LOCK_EX)
            r, sec = export(cn_name)
            fcntl.flock(lf, fcntl.LOCK_UN)
        rep.check("S5", "busy -> 2202082 within 5 s", r.error_code == E_MAP_IO_FAILED and sec < 5,
                  f"{r.error_code} in {sec:.1f}s")
        rep.check("S5", "export succeeds after the lock is released", export(cn_name)[0].success)

        # ── S6 命令行工具与服务互通 ──
        print("▶ S6 command line tool with the node running")
        rc, j = run_tool(["export", "--map-folder", str(maps), "--map", cn_name])
        rep.check("S6", "tool export: exit 0, JSON fields same as the service response",
                  rc == 0 and j.get("success") and all(k in j for k in RESPONSE_FIELDS) and j.get("op") == "export",
                  str(j))
        r = import_(stage(j.get("file_path", "")), new_name="工具导出")
        rep.check("S6", "tool export -> service import", r.success, r.message)
        r_svc, _ = export(en_name)
        rc, j = run_tool(["import", "--map-folder", str(maps), "--file", r_svc.file_path, "--name", "服务导出"])
        rep.check("S6", "service export -> tool import (any path, source kept)",
                  rc == 0 and j.get("map_name") == "服务导出" and Path(r_svc.file_path).exists(), str(j))
        rep.check("S6", "map imported by the tool is visible to get_maps", "服务导出" in names())
        rc, j = run_tool(["import", "--map-folder", str(maps), "--file", r_svc.file_path, "--name", "服务导出",
                          "--on-conflict", "reject"])
        rep.check("S6", "tool business failure: exit 1 with error_code", rc == 1 and j.get("error_code") ==
                  E_MAP_NAME_EXISTS, str(j))
        rc, j = run_tool(["import", "--map-folder", str(maps), "--file", r_svc.file_path, "--on-conflict", "overwrite"])
        rep.check("S6", "tool overwrite without --force: exit 2", rc == 2 and not j.get("success"), str(j))
        rc, j = run_tool(["export", "--map-folder", str(maps)])
        rep.check("S6", "tool usage error: exit 2", rc == 2, str(j))

        # ── S7 节点与工具同时导入同名包 ──
        print("▶ S7 concurrent import (service + tool, same package)")
        p1, p2 = stage(zip_cn, "race1.zip"), stage(zip_cn, "race2.zip")
        out = {}
        t = threading.Thread(target=lambda: out.__setitem__("tool", run_tool(
            ["import", "--map-folder", str(maps), "--file", p2, "--name", "并发"])))
        t.start()
        out["svc"] = import_(p1, new_name="并发")
        t.join()
        n_svc, n_tool = out["svc"].map_name, out["tool"][1].get("map_name")
        rep.check("S7", "both imported under distinct names", out["svc"].success and out["tool"][0] == 0 and
                  n_svc != n_tool and {n_svc, n_tool} <= {"并发", "并发_copy1"}, f"{n_svc} / {n_tool}")

        # ── S8 名字规则在 map_command 上 ──
        print("▶ S8 unified name rule on map_command")
        for bad_name, why in (("a:b", "Windows-illegal char"), ("图" * 67, "201 bytes"), ("图" * 100, "300 bytes"),
                              ("尾随　", "trailing full-width space"), ("..", "dot-dot")):
            r = call(cmd, MapManagerCommand.Request(op=3, map_name=en_name + "_old", new_name=bad_name))
            rep.check("S8", f"rename to invalid name ({why}) -> 2202020", r.error_code == E_MAP_NAME_INVALID, r.message)
        rep.check("S8", "map_manager still alive after over-long rename", procs[0].poll() is None and
                  en_name + "_old" in names())
        r = call(cmd, MapManagerCommand.Request(op=3, map_name=en_name + "_old", new_name="三楼 东区"))
        rep.check("S8", "rename to a valid Chinese name", r.success, r.message)
        r = call(cmd, MapManagerCommand.Request(op=2, map_name=".."))
        rep.check("S8", "remove '..' -> 2202020, map folder intact", r.error_code == E_MAP_NAME_INVALID and
                  (maps / cn_name / "slam/map.yaml").exists(), r.message)
        r = call(cmd, MapManagerCommand.Request(op=4, map_name="../" + cn_name, new_name="stolen"))
        rep.check("S8", "restore '../<map>' -> 2202020, map not moved", r.error_code == E_MAP_NAME_INVALID and
                  (maps / cn_name).exists() and not (maps / "stolen").exists(), r.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_nodes(procs)

    # ── S9 节点停掉后的应急使用（无 ROS 环境、C locale）──
    print("▶ S9 emergency use: node stopped, no ROS environment, C locale")
    rep.check("S9", "map_manager stopped", all(p.poll() is not None for p in procs))
    rc, j = run_tool(["list", "--map-folder", str(maps)], bare_env=True)
    listed = [m.get("name") for m in j.get("maps", [])]
    rep.check("S9", "tool list works and shows Chinese names", rc == 0 and cn_name in listed and "三楼 东区" in listed
              and not any(n.startswith(".") for n in listed), str(listed))
    rc, j = run_tool(["export", "--map-folder", str(maps), "--map", cn_name, "--full"], bare_env=True)
    rep.check("S9", "tool export (full) works", rc == 0 and Path(j.get("file_path", "/nonexistent")).exists(), str(j))
    rc2, j2 = run_tool(["import", "--map-folder", str(maps), "--file", j.get("file_path", ""), "--name", "应急导入"],
                       bare_env=True)
    rep.check("S9", "tool import works", rc2 == 0 and (maps / "应急导入" / "slam/map.pbstream").exists(), str(j2))
    leftovers = [p.name for d in (maps / ".exports", maps / ".imports") if d.exists() for p in d.iterdir()
                 if p.name.startswith(".staging_")]
    rep.check("S9", "no staging dirs left", not leftovers, str(leftovers))

    print("▶ isolation")
    rep.check("ISO", "real ~/.dexe/data/maps untouched", snapshot_mtimes(real_maps) == real_before)
    config_after = {k: yaml_value(read_yaml_lines(config_yaml), k) for k in ("map_folder", "default_map_name")}
    rep.check("ISO", "install config values unchanged (map_folder / default_map_name)", config_after == config_before,
              f"{config_before} -> {config_after}")

    total, failed = len(rep.rows), rep.failed
    print(f"\n== {total - len(failed)}/{total} passed ==")
    if args.keep or failed:
        print(f"temp dir kept: {tmp}  (map_manager.log / map_asset_manager.log inside)")
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())

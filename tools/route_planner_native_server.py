#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0

import json
import os
import subprocess
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = 8765
WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PLANNER_BIN = os.path.join(
    WORKSPACE_DIR,
    "install/nav2_route_planner/lib/nav2_route_planner/nav_route_native_planner"
)
FRENET_RPP_BIN = os.path.join(
    WORKSPACE_DIR,
    "install/nav2_frenet_ilqr_controller/lib/nav2_frenet_ilqr_controller/frenet_rpp_step_sim"
)
DOCS_DIR = os.path.join(WORKSPACE_DIR, "docs")

class RoutePlannerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DOCS_DIR, **kwargs)

    def end_headers(self):
        # 允许跨域，方便任意方式调试
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.path = "/route_grid_planner_visualizer.html"
            return super().do_GET()
        elif self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            status = {
                "status": "online",
                "backend": "ROS 2 Humble Native C++",
                "planner_binary": PLANNER_BIN,
                "planner_binary_exists": os.path.exists(PLANNER_BIN),
                "frenet_rpp_binary": FRENET_RPP_BIN,
                "frenet_rpp_binary_exists": os.path.exists(FRENET_RPP_BIN),
                "workspace": WORKSPACE_DIR
            }
            self.wfile.write(json.dumps(status).encode("utf-8"))
            return
        else:
            return super().do_GET()

    def do_POST(self):
        if self.path == "/api/plan":
            content_length = int(self.headers.get("Content-Length", 0))
            post_body = self.rfile.read(content_length)

            try:
                with open("/tmp/last_plan_request.json", "wb") as f_dbg:
                    f_dbg.write(post_body)
            except Exception:
                pass

            if not os.path.exists(PLANNER_BIN):
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                err = {
                    "success": false,
                    "error": f"Planner binary not found at {PLANNER_BIN}. Run colcon build first."
                }
                self.wfile.write(json.dumps(err).encode("utf-8"))
                return

            try:
                # 准备运行环境：需要加载 ROS 2 及工作空间的动态链接库
                env = os.environ.copy()
                install_dir = os.path.join(WORKSPACE_DIR, "install")
                # 遍历 install 下所有包的 lib 路径，确保 nav_interfaces、nav2_msgs 等依赖库均可解析
                extra_libs = []
                if os.path.exists(install_dir):
                    for pkg in os.listdir(install_dir):
                        pkg_lib = os.path.join(install_dir, pkg, "lib")
                        if os.path.isdir(pkg_lib):
                            extra_libs.append(pkg_lib)

                current_ld = env.get("LD_LIBRARY_PATH", "")
                env["LD_LIBRARY_PATH"] = ":".join(extra_libs) + f":{current_ld}:/opt/ros/humble/lib"

                proc = subprocess.Popen(
                    [PLANNER_BIN],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env
                )
                stdout, stderr = proc.communicate(input=post_body, timeout=5.0)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()

                if proc.returncode == 0 and stdout:
                    self.wfile.write(stdout)
                else:
                    err_msg = stderr.decode("utf-8", errors="ignore") if stderr else "Unknown error"
                    res = {
                        "success": False,
                        "error": err_msg,
                        "returncode": proc.returncode
                    }
                    self.wfile.write(json.dumps(res).encode("utf-8"))
            except Exception as ex:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                res = {"success": False, "error": str(ex)}
                self.wfile.write(json.dumps(res).encode("utf-8"))
        elif self.path == "/api/step_simulation":
            content_length = int(self.headers.get("Content-Length", 0))
            post_body = self.rfile.read(content_length)

            try:
                with open("/tmp/last_step_sim_request.json", "wb") as f_dbg:
                    f_dbg.write(post_body)
            except Exception:
                pass

            if not os.path.exists(FRENET_RPP_BIN):
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                err = {
                    "success": False,
                    "error": f"Frenet RPP binary not found at {FRENET_RPP_BIN}. Run colcon build first."
                }
                self.wfile.write(json.dumps(err).encode("utf-8"))
                return

            try:
                env = os.environ.copy()
                install_dir = os.path.join(WORKSPACE_DIR, "install")
                extra_libs = []
                if os.path.exists(install_dir):
                    for pkg in os.listdir(install_dir):
                        pkg_lib = os.path.join(install_dir, pkg, "lib")
                        if os.path.isdir(pkg_lib):
                            extra_libs.append(pkg_lib)

                current_ld = env.get("LD_LIBRARY_PATH", "")
                env["LD_LIBRARY_PATH"] = ":".join(extra_libs) + f":{current_ld}:/opt/ros/humble/lib"

                proc = subprocess.Popen(
                    [FRENET_RPP_BIN],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env
                )
                stdout, stderr = proc.communicate(input=post_body, timeout=2.0)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()

                if proc.returncode == 0 and stdout:
                    self.wfile.write(stdout)
                else:
                    err_msg = stderr.decode("utf-8", errors="ignore") if stderr else "Simulation runner error"
                    res = {
                        "success": False,
                        "error": err_msg,
                        "returncode": proc.returncode
                    }
                    self.wfile.write(json.dumps(res).encode("utf-8"))
            except Exception as ex:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                res = {"success": False, "error": str(ex)}
                self.wfile.write(json.dumps(res).encode("utf-8"))
        elif self.path == "/api/save_graph":
            content_length = int(self.headers.get("Content-Length", 0))
            post_body = self.rfile.read(content_length)
            try:
                data = json.loads(post_body.decode("utf-8"))
                save_path = os.path.join(DOCS_DIR, "custom_route_network.json")
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                res = {
                    "success": True,
                    "message": "路网已成功保存到工程文档目录",
                    "path": save_path,
                    "node_count": len(data.get("nodes", {})),
                    "edge_count": len(data.get("edges", []))
                }
                self.wfile.write(json.dumps(res, ensure_ascii=False).encode("utf-8"))
            except Exception as ex:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                res = {"success": False, "error": str(ex)}
                self.wfile.write(json.dumps(res).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

def main():
    print(f"==================================================")
    print(f"  RouteGridPlanner 原生 C++ 本地联调服务 (方案 A)")
    print(f"==================================================")
    print(f"工作空间目录: {WORKSPACE_DIR}")
    print(f"原生核心程序: {PLANNER_BIN} (存在: {os.path.exists(PLANNER_BIN)})")
    print(f"服务监听端口: http://localhost:{PORT}/")
    print(f"可视化器入口: http://localhost:{PORT}/route_grid_planner_visualizer.html")
    print(f"API 规划接口: http://localhost:{PORT}/api/plan")
    print(f"==================================================")

    server = HTTPServer(("0.0.0.0", PORT), RoutePlannerHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务已平稳停止。")

if __name__ == "__main__":
    main()

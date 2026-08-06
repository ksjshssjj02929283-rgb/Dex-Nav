#!/bin/bash

# 获取脚本所在目录，确保路径正确
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$DIR/../.."

echo "============================================="
echo "1. 正在编译 action_test 验证包..."
echo "============================================="
# 避免 conda 环境下的 Python 干扰 ROS 2 编译
export PATH=$(echo $PATH | tr ":" "\n" | grep -v "miniconda3" | tr "\n" ":" | sed 's/:$//')

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
rm -rf "$WORKSPACE_DIR/build/action_test" "$WORKSPACE_DIR/install/action_test"
colcon build --packages-select action_test --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3

if [ $? -ne 0 ]; then
    echo "编译失败！"
    exit 1
fi

echo "============================================="
echo "2. 正在启动验证节点..."
echo "============================================="
source install/setup.bash

# 清理旧日志
rm -f "$DIR/server.log" "$DIR/client_a.log" "$DIR/client_b.log"

# 启动 Server
ros2 run action_test fake_nav_server > "$DIR/server.log" 2>&1 &
SERVER_PID=$!
echo "Server 已启动 (PID: $SERVER_PID)"
sleep 1

# 启动 Client A (1秒后会自动发送 Goal)
ros2 run action_test client_a > "$DIR/client_a.log" 2>&1 &
CLIENT_A_PID=$!
echo "Client A 已启动 (PID: $CLIENT_A_PID)"

# 启动 Client B (2秒后会自动发送 Goal 抢占 Client A)
ros2 run action_test client_b > "$DIR/client_b.log" 2>&1 &
CLIENT_B_PID=$!
echo "Client B 已启动 (PID: $CLIENT_B_PID)"

echo "正在运行验证，等待 6 秒..."
sleep 6

# 停止节点
kill $SERVER_PID $CLIENT_A_PID $CLIENT_B_PID >/dev/null 2>&1
wait $SERVER_PID $CLIENT_A_PID $CLIENT_B_PID 2>/dev/null

echo "============================================="
echo "3. 验证日志分析结果"
echo "============================================="

echo -e "\n--- [Server 日志] ---"
cat "$DIR/server.log"

echo -e "\n--- [Client A (x=1.0) 日志] ---"
cat "$DIR/client_a.log"

echo -e "\n--- [Client B (x=2.0) 日志] ---"
cat "$DIR/client_b.log"

echo "============================================="
echo "验证完毕！"
echo "============================================="

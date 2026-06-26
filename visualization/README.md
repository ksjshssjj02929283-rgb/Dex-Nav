
# Dexe Mobile Application Visualization

本目录包含 Dexe 移动机器人相关的可视化资源、launch 启动脚本及相关文档。

## 目录结构

- `dexforce_description/launch/`：机器人描述与bringup 启动文件
- `dexhand_joint_converter/`：灵巧手关节转换节点
- `general_joint_converter/`：通用关节转换节点
- `robot_status_monitor/`：系统状态监控节点
- `robot_status_msgs/`:自定义的机器人状态接口文件

## 主要功能

- 一键启动机器人描述、关节转换、状态监控等节点
- 支持仿真与真机两种模式

## 依赖

- ROS 2 (建议 Humble 及以上)
- 已正确安装并编译本工作区相关 package

## 使用方法

1. 克隆本仓库并进入本目录：
	```bash
	git clone <repo_url>
	cd dexe_mobile_application/visualization
	```
2. 编译相关 package：
	```bash
    source  path/to/interface  //source interface install dir
	colcon build --packages-select dexforce_description dexhand_joint_converter general_joint_converter robot_status_monitor robot_status_msgs
	source install/setup.bash
	```
3. 启动全部服务（默认仿真模式）：
	```bash
	ros2 launch dexforce_description dexforce_bringup.launch.py
	```
	启动非仿真模式：
	```bash
	ros2 launch dexforce_description dexforce_bringup.launch.pyuse_simulation:=false
	```

## 常见问题
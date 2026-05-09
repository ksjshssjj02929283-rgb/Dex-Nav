# smacc2_nav_mission

一个最小可运行的 SMACC2 导航任务状态机示例。

## 状态流程

1. `StWaitNav2`：等待 Nav2 节点就绪
2. `StNavigateToPickup`：导航到取货点
3. `StNavigateToDropoff`：导航到放货点
4. `StMissionComplete`：任务结束（成功或中止）

## 依赖

- smacc2
- cl_nav2z
- nav2_bringup

## 编译

```bash
cd /home/fengbro/dexforce/1_navigation/nav_dev_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select smacc2_nav_mission
```

## 运行

```bash
source install/setup.bash
ros2 launch smacc2_nav_mission smacc2_nav_mission.launch.py
```

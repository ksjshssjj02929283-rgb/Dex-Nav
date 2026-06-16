# Project Setup Instructions

## 1. Open a GUI in Galactics
1. Navigate to the development directory:
   ```bash
   cd ~/b1_main_v1/b1_devel
   ```
2. Run the test UI script:
   ```bash
   sh b1_test_ui.sh
   ```
3. Select "Launch Robot" -> "One-click launch"

## 2. VM Workstation Setup
1. Go to the VM workstation (password: `123456`)
2. Open a terminal and run:
   ```bash
   rviz2
   ```
3. Open the file, select config, and choose `nav2_xxx`

## 3. Install Keyboard Libraries
```bash
sudo apt-get install libncurses5-dev libncursesw5-dev
``` 

## 4. Activating ROS Environment and Launching 
1. Launch TurtleBot3 in Gazebo:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   or 
   ros2 launch turtlebot3_gazebo turtlebot3_with_apriltags.launch.py 
   ```

2. Launch TurtleBot3 Navigation2:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py
   ```

## Example 1. run whole tree 
```bash
colcon build --packages-select mobile_bt_navigator
source ./install/setup.bash && ros2 run mobile_bt_navigator b1_loop_tree
```
## Example 2. run streer control detect sample
```bash
source ./install/setup.bash && ros2 run mobile_bt_navigator check_steer_press
```
 in another terminal, publish /status (touch1 or touch) for testing
 ```bash 
ros2 topic pub /status custom_interfaces/msg/Status "{touch1: 1.0, touch2: 0.0}" -r 2
ros2 topic pub /status custom_interfaces/msg/Status "{touch1: 0.0, touch2: 0.0}" -r 2
 ```

## Example 3. auto_docking sample and test
   
   **Launch Docking Tree**

      ```sh 
      source ./install/setup.bash && ros2 run mobile_bt_navigator auto_docking_tree
      source ./install/setup.bash && ros2 run mobile_bt_navigator full_docking_tree
      ```
   kill node
   ```bash 
   ps aux | grep mobile_bt_autodocking_node
   ```

   in another terminal. publish /status: float64 batsoc, float64 batcurrent for testing
   ```bash 
   # pubilsh charging 
   ros2 topic pub  /status custom_interfaces/msg/Status "{batcurrent: 3.0}" -r 2
   # publish low battery
   ros2 topic pub  /status custom_interfaces/msg/Status "{batsoc: 0.1}" -r 2
   # publish full battery
   ros2 topic pub  /status custom_interfaces/msg/Status "{batsoc: 0.91}"  -r 2
   # pubilsh not charging 
   ros2 topic pub  /status custom_interfaces/msg/Status "{batcurrent: 0.0, batsoc: 0.1}" -r 20 
   # isCharging and Full
   ros2 topic pub  /status custom_interfaces/msg/Status "{batcurrent: 3.0, batsoc: 0.91}" -r 20  
   

   ```

# Extra Tips

1. **When not allowed to edit code in host VSCode**

   ```sh
   sudo chown -R susan:susan /home/susan/Projects/b1_humble/
   ```
2. ** Simulation to Real robot**
   change 
   1. maintree.xml undocking direction -1 to 1; 
   2. goal_poses.yaml; 
   3. docking.yaml; 
   4. run_control_robot.cpp: change line 51 from control_robot_simulation.py to: control_robot.py  
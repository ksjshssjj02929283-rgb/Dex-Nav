# Installation

To install the required Python packages, run the following commands:

```sh
python3 -m pip install pupil-apriltags
python3 -m pip install ament_index_python
python3 -m pip install opencv-python
sudo apt-get install ros-humble-control-toolbox ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt-get install ros-humble-apriltag-ros
pip install scipy
pip install transforms3d

sudo apt install ros-humble-rqt ros-humble-rqt-plot ros-humble-rqt-common-plugins
sudo ln -s /opt/ros/humble/lib/rqt_plot/rqt_plot /usr/local/bin/rqt_plot
export PATH=$PATH:/opt/ros/humble/lib/rqt_plot

```

downside numpy, in ARM, x86 no need
```sh
pip install numpy==1.26.0
```

(Install in China, run above with Chinese source)
```sh
python3 -m pip install pupil-apriltags -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -m pip install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple

```

# Configuration

open config file "camera_config.json"

Intrinsic parameters of camera: 

![Example Image](/home/Projects/b1_humble/src/b1_perception/images/IntrinsicParameters.png)

# Open Camera Driver in Buddi, in galactic

```sh
    cd ~/b1_main_v1/b1_devel/b1_sensor/
    source ./install/setup.bash
    ros2 launch orbbec_camera dabai_dcw2.launch.py
    ros2 launch orbbec_camera multi_camera.launch.py
    ros2 launch b1_tf_manager robot_status.launch.py
```

 root@jetson-orin-nx:/home/Projects/b1_main_v1/b1_devel/b1_tool/script# ./2run-nav.sh 

<!-- move linear  -->
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1

<!-- move angular  -->
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1

<!-- undocking command -->
ros2 run b1_perception undocking.py --timeout 1 --direction 1 --speed 0.015
```

# Run apriltags detection with camera topic

```sh
source ./install/setup.bash
ros2 run b1_perception detect_apriltags.py

ros2 run b1_perception control_robot.py 
(it's not allowed run when obstacles in front of the charging station.
BT check obstacles exist or not? ... TODO List ... )
 # Limit the max speed 0.1-0.3 m/s, tolarence.
 
```

# The control law:

The control law for determining the linear and angular velocity to move toward the goal works as follows. Let d be the distance to the goal. Let a be the angle between the robot heading and the goal position, where left is positive. Let b be the angle between the goal direction and the final pose angle, where left is positive. Then the robot linear and angular velocities are calculated like this:

v = Kp * d
w = Ka*a + Kb*b
See Autonomous Mobile Robots, Second Edition by Siegwart et. al., section 3.6.2.4. In this code, when the robot is near enough to the goal, v is set to zero, and w is simply Kb*b.

To ensure convergence toward the goal, Kp and Ka must be positive, Kb must be negative, and Ka must be greater than Kp. To ensure robust convergence, so that the robot never changes direction, Ka - 5/3*Kb - 2/pi*Kp must be greater than zero.

# Control Strategy:

Rotation First: If the y offset is significant compared to x, rotate the robot first to minimize the y error, aligning the robot more directly with the tag.
Move and Adjust: Once the robot is mostly aligned with the AprilTag (i.e., y is minimal), move forward/backward to adjust the x position while making fine adjustments to theta.

# Action List

1. Rotating the robot to align with the direction towards the target. 
2. Moving the robot to the target position. 
3. Adjusting the robot’s orientation to the target yaw.


# Simulation to Real Robot change:

1. camera_config.json; 
2. docking_config.json;
3. running code: from control_robot_simulation.py to control_robot.py
4. view the frame tree: ros2 run tf2_tools view_frames
 

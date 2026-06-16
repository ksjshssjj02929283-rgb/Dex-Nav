# b1_simulation
This is the simulation package of B1 robot with full sensors and dynamic environment.

# Installation
1. `b1_simulation` depends on `navigation2`, so you need to compile and source `navigation2` first;
2. Clone and compile `b1_simulation` package.
```
mkdir -p b1_simulation/src
cd b1_simulation/src
git clone https://github.com/Grabotics-B1/b1_simulation.git
cd ..
colcon build
source install/setup.bash
```
    
# Running
1. We use the ground truth `odom` from `libgazebo_ros_diff_drive.so` in gazebo, so comment `amcl` in `localization_launch.py` of `navigation2` first, then compile and source `navgation2` package again;
2. Configure `~/.bashrc`, add the following lines to the end of `~/.bashrc`
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:{Your path}/b1_simulation/src/b1_simulation 
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/opt/ros/humble/lib
```
3. Launch simulation
```
ros2 launch buddy_nav2 buddy_nav2.launch.py
```
# Swerve Robot Description

## Dependencies

- `dexforce_description` lives in the sibling repo
  `dexe_mobile_application/visualization/dexforce_description` (not in this repo).
  Build and source that workspace before building this package:

  ```bash
  cd ~/dexe_mobile_application/visualization
  colcon build --packages-select dexforce_description
  source ~/dexe_mobile_application/visualization/install/setup.bash

  cd ~/dexe_mobile_nav
  colcon build --packages-select swerve_robot_description swerve_robot_gazebo swerve_controller
  source install/setup.bash
  ```

## Overview

- This package contains the robot urdf/xacro files, meshes and gazebo controller config for a swerve drive robot.

- The simulation controller configuration/parameters can be found in `config/sim_controllers.yaml`. 

- The `description.launch` file parses the robot xacro file and launches `robot_state_publisher` to get `/robot_description` and `/tf`.

    - Args:
        1. `sim:` defaults to `false`. If `true` then loads Gazebo Control and Sensor Plugins.

## Usage

The `description.launch` file is called by the simulator launch file with the appropriate argument.

## URDF Structure

Main xacro file: `urdf/swerve_robot.xacro` imports the following:
- Robot Base: `urdf/include/swerve_robot_base.xacro`
    - Robot chassis and base link: `urdf/include/robot_parts/chassis.xacro`
    - Steering link and joints: `urdf/include/robot_parts/steering.xacro`
    - Wheel link and joints: `urdf/include/robot_parts/wheel.xacro`
- Gazebo ROS Control Plugin: `urdf/include/gazebo_ros2_control.xacro`

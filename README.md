# HORC LAB Robotics Turtlebot3
This branch (pc-dev-final) contains the packages required for general Turtlebot3 operation and Nav2 controller plugin development for the remote PC. Most packages in this branch are finished and intended for local development and general Turtlebot3 use.

---
This project contains the following list of `ROS2` packages:

- **`DynamixelSDK`** (S): Motor controller submodule.
- **`nav2_custom_controller`** (A): C++ controller plugin template for Nav2.
- **`nav2_interface`** (F): Python package providing an interface for operation of Nav2.
- **`maps`** (F): Custom maps used by the robot.
- **`nav2_nonlinear_feedforward_controller`**(F): C++ controller plugin for Nav2.
- **`nav2_pure_pursuit_controller`** (F): C++ controller plugin for Nav2.
- **`performance_plotter`** (F): Python package for plotting position and velocity data.
- **`turtlebot3`** (S): General Turtlebot3 packages.
- **`turtlebot3_msgs`** (S): General Turtlebot3 packages.

Package status legend:
```python
- (A): Actively developed package. Expect changes.
- (N): Not an actively developed package.
- (F): Finished package. Expect minimal changes after requests.
- (S): Submodule. Created by Turtlebot3 team. 
```

## Robot Submodule (`DynamixelSDK`)

This package implements the interface for motor control. Refer to package README for details.

## C++ Nav2 plugin template (`nav2_custom_controller`)

This is a C++ Nav2 plugin package template to be ran as part of the Nav2 stack. Develop custom controller logic in this package for use in Nav2.

## Python Nav2 operation package (`nav2_interface`)

This is a Python ROS2 package that provides an interface for Nav2 operation. Intended to replace Rviz when using Nav2.

## SLAM Maps (`maps`)

Contains custom maps loaded to the robot when using Nav2. New SLAM generated maps go here.

## C++ Nav2 plugin (`nav2_nonlinear_feedforward_controller`)

This is a C++ Nav2 plugin package to be ran as part of the Nav2 stack. Custom controller to work with Nav2 planning and localization.

## C++ Nav2 plugin (`nav2_pure_pursuit_controller`)

This is a C++ Nav2 plugin package to be ran as part of the Nav2 stack. Controller from navigation2 controller plugins git repository made to work with Nav2 planning and localization.

## Python plotter package (`performance_plotter`)

This is a Python ROS2 package that plots robot response. Plots x and y positions vs time, trajectory, linear speed vs time, and angular speed vs time from /odom topic.

## Robot Submodule (`turtlebot3`)

This package implements basic Turtlebot3 packages. Refer to package README for details.

## Robot Submodule (`turtlebot3_msgs`)

This package implements basic Turtlebot3 packages. Refer to package README for details.


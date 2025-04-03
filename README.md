# HORC LAB Swarm Robotics Turtlebot3

---
This project contains the following list of `ROS2` packages:

- **`DynamixelSDK`** (S): Motor controller submodule.
- **`burger_test`** (F): Python package testing basic motion.
- **`maps`** (A): Custom maps used by the robot.
- **`nav2_nonlinear_feedforward_controller`**(A): C++ controller plugin for Nav2.
- **`nav2_pure_pursuit_controller`** (A): C++ controller plugin for Nav2.
- **`pose_plotter`** (A): Python package for plotting position and velocity data.
- **`turtlebot3`** (S): General Turtlebot3 packages.
- **`turtlebot3_control`** (N): Python package for testing controllers in simulation.
- **`turtlebot3_msgs`** (S): General Turtlebot3 packages.
- **`turtlebot3_simulations`** (S): Turtlebot3 packages for gazebo simulations. Intended for remote PC.

Package status legend:
```python
- (A): Actively developed package. Expect changes.
- (N): Not an actively developed package.
- (F): Finished package. Expect minimal changes after requests.
- (S): Submodule. Created by Turtlebot3 team. 
```

## Robot Submodule (`DynamixelSDK`)

This package implements the interface for motor control. Refer to package README for details.

## Motion Test Package (`burger_test`)

This package is a Python node that commands basic movement. Intended to be an introduction to running custom Python nodes on the robot. burger_test.py script can be modified for custom movements.

## SLAM Maps (`maps`)

Contains custom maps loaded to the robot when using Nav2. New SLAM generated maps go here.

## C++ Nav2 plugin (`nav2_nonlinear_feedforward_controller`)

This is a C++ Nav2 plugin package to be ran as part of the Nav2 stack. Custom controller to work with Nav2 planning and localization.

## C++ Nav2 plugin (`nav2_pure_pursuit_controller`)

This is a C++ Nav2 plugin package to be ran as part of the Nav2 stack. Controller from navigation2 controller plugins git repository made to work with Nav2 planning and localization.

## Python plotter package (`pose_plotter`)

This is a Python ROS2 package that plots robot response. Plots x and y positions vs time, trajectory, linear speed vs time, and angular speed vs time from /odom topic.

## Robot Submodule (`turtlebot3`)

This package implements basic Turtlebot3 packages. Refer to package README for details.

## Python simulation controller test package (`turtlebot3_control`)

This is a Python ROS2 package that allows for controller development with /odom localization and direct trajectory path planning. Use to test control logic in Gazebo simulation. feedforward_proportional.py script can be modified to test different control logic.

## Robot Submodule (`turtlebot3_msgs`)

This package implements basic Turtlebot3 packages. Refer to package README for details.

## Robot Submodule (`turtlebot3_simulations`)

This package implements Turtlebot3 packages for Gazebo simulations. Refer to package README for details.

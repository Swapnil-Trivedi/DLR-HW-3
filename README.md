# DLR-HW-3
Deep learning robotics HW-3

## ROS2 Turtle PID Controller for Turtlesim

This ROS2 Python node implements a PID controller that drives the turtle in the [Turtlesim](https://index.ros.org/p/turtlesim/) simulator towards a desired target position \((x, y)\). The controller adjusts the turtle’s linear and angular velocities using PID algorithms to smoothly reach the goal.

---

## Features

- PID control for **linear velocity** (distance to target) and **angular velocity** (heading towards target).
- User inputs target coordinates constrained within the Turtlesim window \([0, 11] \times [0, 11]\).
- Real-time pose subscription and velocity command publishing.
- Data logging during the motion for:
  - Turtle position \((x, y)\) vs time.
  - Control inputs (linear velocity \(v\) and angular velocity \(\omega\)) vs time.
- Automatically stops the turtle when close enough to the target.
- Plots the recorded data when the node is stopped (using Ctrl+C).

---

## Prerequisites

- **ROS2** installed and sourced (tested with Foxy/Galactic/Humble).
- The **turtlesim** package installed

```
ros2 run turtlesim turtlesim_node

python turtle_pid_controller.py
```
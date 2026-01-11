# In Progress

# **box_bot — ROS 2 Mobile Robot Simulation**

## Description

This repository contains the URDF and simulation configuration for **box_bot**, a mobile robot designed for **ROS 2** and **Gazebo Sim**. The robot implements a **differential drive system** with integrated **2D lidar sensing**, allowing it to navigate virtual environments, avoid obstacles, and publish real-time sensor data.

The robot is modeled using a **modular Xacro-based design**, separating physical properties, sensor definitions, and Gazebo plugins to maintain a clean and scalable codebase. The system currently supports **teleoperation via ROS 2 topics** while publishing live laser scan, odometry, and TF data.

---
## Technologies

This project was developed using:

- ROS 2 (Humble / Jazzy)
- Gazebo Sim (Ignition)
- URDF & Xacro (Robot Modeling)
- Python (Launch files and control nodes)

---
## Features

- **Differential Drive System**  
  Fully functional motion using the `gz-sim-diff-drive-system` plugin.

- **Lidar Integration**  
  360-degree GPU-based lidar simulation publishing to the `/scan` topic.

- **Modular Robot Description**  
  Separate Xacro files for inertial macros, lidar configuration, and Gazebo control.

- **Teleoperation Ready**  
  Responds to standard `geometry_msgs/Twist` messages on the `/cmd_vel` topic.

- **Odometry & TF**  
  Real-time tracking of robot position and coordinate transforms.

- **Optimized Physics**  
  Custom friction parameters for the caster wheel to ensure smooth turning.

---
## The Process

1. **Robot Modeling** — Designed the chassis, wheels, and caster using URDF primitives and Xacro macros.

2. **Inertial Calculation** — Implemented standard inertial macros to ensure realistic physics interactions in Gazebo Sim.

3. **Sensor Integration** — Configured a 2D lidar sensor with defined range and resolution parameters.

4. **Gazebo Sim Migration** — Migrated plugins from Gazebo Classic to the modern Gazebo Sim architecture.

---
## What I Learned

- Best practices for structuring **URDF/Xacro** files for modularity and reuse

- Differences between **Gazebo Classic** and **Gazebo Sim** plugin architectures

- How to bridge messages between **GZ Transport** and **ROS 2**

- The importance of **inertial tensors** and **friction coefficients** for stable robot motion

---
## How It Can Be Improved

- **SLAM Integration** — Add `slam_toolbox` for autonomous environment mapping

- **Navigation Stack** — Configure Nav2 for goal-based path planning

- **Visual Enhancements** — Add meshes and textures for improved visualization

- **Autonomous Obstacle Avoidance** — Implement a Python node to process lidar data and steer away from obstacles

---
## How to Run the Project

1. **Launch the Simulation**
   ```bash
   ros2 launch your_package_name sim.launch.py

This project is a work in progress as I continue to explore the capabilities of ROS 2 and Gazebo Sim.

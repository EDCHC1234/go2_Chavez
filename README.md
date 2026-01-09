# Unitree GO2 ROS 2 – SLAM & Path Planning (Dijkstra)

> This project implements a complete SLAM and path planning system for the Unitree GO2 quadruped robot using ROS 2 Humble. The robot performs mapping in a simulated environment and generates collision-free paths in RViz using the Dijkstra algorithm.

## Unitree GO2
<div style="display: flex; gap: 50px;">
  <img src="https://oss-global-cdn.unitree.com/static/c487f93e06954100a44fac4442b94d94_288x238.png" width="250" />
  <img src="imagenes/go2_gazebo.png" width="350" /> 
</div>

> Unitree GO2 is a quadruped robot designed for research and development in locomotion, perception, and autonomous navigation. In this project, the robot is simulated in Gazebo and visualized in RViz.

---

## Project Overview

This project is divided into two main parts:

- **Part A – SLAM Mapping**
  - The robot explores the environment and generates a 2D occupancy grid map.
- **Part B – Path Planning**
  - A custom ROS 2 node computes a path using the **Dijkstra algorithm**.
  - The path is visualized in RViz and updated when new goals are sent.

---

## Algorithms Used

### SLAM
- Uses LiDAR and odometry data.
- Generates a 2D occupancy grid map of the environment.
- The map is saved and reused for navigation.

### Path Planning – Dijkstra
- The map is discretized into a grid.
- Each cell is treated as a graph node.
- Dijkstra finds the shortest path from the robot position to the goal.
- The resulting path is published as `nav_msgs/Path`.

---

## Workspace Setup

### Create workspace
```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src

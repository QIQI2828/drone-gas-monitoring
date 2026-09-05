# 工业园区无人机气体环境智能监测系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11.0-orange)](http://gazebosim.org)

基于 ROS 和 Gazebo 的无人机气体环境智能监测仿真系统，支持多气源高斯烟羽扩散建模、横切面结构化采样与排放强度反演。

## 功能特性

- 工业园区三维场景（Gazebo）
- RotorS Firefly 六旋翼无人机
- 有风条件下多源高斯烟羽模型（含地面反射）
- 横切面结构化采样策略（9 下风距 × 14 横切点 × 3 高度层）
- 加权最小二乘 + L-BFGS-B 源强反演（误差 < 1%）
- RViz 气体云实时可视化

## 环境依赖

- Ubuntu 20.04 / ROS Noetic / Gazebo 11
- RotorS / mav_comm
- Python 3.8+ / NumPy / SciPy

```bash
sudo apt-get install ros-noetic-rotors-simulator
```

## 安装与启动

```bash
cd ~/catkin_ws/src
git clone <repository-url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 一键启动完整仿真
roslaunch gas_source02 simulation.launch
```

## 单独启动各模块

```bash
# 1. 启动 Gazebo 场景与无人机
roslaunch drone_gazebo02 rotors_industrial_park.launch

# 2. 启动浓度服务
rosrun gas_source02 gas_source02_server.py

# 3. 启动传感器节点
rosrun gas_source02 gas_sensor_node.py

# 4. 启动无人机控制器
rosrun gas_source02 drone_controller.py

# 5. 启动主动采样节点
rosrun gas_source02 active_sampling_drone.py

# 6. 启动可视化（RViz 气体云）
rosrun gas_source02 gas_visualizer.py
```

## 项目结构

```
src/
├── drone_gazebo02/          # Gazebo 场景与无人机模型
├── gas_source02/            # 浓度服务、采样控制、反演、可视化
├── mav_comm-master/         # 第三方依赖
└── rotors_simulator/        # 第三方依赖（RotorS）
```


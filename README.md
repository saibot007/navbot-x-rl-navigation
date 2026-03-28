# NavBot-X: Autonomous Robot Navigation using Reinforcement Learning

NavBot-X is a robotics reinforcement learning project focused on autonomous mobile robot navigation in simulation. The system is built on **Ubuntu**, **ROS 2 Jazzy**, and **Gazebo Sim (`ros_gz`)**, with a custom **Gymnasium** environment and **PPO** training pipeline using **Stable-Baselines3**.

The project goal is to train a differential-drive robot to navigate toward a goal while avoiding obstacles using LiDAR-based perception and reinforcement learning.

---

## Project Overview

This project combines:

- robot modeling in ROS 2
- Gazebo Sim world creation
- differential-drive motion control
- LiDAR and odometry integration
- custom Gym-style RL environment design
- PPO-based training for autonomous navigation

The current system already supports full end-to-end interaction between simulation, sensing, action, episodic reset, and RL training.

---

## What Has Been Built

### Simulation and Robot Stack
- Created the **NavBot-X** robot simulation pipeline
- Built robot description package for ROS 2 Jazzy
- Spawned robot successfully in **Gazebo Sim**
- Added obstacle world for navigation testing
- Integrated differential-drive control through `/cmd_vel`
- Added LiDAR sensing through `/scan`
- Added odometry through `/odom`

### RL Pipeline
- Designed a custom **Gymnasium** environment
- Built observation pipeline from:
  - downsampled LiDAR data
  - odom-derived goal distance
  - goal heading information
- Built action pipeline for robot velocity control
- Implemented reward shaping for navigation learning
- Implemented episodic reset support
- Integrated **PPO** using **Stable-Baselines3**
- Verified PPO training loop can run across episodes

### Engineering Progress
- Solved multiple ROS 2 + Gazebo integration issues
- Debugged sensor bridging with `ros_gz_bridge`
- Structured project for iterative robotics/RL development
- Organized code for future internship-ready expansion

---

## Current Status

The **full RL pipeline skeleton is working end-to-end**:

- **Observation:** LiDAR + odom-derived goal information
- **Action:** velocity commands
- **Environment:** Gazebo obstacle world
- **Reset:** episodic reset working
- **Training loop:** PPO runs without hanging

### Current Meaning of This Stage
This does **not** yet mean that NavBot-X has learned a strong final autonomous navigation policy.

It means the project has reached the critical integration milestone where:

- the robot can move
- the robot can sense
- the environment can provide observations
- the RL agent can act
- episodes can restart
- PPO can train on the environment

This is the foundation required before serious reward tuning, longer training, evaluation, and performance optimization.

---

## Tech Stack

- **OS:** Ubuntu
- **Middleware:** ROS 2 Jazzy
- **Simulator:** Gazebo Sim / `ros_gz`
- **Language:** Python
- **Editor:** VS Code
- **RL Environment API:** Gymnasium
- **RL Algorithm:** PPO
- **Training Library:** Stable-Baselines3

---

## Project Structure

```text
navbot-x/
├── rl/
│   ├── env/
│   │   ├── __init__.py
│   │   └── navbot_env.py          # Custom Gymnasium RL environment
│   ├── train/
│   │   ├── __init__.py
│   │   ├── test_env.py            # Environment testing script
│   │   └── train_ppo.py           # PPO training script
│   ├── logs/                      # PPO / tensorboard logs
│   └── models/                    # Saved models
│
├── ros2_ws/
│   ├── src/
│   │   ├── navbot_x_description/  # Robot URDF/Xacro, RViz config, robot model
│   │   ├── navbot_x_bringup/      # Launch files, bridges, robot spawning
│   │   └── navbot_x_sim/          # Gazebo worlds and simulation assets
│   ├── build/
│   ├── install/
│   └── log/
│
├── configs/
├── datasets/
├── logs/
├── media/
├── models/
└── README.md

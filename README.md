# NavBot-X: Autonomous Robot Navigation using Reinforcement Learning

NavBot-X is a robotics reinforcement learning project focused on autonomous mobile robot navigation in simulation. The system is built on **Ubuntu**, **ROS 2 Jazzy**, and **Gazebo Sim (`ros_gz`)**, with a custom **Gymnasium** environment and a **PPO** training pipeline using **Stable-Baselines3**.

The project goal is to train a differential-drive robot to navigate toward a goal using LiDAR and odometry-based observations, while building a reusable robotics RL pipeline that can later scale toward more advanced simulation and Physical AI workflows.

---

## Project Overview

This project combines:

- robot modeling in ROS 2
- Gazebo Sim world integration
- differential-drive motion control
- LiDAR and odometry sensing
- custom Gymnasium RL environment design
- PPO-based reinforcement learning for navigation
- episodic reset and evaluation for multi-episode training

The current system supports full end-to-end interaction between simulation, sensing, control, reset, training, and evaluation.

---

## What Has Been Built

### Simulation and Robot Stack
- Built the **NavBot-X** robot simulation pipeline
- Created a robot description package for **ROS 2 Jazzy**
- Spawned the robot successfully in **Gazebo Sim**
- Integrated differential-drive motion control through `/cmd_vel`
- Integrated LiDAR sensing through `/scan`
- Integrated odometry through `/odom`
- Added a navigation test world with obstacles

### RL Environment
- Designed a custom **Gymnasium** environment for navigation
- Built an observation pipeline using:
  - downsampled LiDAR readings
  - goal distance
  - goal heading
  - robot motion feedback from odometry
- Built a continuous action pipeline for linear and angular velocity control
- Implemented reward shaping for navigation learning
- Implemented episodic reset logic
- Solved the reset/odometry consistency problem by using:
  - Gazebo pose reset
  - relative odometry reference after each reset

### PPO Training Pipeline
- Integrated **PPO** with **Stable-Baselines3**
- Added environment validation with `check_env`
- Added training and evaluation scripts
- Added saved-model support for checkpointed experiments
- Verified multi-episode PPO training works without hanging

---

## Major Engineering Milestone

One of the biggest engineering blockers in this project was episodic reset reliability.

Earlier approaches such as world reset and delete-respawn reset were unstable during long training runs. The current environment uses **Gazebo pose reset** together with a **relative odometry reference update** after every reset. This fixed the false goal-detection issue caused by accumulated odometry and made longer RL training runs feasible.

This was the key infrastructure milestone that allowed the project to move from simulator debugging to actual policy learning.

---

## Current Status

NavBot-X has now moved beyond the initial pipeline-validation stage and achieved a strong learning milestone on the easy randomized navigation task.

### Current working status
- robot spawning works
- `/cmd_vel` control works
- `/scan` works
- `/odom` works
- episodic reset works
- PPO training works
- evaluation works
- the robot can now solve the randomized stage-0 navigation task reliably

### Latest RL milestone
After PPO training on the randomized stage-0 task, evaluation reached:

- **Success rate:** `0.90`
- **Collision rate:** `0.00`
- **Truncation rate:** `0.10`
- **Average reward:** `118.82`
- **Average episode length:** `101.90`

This means the learned policy is no longer only partially successful or dependent on one exact start-goal pair. The robot can now solve the easy randomized task with strong reliability.

### What this means
This result means:

- the robotics RL stack is working end-to-end
- the robot can reliably reach the goal on a randomized easy task
- the project has progressed from basic integration to real policy learning
- NavBot-X is now ready to move into the next curriculum stage

This does **not** yet mean:

- robust obstacle navigation is fully solved
- the policy generalizes to all task settings
- the project is complete
- the system is sim-to-real ready

At this stage, NavBot-X has achieved a **robust randomized stage-0 navigation milestone**.
---

## Current Learning Setup

The current RL environment includes:

- custom continuous-action navigation control
- LiDAR-based obstacle awareness
- goal-distance and heading observations
- smoothed velocity commands
- repeated `/cmd_vel` publishing per RL step
- reward shaping for:
  - progress toward goal
  - heading improvement
  - goal-aligned forward motion
  - obstacle clearance
  - collision penalty
  - success bonus

The current training focus is to stabilize policy performance on the easy task and then gradually expand to randomized starts, harder geometry, and more realistic navigation scenarios.

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
│   ├── eval/
│   │   └── eval_ppo.py            # PPO evaluation script
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
```
---

## Next Steps

The next development stages are:

1. enable curriculum progression from stage 0 to stage 1
2. train on moderately harder randomized navigation conditions
3. evaluate generalization under wider spawn, yaw, and goal variation
4. move toward richer navigation settings and harder world geometry
5. improve experiment tracking, demonstrations, and project presentation
6. later extend the pipeline toward higher-end simulation and Physical AI workflows such as Isaac Sim / Isaac Lab / Omniverse-style RL environments
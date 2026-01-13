# Fanuc CRX-10iAL + Robotiq 2F-85 - MoveIt 2

![Alt Text](image.png)

## Overview
This repository documents the **end-to-end MoveIt 2 bringup** of a **Fanuc CRX-10iAL** collaborative robot with a **Robotiq 2F-85 gripper** using **ROS 2 Humble**.

The goal was to achieve:
- RViz-only simulation (no Gazebo)
- Stable ros2_control integration
- MoveIt planning & execution
- Proper gripper actuation using a single master joint
- Clean bringup architecture

---

## System Architecture

**ROS 2 Humble**
- robot_state_publisher
- ros2_control
- joint_state_broadcaster
- JointTrajectoryController (arm)
- GripperActionController (gripper)
- MoveIt 2 (OMPL / CHOMP)
- RViz Motion Planning Plugin

---

## Robot Description
- URDF generated via Xacro
- 6-DOF Fanuc arm
- Robotiq 2F-85 attached at `tool0`
- Gripper uses **one actuated joint (`finger_joint`)**
- Remaining gripper joints are mimic + passive

---

## Controllers

### Arm Controller
- `joint_trajectory_controller/JointTrajectoryController`
- Position interface
- 6 joints

### Gripper Controller
- `position_controllers/GripperActionController`
- Single joint: `finger_joint`
- Action-based interface (required by MoveIt)

---

## MoveIt Configuration
- Groups:
  - `arm`: base_link → tool0
  - `gripper`: finger_joint
- End effector properly declared
- Collision matrix heavily pruned for Robotiq internals
- CHOMP + OMPL supported

---

## How to Run

```bash
ros2 launch fanuc_bringup bringup.launch.py
```

This launches:
- robot_state_publisher
- ros2_control
- controllers
- move_group
- RViz Motion Planning UI

---

## ▶️ Running Cartesian Linear Motion (Pilz LIN)

This project supports **Cartesian space linear motion** using the **Pilz Industrial Motion Planner (LIN)** via the MoveIt Python API.

---

### Prerequisites

Ensure the following are running correctly:

- Robot description and `ros2_control` controllers  
- MoveIt `move_group`  
- Correct planning parameters (`planning_python_api.yaml`) configured for Pilz LIN  

Make sure your workspace is built and sourced:

```bash
cd ~/dexsent/dex_ws
colcon build
source install/setup.bash
```

---

### Step 1: Bring up the robot and MoveIt

Launch the full bringup (robot + controllers + MoveIt + RViz):

```bash
ros2 launch fanuc_bringup bringup.launch.py
```

Wait until RViz shows:

```
Ready to take commands for planning group arm.
```

---

### Step 2: Run Cartesian LIN motion

In a **new terminal**, source the workspace and launch the Cartesian motion node:

```bash
source ~/dexsent/dex_ws/install/setup.bash
ros2 launch fanuc_scripts lin_move.launch.py
```

---

### What this does

- Plans a **Cartesian straight-line (LIN)** motion in task space  
- Moves the **FANUC CRX-10iAL** end-effector to the target pose  
- Executes the trajectory via `arm_controller`  

## Known Limitations
- No Gazebo physics
- No force/torque feedback
- Gripper collision geometry simplified

---

## Status
✅ Planning  
✅ Execution  
✅ Arm control  
✅ Gripper open/close  

---

## Debugging Log 🐞

This project involved extensive debugging across **ros2_control, MoveIt 2, controllers, and gripper integration**.

All errors, root causes, and fixes are documented here:  
➡️ **[DEBUGGING.md](DEBUGGING.md)**

---

## Credits
Fanuc CRX URDF, Robotiq 2F-85 model, ROS 2, MoveIt 2

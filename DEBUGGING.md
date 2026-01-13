# Debugging Log — Fanuc CRX-10iAL + Robotiq 2F-85

This document captures **every major failure**, the **exact error messages**, and the **root causes** encountered while bringing up the robot.

---

## 1. No State Interfaces Available
**Error**
```
State interface not available
```

**Cause**
- No Gazebo
- No fake hardware configured

**Fix**
- Used ros2_control + JointStateBroadcaster
- RViz-only simulation

---

## 2. Missing Inertials (Gazebo)
**Error**
```
Gazebo requires inertial tags
```

**Fix**
- Dropped Gazebo entirely
- Used RViz + ros2_control only

---

## 3. Gripper Planning Always Fails
**Error**
```
Unable to sample valid states for gripper
```

**Cause**
- Tried planning multiple gripper joints
- Robotiq gripper only exposes ONE real joint

**Fix**
- Use only `finger_joint`
- Mark all other joints passive

---

## 4. Wrong Gripper Controller Type
**Error**
```
Loader for controller 'gripper_controller' not found
```

**Cause**
- Used `forward_command_controller`
- MoveIt expects an action interface

**Fix**
```
position_controllers/GripperActionController
```

---

## 5. Velocity State Interface Missing
**Error**
```
finger_joint/velocity does not exist
```

**Cause**
- GripperActionController expects velocity state

**Fix**
- Switched back to correct controller type
- Ensured correct interface exposure

---

## 6. Collision at Start State
**Error**
```
Start state appears to be in collision
```

**Cause**
- Robotiq self-collisions
- No SRDF collision matrix

**Fix**
- Added extensive disable_collisions in SRDF

---

## 7. Action Exists But No Server
**Symptom**
```
Action clients: 1
Action servers: 0
```

**Cause**
- Controller loaded but not activated

**Fix**
- Correct controller YAML
- Single joint (not array)

---

## Final Outcome
✅ Stable bringup  
✅ Arm planning + execution  
✅ Gripper open/close via MoveIt  
✅ Clean architecture  

---

## Lessons Learned
- Industrial grippers ≠ trajectory controllers
- Mimic joints must be passive in MoveIt
- RViz-only simulation is valid and powerful
- Most MoveIt failures are **semantic**, not kinematic

---

## This log exists so I never debug this again 🙂

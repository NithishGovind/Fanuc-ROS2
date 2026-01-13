#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.logging import get_logger

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def move_robot():
    logger = get_logger("fanuc_moveit_py")

    moveit = MoveItPy(node_name="fanuc_moveit_py")

    arm = moveit.get_planning_component("arm")
    gripper = moveit.get_planning_component("gripper")

    robot_model = moveit.get_robot_model()

    # ---------------- ARM ----------------
    arm_state = RobotState(robot_model)
    arm_state.set_joint_group_positions(
        "arm",
        np.array([0.3, -0.4, 0.6, 0.0, 0.5, 0.0])
    )

    # ---------------- GRIPPER ----------------
    gripper_state = RobotState(robot_model)
    gripper_state.set_joint_group_positions("gripper", np.array([0.8]))

    arm.set_start_state_to_current_state()
    gripper.set_start_state_to_current_state()

    arm.set_goal_state(robot_state=arm_state)
    gripper.set_goal_state(robot_state=gripper_state)

    logger.info("Planning arm...")
    arm_plan = arm.plan()

    logger.info("Planning gripper...")
    gripper_plan = gripper.plan()

    if arm_plan and gripper_plan:
        logger.info("Executing trajectories ✅")

        moveit.execute("arm", arm_plan.trajectory)
        moveit.execute("gripper", gripper_plan.trajectory)

    else:
        logger.error("Planning failed ❌")


def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

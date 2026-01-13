#!/usr/bin/env python3

import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


def cartesian_lin_move():
    logger = get_logger("fanuc_cartesian_lin")

    moveit = MoveItPy(node_name="fanuc_cartesian_lin")
    arm = moveit.get_planning_component("arm")

    arm.set_start_state_to_current_state()

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"

    target_pose.pose.position.x = 0.45
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.35

    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.707
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.707

    arm.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="tool0"
    )

    logger.info("Planning Cartesian LIN (Pilz)")
    plan = arm.plan()

    if plan:
        logger.info("Executing Cartesian LIN")
        moveit.execute("arm", plan.trajectory)
    else:
        logger.error("Cartesian LIN planning failed")


def main():
    rclpy.init()
    cartesian_lin_move()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

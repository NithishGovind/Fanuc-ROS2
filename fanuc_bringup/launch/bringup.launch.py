import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ---------------- Arguments ----------------
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="Use simulation time"
    )
    is_sim = LaunchConfiguration("is_sim")

    # ---------------- Robot Description ----------------
    fanuc_description_pkg = get_package_share_directory("fanuc_description")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                fanuc_description_pkg,
                "urdf",
                "crx10ial.urdf.xacro"
            )
        ]),
        value_type=str
    )

    # ---------------- Robot State Publisher ----------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": is_sim},
        ],
    )

    # ---------------- ros2_control ----------------
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("fanuc_controller"),
                "config",
                "fanuc_controllers.yaml",
            ),
        ],
    )

    # ---------------- Controller Spawners ----------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # ---------------- MoveIt Config ----------------
    moveit_config = (
        MoveItConfigsBuilder("fanuc_crx10ial", package_name="fanuc_moveit")
        .robot_description(file_path=os.path.join(
            fanuc_description_pkg,
            "urdf",
            "crx10ial.urdf.xacro"
        ))
        .robot_description_semantic(file_path="config/fanuc.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # ---------------- Move Group ----------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ---------------- RViz ----------------
    rviz_config = os.path.join(
        get_package_share_directory("fanuc_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ---------------- Launch Order ----------------
    return LaunchDescription([
        is_sim_arg,

        robot_state_publisher_node,
        controller_manager_node,

        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,

        move_group_node,
        rviz_node,
    ])

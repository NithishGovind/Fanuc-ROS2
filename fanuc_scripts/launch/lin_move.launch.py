import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    fanuc_description = get_package_share_directory("fanuc_description")

    moveit_config = (
        MoveItConfigsBuilder("fanuc_crx10ial", package_name="fanuc_moveit")
        .robot_description(
            file_path=os.path.join(
                fanuc_description,
                "urdf",
                "crx10ial.urdf.xacro",
            )
        )
        .robot_description_semantic(file_path="config/fanuc.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    fanuc_moveit_py_node = Node(
        package="fanuc_scripts",
        executable="lin_move.py",
        name="lin_move",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        fanuc_moveit_py_node,
    ])

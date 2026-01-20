#!/usr/bin/env python3
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


pkg_name = "kalman_filter"
pkg = FindPackageShare(package=pkg_name).find(pkg_name)

# Params yaml
yaml_file = "params.yaml"
params_file = Path(pkg) / "config" / yaml_file

with open(params_file) as file:
    try:
        launch_args = yaml.safe_load(file)
    except yaml.YAMLError as exception:
        print(exception)




def generate_launch_description():

    #! Envs
    colorful_logs = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    #! nodes

    # linear_kalman_filter_node
    kalman_pkg = "kalman_filter"
    linear_kalman_filter_node = Node(
        package = kalman_pkg,
        executable = "linear_kalman_filter_node",
        # name = "kalman_filter",
        parameters=[params_file],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        respawn=False,
        emulate_tty=True
    )


    #! rviz
    rviz_config_filename = "/home/ubuntu/state_estimation/src/kalman_filter/config/rviz.rviz"
    rviz_config_filepath = PathJoinSubstitution([
        pkg, "rviz", rviz_config_filename
    ])

    # rviz_node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_filepath],
        output="screen",
        # condition=IfCondition(rviz),
    )


    #! Node execution
    ld = LaunchDescription()

    # envs
    ld.add_action(colorful_logs)

    # nodes
    ld.add_action(linear_kalman_filter_node)

    # debug
    ld.add_action(rviz_node)

    return ld
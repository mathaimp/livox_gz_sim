# Copyright 2026 Mathai Mathew
# SPDX-License-Identifier: MIT

"""
A Livox lidar on a pole in a small room.

Gazebo, robot_state_publisher, the Livox plugins publishing /livox/lidar (and
/livox/imu for the Mid-360), and rviz.

    ros2 launch livox_gz_sim demo.launch.py [model:=hap] [gui:=false] [rviz:=false]
        [xfer_format:=1] [slices:=5] [resolution:=0.4] [downsample:=4]

model: mid360 (default) or hap

xfer_format as livox_ros_driver2: 0 PointCloud2, 1 CustomMsg (rviz cannot show it)
slices, resolution, downsample: as the mid360 xacro macro (urdf/mid360.xacro)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('livox_gz_sim')
    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'demo.urdf.xacro'),
                 ' model:=', LaunchConfiguration('model'),
                 ' xfer_format:=', LaunchConfiguration('xfer_format'),
                 ' slices:=', LaunchConfiguration('slices'),
                 ' resolution:=', LaunchConfiguration('resolution'),
                 ' downsample:=', LaunchConfiguration('downsample')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='mid360'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('xfer_format', default_value='0'),
        DeclareLaunchArgument('slices', default_value='1'),
        DeclareLaunchArgument('resolution', default_value='0.2'),
        DeclareLaunchArgument('downsample', default_value='1'),
        # the server resolves package://livox_gz_sim/meshes/... through this
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': PythonExpression([
                "'-r ' + ('' if '", gui, "' == 'true' else '-s ') + '",
                os.path.join(share, 'worlds', 'demo.sdf'), "'"])}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'livox_demo', '-topic', 'robot_description'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'demo.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

"""A Mid-360 on a pole in a small room: Gazebo, robot_state_publisher, the
point cloud bridged to ROS, and rviz.

    ros2 launch livox_gz_sim demo.launch.py [gui:=false] [rviz:=false]
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
    share = get_package_share_directory("livox_gz_sim")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "demo.urdf.xacro")]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # the server resolves package://livox_gz_sim/meshes/... through this
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(share)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": PythonExpression([
                "'-r ' + ('' if '", gui, "' == 'true' else '-s ') + '",
                os.path.join(share, "worlds", "demo.sdf"), "'"])}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "mid360_demo", "-topic", "robot_description"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            remappings=[("/livox/lidar/points", "/livox/lidar")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "demo.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])

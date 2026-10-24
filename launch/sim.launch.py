"""Gazebo Fortress + diffbot (with IMU) + ros_gz bridge + robot_state_publisher (+ RViz).

  ros2 launch ekf_fusion sim.launch.py world:=floor        # or world:=slippery
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("ekf_fusion")
    robot_desc = ParameterValue(Command(["xacro ", os.path.join(pkg, "urdf", "diffbot.urdf.xacro")]),
                                value_type=str)
    world = PathJoinSubstitution([pkg, "worlds", [LaunchConfiguration("world"), ".sdf"]])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [LaunchConfiguration("gz_extra"), " -r ", world]}.items())

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "diffbot", "-z", "0.08"])

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_desc, "use_sim_time": True}])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=[
                      "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                      "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                      "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                      "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                      "/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                  ])

    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", os.path.join(pkg, "rviz", "fusion.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="floor"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # gz_extra:=-s runs Gazebo headless (server only)
        DeclareLaunchArgument("gz_extra", default_value=""),
        gz, rsp, spawn, bridge, rviz,
    ])

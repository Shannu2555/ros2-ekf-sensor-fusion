"""Wheel odometry + two robot_localization EKFs + route driver + logger. Start sim.launch.py first.

  ros2 launch ekf_fusion fusion.launch.py tag:=floor calibration:=measured out_dir:=$HOME/fusion_logs
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# What the wheel odometry *believes* the robot looks like. The simulated robot is exactly
# r = 0.050 m on both wheels and a 0.300 m track.
CALIBRATION = {
    "exact": {"wheel_radius_left": 0.05, "wheel_radius_right": 0.05, "wheel_separation": 0.30},
    # what you'd realistically get measuring a robot with a tape measure:
    # track width 1 % off, wheel radii 0.1 % apart
    "measured": {"wheel_radius_left": 0.05005, "wheel_radius_right": 0.04995, "wheel_separation": 0.303},
}


def nodes(context):
    cfg = lambda f: os.path.join(get_package_share_directory("ekf_fusion"), "config", f)  # noqa: E731
    calib = CALIBRATION[LaunchConfiguration("calibration").perform(context)]
    logger = Node(package="ekf_fusion", executable="logger_node", name="fusion_logger", output="screen",
                  parameters=[cfg("fusion.yaml"), {"tag": LaunchConfiguration("tag"),
                                                   "out_dir": LaunchConfiguration("out_dir")}])
    return [
        Node(package="ekf_fusion", executable="wheel_odometry_node", name="wheel_odometry",
             output="screen", parameters=[cfg("fusion.yaml"), calib]),
        Node(package="ekf_fusion", executable="imu_relay_node", name="imu_relay",
             output="screen", parameters=[cfg("fusion.yaml")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_fused", output="screen",
             parameters=[cfg("ekf_fused.yaml")], remappings=[("odometry/filtered", "odometry/fused")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_wheel_only", output="screen",
             parameters=[cfg("ekf_wheel_only.yaml")], remappings=[("odometry/filtered", "odometry/wheel_only")]),
        Node(package="ekf_fusion", executable="route_driver_node", name="route_driver",
             output="screen", parameters=[cfg("fusion.yaml"),
                                          {"settle_time": float(LaunchConfiguration("settle_time").perform(context))}]),
        logger,
        # RViz trails only (viz:=true). Ground truth is in "world", the estimates in "odom"; both
        # start at the same spot, so an identity transform between them is exact.
        Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_odom",
             arguments=["--frame-id", "world", "--child-frame-id", "odom"],
             condition=IfCondition(LaunchConfiguration("viz"))),
        Node(package="ekf_fusion", executable="path_viz_node", name="path_viz",
             parameters=[{"use_sim_time": True,
                          "icy_patch": LaunchConfiguration("icy_patch").perform(context) == "true"}],
             condition=IfCondition(LaunchConfiguration("viz"))),
        # the logger exits when the laps are done -> stop everything
        RegisterEventHandler(OnProcessExit(target_action=logger, on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tag", default_value="floor"),
        DeclareLaunchArgument("calibration", default_value="measured", choices=list(CALIBRATION)),
        DeclareLaunchArgument("out_dir", default_value="/tmp/fusion_logs"),
        DeclareLaunchArgument("viz", default_value="false"),
        DeclareLaunchArgument("icy_patch", default_value="false"),      # draw the patch in RViz
        DeclareLaunchArgument("settle_time", default_value="3.0"),      # wait before driving off [s]
        OpaqueFunction(function=nodes),
    ])

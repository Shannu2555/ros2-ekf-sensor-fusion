"""Encoder-based wheel odometry, published the way a typical robot base would publish it.

Reads the wheel angles Gazebo reports on /joint_states, turns them into encoder ticks and
integrates them with the robot geometry it was configured with. No TF here: the EKF owns
odom -> base_link."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ekf_fusion.wheel_odometry import WheelOdometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        p = lambda n, v: self.declare_parameter(n, v).value     # noqa: E731
        self.odo = WheelOdometry(p("wheel_radius_left", 0.05), p("wheel_radius_right", 0.05),
                                 p("wheel_separation", 0.30), p("ticks_per_rev", 2048))
        self.left, self.right = p("left_joint", "left_wheel_joint"), p("right_joint", "right_wheel_joint")
        self.var_v = p("twist_var_vx", 4e-4)
        self.var_w = p("twist_var_wz", 1e-2)
        self.pub = self.create_publisher(Odometry, p("topic", "wheel/odom"), 20)
        self.create_subscription(JointState, "joint_states", self.on_joints, 50)

    def on_joints(self, m):
        try:
            al = m.position[m.name.index(self.left)]
            ar = m.position[m.name.index(self.right)]
        except ValueError:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.odo.update(t, al, ar):
            return
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.odo.x, self.odo.y
        o.pose.pose.orientation.z = math.sin(self.odo.theta / 2)
        o.pose.pose.orientation.w = math.cos(self.odo.theta / 2)
        o.twist.twist.linear.x = self.odo.v
        o.twist.twist.angular.z = self.odo.w
        cov = [0.0] * 36
        cov[0], cov[7], cov[14], cov[21], cov[28], cov[35] = self.var_v, 1e-6, 1e-6, 1e-6, 1e-6, self.var_w
        o.twist.covariance = cov
        o.pose.covariance = [0.0] * 36
        self.pub.publish(o)


def main():
    rclpy.init()
    n = WheelOdometryNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise          # only ignore errors from messages arriving while ROS shuts down
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

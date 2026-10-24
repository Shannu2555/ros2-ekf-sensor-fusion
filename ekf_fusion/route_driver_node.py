"""Drives the laps. It steers with Gazebo's ground-truth pose on purpose: that way every run
drives exactly the same path, and the estimators I'm comparing only listen, they don't steer."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from ekf_fusion.route import PurePursuit, square_laps


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RouteDriver(Node):
    def __init__(self):
        super().__init__("route_driver")
        p = lambda n, v: self.declare_parameter(n, v).value     # noqa: E731
        path = square_laps(p("side", 3.0), p("laps", 5), p("corner_radius", 0.6))
        self.pp = PurePursuit(path, p("lookahead", 0.35), p("speed", 0.3), p("w_max", 1.2))
        self.settle = p("settle_time", 3.0)
        self.pose = None
        self.reported = 0
        self.ready_since = None
        # don't move until every estimator is up, so they all start from the same standstill
        self.waiting_for = {"wheel/odom", "odometry/fused", "odometry/wheel_only", "imu/data"}
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.done_pub = self.create_publisher(Bool, "route/done", 10)
        self.create_subscription(Odometry, "ground_truth", self.on_truth, 20)
        for topic in ("wheel/odom", "odometry/fused", "odometry/wheel_only"):
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.waiting_for.discard(t), 10)
        self.create_subscription(Imu, "imu/data", lambda m: self.waiting_for.discard("imu/data"), 10)
        self.create_timer(0.05, self.step)

    def on_truth(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def step(self):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.waiting_for:
            return
        if self.ready_since is None:
            self.ready_since = now
            self.get_logger().info(f"all estimators up, starting in {self.settle:.0f} s")
        if now - self.ready_since < self.settle:
            return
        v, w = self.pp.command(*self.pose)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)
        pct = int(self.pp.progress() * 10)
        if pct > self.reported:
            self.reported = pct
            self.get_logger().info(f"route {pct * 10}% done")
        if self.pp.done():
            self.done_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    n = RouteDriver()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise          # only ignore errors from messages arriving while ROS shuts down
    finally:
        try:
            n.cmd_pub.publish(Twist())     # stop the robot if we're killed mid-route
        except Exception:
            pass                           # context already gone at launch shutdown: nothing to stop
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Logs ground truth and every pose estimate side by side at 20 Hz (sim time) into one CSV.
Exits when the route driver says the laps are done, which ends the launch."""
import csv
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from ekf_fusion.route_driver_node import yaw_from_quat

SOURCES = {"truth": "ground_truth", "wheel": "wheel/odom", "ekf_wheel": "odometry/wheel_only",
           "ekf_fused": "odometry/fused"}


class Logger(Node):
    def __init__(self):
        super().__init__("fusion_logger")
        out_dir = self.declare_parameter("out_dir", os.path.expanduser("~/fusion_logs")).value
        tag = self.declare_parameter("tag", "floor").value
        self.timeout = self.declare_parameter("timeout", 900.0).value
        os.makedirs(out_dir, exist_ok=True)
        self.fn = os.path.join(out_dir, f"{tag}.csv")
        self.f = open(self.fn, "w", newline="")
        self.w = csv.writer(self.f)
        cols = ["t"]
        for k in SOURCES:
            cols += [f"{k}_stamp", f"{k}_x", f"{k}_y", f"{k}_yaw"]
        self.w.writerow(cols + ["gyro_wz", "wheel_wz"])
        self.latest = {k: None for k in SOURCES}
        self.gyro = float("nan")
        for k, topic in SOURCES.items():
            self.create_subscription(Odometry, topic, lambda m, k=k: self.on_odom(k, m), 20)
        self.create_subscription(Imu, "imu/data", self.on_imu, 50)
        self.create_subscription(Bool, "route/done", self.on_done, 10)
        self.t0 = None
        self.finished = False
        self.create_timer(0.05, self.write_row)
        self.get_logger().info(f"logging to {self.fn}")

    def on_odom(self, k, m):
        p = m.pose.pose
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9     # lets the analysis line sources up exactly
        self.latest[k] = (p.position.x, p.position.y, yaw_from_quat(p.orientation), m.twist.twist.angular.z, stamp)

    def on_imu(self, m):
        self.gyro = m.angular_velocity.z

    def write_row(self):
        if self.finished or any(v is None for v in self.latest.values()):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        row = [f"{t - self.t0:.3f}"]
        for k in SOURCES:
            x, y, yaw, _, stamp = self.latest[k]
            row += [f"{stamp:.3f}", f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}"]
        row += [f"{self.gyro:.5f}", f"{self.latest['wheel'][3]:.5f}"]
        self.w.writerow(row)
        if t - self.t0 > self.timeout:
            self.get_logger().warn("timeout before the route finished")
            self.stop()

    def on_done(self, m):
        if m.data and not self.finished:
            self.get_logger().info(f"route done after {self.get_clock().now().nanoseconds * 1e-9 - self.t0:.1f} s")
            self.stop()

    def stop(self):
        self.finished = True
        self.f.flush()
        raise SystemExit

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = Logger()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

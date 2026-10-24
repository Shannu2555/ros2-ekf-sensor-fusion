"""Gives the Gazebo IMU messages a frame the TF tree knows and an honest covariance.

The bridged message comes with a scoped Gazebo frame name and zero covariance. robot_localization
would read zero covariance as "perfect sensor", so I fill in the noise I configured for the gyro."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__("imu_relay")
        self.frame = self.declare_parameter("frame_id", "imu_link").value
        gyro_std = self.declare_parameter("gyro_stddev", 0.009).value
        acc_std = self.declare_parameter("accel_stddev", 0.02).value
        self.gyro_var, self.acc_var = gyro_std ** 2, acc_std ** 2
        self.pub = self.create_publisher(Imu, "imu/data", 50)
        self.create_subscription(Imu, "imu", self.on_imu, 50)

    def on_imu(self, m):
        m.header.frame_id = self.frame
        m.orientation_covariance = [-1.0] + [0.0] * 8        # "no orientation": not used, not trusted
        g, a = float(self.gyro_var), float(self.acc_var)
        m.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        m.linear_acceleration_covariance = [a, 0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, a]
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ImuRelay()
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

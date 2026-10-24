"""Turns the odometry streams into nav_msgs/Path trails for RViz. Visual only, not used for any numbers."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

TRAILS = {"ground_truth": "viz/path/ground_truth", "wheel/odom": "viz/path/wheel",
          "odometry/fused": "viz/path/fused"}


class PathViz(Node):
    def __init__(self):
        super().__init__("path_viz")
        self.every = self.declare_parameter("keep_every", 5).value
        self.paths, self.pubs, self.count = {}, {}, {}
        for src, dst in TRAILS.items():
            self.paths[src] = Path()
            self.pubs[src] = self.create_publisher(Path, dst, 2)
            self.count[src] = 0
            self.create_subscription(Odometry, src, lambda m, s=src: self.on_odom(s, m), 20)

    def on_odom(self, src, m):
        self.count[src] += 1
        if self.count[src] % self.every:
            return
        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.pose.pose
        path = self.paths[src]
        path.header.frame_id = m.header.frame_id
        path.header.stamp = m.header.stamp
        path.poses.append(ps)
        self.pubs[src].publish(path)


def main():
    rclpy.init()
    n = PathViz()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

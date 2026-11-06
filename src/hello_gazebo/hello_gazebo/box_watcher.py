import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class BoxWatcher(Node):
    """Print the height of the box simulated in Gazebo."""

    def __init__(self):
        super().__init__('box_watcher')
        self.subscription = self.create_subscription(
            Odometry, '/model/box/odometry', self.on_odometry, 10)

    def on_odometry(self, msg):
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Box height: {z:.3f} m')


def main():
    rclpy.init()
    node = BoxWatcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Odometry reporter for Idefix.

Subscribes to '/odom', the standard topic carrying the robot's estimated
position, and prints where the robot thinks it is.

Like the commander, this is IDENTICAL in sim and on the real robot. In Gazebo
the diff-drive plugin publishes /odom; on the real robot, wheel odometry (later
fused with the IMU) publishes the same /odom. The same code reads both.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry       # message type carrying odometry


class OdomReporter(Node):
    def __init__(self):
        super().__init__('odom_reporter')

        # A subscription: type Odometry, topic 'odom', callback self.on_odom,
        # queue size 10. ROS2 calls on_odom for every new message that arrives.
        self.sub = self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.get_logger().info('listening for /odom...')

    def on_odom(self, msg: Odometry):
        # Position is at msg.pose.pose.position. We read x and y.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'position: x={x:.2f} m, y={y:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = OdomReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
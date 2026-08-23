"""Velocity commander for Idefix.

Publishes a constant velocity command on '/cmd_vel', the standard topic a
mobile robot listens to for "how fast to drive forward and how fast to turn".

The key property: this node is IDENTICAL in simulation and on the real robot.
In Gazebo the diff-drive plugin listens to /cmd_vel and moves the model; on the
real robot the base (via micro-ROS) listens to the same /cmd_vel and drives the
motors. One node, both worlds, no changes.
"""

import rclpy                            # the ROS2 Python library
from rclpy.node import Node             # base class for every ROS2 node
from geometry_msgs.msg import Twist     # message type for velocity commands


class VelocityCommander(Node):
    def __init__(self):
        # The node's name, as seen in 'ros2 node list'.
        super().__init__('velocity_commander')

        # Parameters let you change the speeds without editing the code.
        self.declare_parameter('linear_speed', 0.2)    # m/s, forward
        self.declare_parameter('angular_speed', 0.5)   # rad/s, turning
        self.linear = self.get_parameter('linear_speed').value
        self.angular = self.get_parameter('angular_speed').value

        # A publisher: message type Twist, topic 'cmd_vel', queue size 10.
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Run self.tick() ten times a second (every 0.1 s).
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'commanding linear={self.linear} m/s, angular={self.angular} rad/s')

    def tick(self):
        # linear.x is forward speed, angular.z is turn rate. Constant values
        # for both make the robot drive in a circle.
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)               # start ROS2
    node = VelocityCommander()          # create the node
    try:
        rclpy.spin(node)                # keep running until Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())       # all-zero command so the robot stops
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
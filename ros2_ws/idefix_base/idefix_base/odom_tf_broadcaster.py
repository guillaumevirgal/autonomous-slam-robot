"""Odometry-to-TF broadcaster for Idefix.

Subscribes to '/odom' and republishes the pose as an 'odom -> base_link'
transform on '/tf'. The ESP32 firmware intentionally does NOT publish '/tf':
tf2_msgs lives in ros2/geometry2 rather than ros2/common_interfaces, and is
not included in the vendored micro-ROS embedded build. Bringing it in would
add a fragile dependency for no functional gain. Splitting /odom on the MCU
from /tf on the Pi also matches the standard ROS 2 pattern used by
ros2_control's diff_drive_controller: hardware publishes /odom, a broadcaster
owns /tf.

The pose is copied 1:1 from the Odometry message and the header stamp is
preserved verbatim (NOT restamped with this node's clock). The firmware
calls rmw_uros_sync_session() right after the agent handshake, so the
ESP32's clock is aligned with the Pi's ROS time domain and the odom stamp
is meaningful to downstream tf2 consumers (slam_toolbox, Nav2, RViz).

Like odom_reporter, this node is IDENTICAL in sim and on the real robot:
in Gazebo the diff-drive plugin publishes /odom; on the real robot the
ESP32 wheel-odometry publisher does the same. The broadcaster does not
know or care which is upstream.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry               # incoming pose (from firmware or sim)
from geometry_msgs.msg import TransformStamped  # outgoing TF message type
from tf2_ros import TransformBroadcaster        # helper that owns the /tf publisher
class OdomTfBroadcaster(Node):
    # Frame names kept as class attributes for now, matching the hardcoded
    # topic style of velocity_commander and odom_reporter. Can be promoted
    # to declared parameters later if the robot is ever namespaced (e.g.
    # multi-robot sim) without changing the callback logic.
    PARENT_FRAME = 'odom'
    CHILD_FRAME = 'base_link'
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        # TransformBroadcaster wraps a publisher on /tf with the QoS that
        # tf2_ros.Buffer expects on the consumer side. Must be kept alive
        # as self.tf_broadcaster: if it goes out of scope, the underlying
        # publisher is destroyed and no TFs go out (silent failure).
        self.tf_broadcaster = TransformBroadcaster(self)
        # Default QoS (reliable, KEEP_LAST 10) matches the micro-ROS
        # default publisher profile on the ESP32 side. QoS mismatch on
        # /odom is silent (no connection, no error), so keeping both ends
        # at the default is the safe pairing.
        self.sub = self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.get_logger().info(
            f'broadcasting {self.PARENT_FRAME} -> {self.CHILD_FRAME} from /odom'
        )
    def on_odom(self, msg: Odometry):
        # Build the outgoing TF. Everything comes from the Odometry message;
        # we do not consult this node's clock, so the TF timestamp reflects
        # the sensor's time of measurement (ESP32-synced) rather than the
        # time at which the Pi happened to receive the message.
        t = TransformStamped()
        # Preserve the odom stamp verbatim. Because firmware runs
        # rmw_uros_sync_session() at agent handshake, this stamp is in
        # the Pi's ROS time domain.
        t.header.stamp = msg.header.stamp
        # Frame convention: parent = 'odom', child = 'base_link'. The
        # odom -> base_link link is the raw odometry estimate; a SLAM
        # node will later publish 'map' -> 'odom' to correct its drift.
        t.header.frame_id = self.PARENT_FRAME
        t.child_frame_id = self.CHILD_FRAME
        # Position is geometry_msgs/Point on the odom side and
        # geometry_msgs/Vector3 on the TF side. Same field layout but
        # distinct types, so rclpy rejects a whole-struct assignment.
        # Copy field by field.
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        # Orientation is Quaternion on both sides, so direct assignment
        # is fine.
        t.transform.rotation = msg.pose.pose.orientation
        # Hand off to tf2, which serialises and publishes on /tf.
        self.tf_broadcaster.sendTransform(t)
def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
if __name__ == '__main__':
    main()

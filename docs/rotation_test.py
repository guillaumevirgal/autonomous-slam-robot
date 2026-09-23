#!/usr/bin/env python3
"""One-shot full-rotation test: commands a single 360 deg in-place turn at
0.5 rad/s (12.566 s) and measures how much rotation /odom and
/odometry/filtered actually accumulated, by integrating unwrapped yaw deltas
throughout the run rather than comparing start/end orientation (which would
read ~0 deg for a perfect full turn)."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ANGULAR_SPEED = 0.5              # rad/s
DURATION_S = 2 * math.pi / ANGULAR_SPEED   # 12.566 s for one full turn
CMD_RATE_HZ = 20.0


def yaw_from_quat(q):
    # yaw (Z) from quaternion, standard atan2 formula
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RotationTracker:
    def __init__(self):
        self.prev_yaw = None
        self.total = 0.0

    def update(self, yaw):
        if self.prev_yaw is not None:
            delta = yaw - self.prev_yaw
            # unwrap to [-pi, pi] so crossing the +-pi boundary doesn't
            # register as a near-2pi jump
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            self.total += delta
        self.prev_yaw = yaw


class RotationTestNode(Node):
    def __init__(self):
        super().__init__('rotation_test')
        self.odom_tracker = RotationTracker()
        self.filtered_tracker = RotationTracker()
        self.last_wz = None

        # instrumentation: raw (t, wz) samples during the commanded phase,
        # and a count of our own publishes vs how many of them we see
        # looped back (best-effort QoS, same as firmware's cmd_vel sub) --
        # a cheap sanity check on whether DDS itself is dropping publishes
        # before they even reach the micro-ROS agent.
        self.wz_samples = []
        self.recording = False
        self.published_count = 0
        self.looped_back_count = 0

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Odometry, '/odom', self.on_odom, reliable)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_filtered, best_effort)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel_loopback, best_effort)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', best_effort)

    def on_odom(self, msg):
        self.odom_tracker.update(yaw_from_quat(msg.pose.pose.orientation))
        self.last_wz = msg.twist.twist.angular.z
        if self.recording:
            self.wz_samples.append((time.monotonic(), self.last_wz))

    def on_filtered(self, msg):
        self.filtered_tracker.update(yaw_from_quat(msg.pose.pose.orientation))

    def on_cmd_vel_loopback(self, msg):
        self.looped_back_count += 1

    def wait_for_stationary(self, threshold=0.02, hold_s=0.5, timeout_s=6.0):
        """Block until measured /odom angular velocity stays below `threshold`
        rad/s continuously for `hold_s` seconds, or `timeout_s` elapses.
        Returns (settled: bool, initial_wz, waited_s)."""
        t0 = time.monotonic()
        initial_wz = None
        below_since = None
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if initial_wz is None and self.last_wz is not None:
                initial_wz = self.last_wz
            if self.last_wz is not None and abs(self.last_wz) < threshold:
                if below_since is None:
                    below_since = now
                elif now - below_since >= hold_s:
                    return True, initial_wz, now - t0
            else:
                below_since = None
            if now - t0 >= timeout_s:
                return False, initial_wz, now - t0

    def run(self):
        # Don't start the measured window until the robot is confirmed
        # stationary (measured /odom angular velocity near zero, not just a
        # fixed sleep) -- PID integral windup from a prior run's sustained
        # turn can otherwise keep it coasting into this run's window and
        # inflate the result past 100% of commanded.
        settled, initial_wz, waited = self.wait_for_stationary()
        if initial_wz is not None and abs(initial_wz) > 0.02:
            self.get_logger().warn(
                f'robot was still moving at start (wz={initial_wz:.3f} rad/s); '
                f'waited {waited:.2f}s for it to settle (settled={settled})')
        if not settled:
            self.get_logger().error(
                'robot never reached stationary before timeout -- result below is unreliable')

        self.get_logger().info(
            f'commanding {ANGULAR_SPEED} rad/s for {DURATION_S:.3f} s (one full turn)')

        # reset trackers so any motion during the settle-wait above doesn't
        # bleed into this run's measured total
        self.odom_tracker = RotationTracker()
        self.filtered_tracker = RotationTracker()

        twist = Twist()
        twist.angular.z = ANGULAR_SPEED

        period = 1.0 / CMD_RATE_HZ
        t_start = time.monotonic()
        t_end = t_start + DURATION_S
        next_pub = t_start
        self.recording = True
        while True:
            now = time.monotonic()
            if now >= t_end:
                break
            if now >= next_pub:
                self.cmd_pub.publish(twist)
                self.published_count += 1
                next_pub += period
            rclpy.spin_once(self, timeout_sec=0.005)
        self.recording = False

        # stop
        self.cmd_pub.publish(Twist())
        self.get_logger().info('commanded rotation complete, settling...')

        # keep commanding zero while waiting for the robot to actually reach
        # zero angular velocity (not just a fixed sleep) -- any residual
        # spin here is legitimately part of this run's measured total, but
        # we need to know about it either way.
        t_settle_start = time.monotonic()
        settled_after, _, waited_after = self.wait_for_stationary(timeout_s=4.0)
        while time.monotonic() < t_settle_start + waited_after:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        if not settled_after:
            self.get_logger().warn(
                f'robot had not settled to zero wz even after {waited_after:.2f}s post-stop '
                f'(last wz={self.last_wz})')

        commanded_deg = 360.0
        odom_deg = math.degrees(self.odom_tracker.total)
        filtered_deg = math.degrees(self.filtered_tracker.total)
        disagreement_deg = abs(odom_deg - filtered_deg)

        # wz sample analysis: how well did measured angular velocity track
        # the 0.5 rad/s setpoint during the commanded phase?
        wz_vals = [w for _, w in self.wz_samples]
        near_zero = sum(1 for w in wz_vals if abs(w) < 0.05)
        overshoot = sum(1 for w in wz_vals if abs(w) > ANGULAR_SPEED * 1.3)
        mean_wz = sum(wz_vals) / len(wz_vals) if wz_vals else float('nan')
        min_wz = min(wz_vals) if wz_vals else float('nan')
        max_wz = max(wz_vals) if wz_vals else float('nan')
        drop_rate = 100.0 * (self.published_count - self.looped_back_count) / self.published_count \
            if self.published_count else float('nan')

        print('')
        print('=== Rotation test results ===')
        print(f'Commanded:        {commanded_deg:.1f} deg')
        print(f'Odom achieved:    {odom_deg:.1f} deg  ({100.0*odom_deg/commanded_deg:.1f}% of commanded)')
        print(f'Filtered achieved:{filtered_deg:.1f} deg  ({100.0*filtered_deg/commanded_deg:.1f}% of commanded)')
        print(f'Odom/filtered disagreement: {disagreement_deg:.2f} deg')
        print('--- instrumentation ---')
        print(f'/cmd_vel published: {self.published_count}, looped back locally: '
              f'{self.looped_back_count} (local DDS drop: {drop_rate:.1f}%)')
        print(f'/odom wz samples during commanded phase: {len(wz_vals)}')
        print(f'  mean={mean_wz:.3f} rad/s (setpoint {ANGULAR_SPEED}), min={min_wz:.3f}, max={max_wz:.3f}')
        print(f'  near-zero samples (|wz|<0.05, possible dropped/stalled command): {near_zero}')
        print(f'  overshoot samples (|wz|>{ANGULAR_SPEED*1.3:.2f}, possible control instability): {overshoot}')
        print('==============================')


def main():
    rclpy.init()
    node = RotationTestNode()
    try:
        node.run()
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

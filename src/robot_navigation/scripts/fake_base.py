#!/usr/bin/env python3
"""Desk-test stand-in for the robot base.

Integrates /cmd_vel with a differential-drive model and publishes /odom plus
the odom -> base_link TF.  This lets Nav2, AMCL and game_manager be exercised
end to end before the micro-ROS chassis exists; it must never be started on
the real robot, where the firmware owns /odom.

The model is intentionally crude: no wheel slip, no dynamics, instantaneous
velocity response.  It is a test harness, not a simulator.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeBase(Node):
    def __init__(self):
        super().__init__('fake_base')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        # Stamp transforms slightly ahead of the clock.  The lidar stamps
        # scans at publish time, so a transform stamped exactly "now" can be a
        # few milliseconds too old when the scan is processed, which makes
        # tf2 throw "extrapolation into the future" and stops Nav2's
        # collision monitor.  Real base firmware should publish TF at >=50 Hz
        # or use the same offset.
        self.declare_parameter('tf_future_offset', 0.05)

        self._rate = self.get_parameter('publish_rate').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self._tf_offset_ns = int(
            self.get_parameter('tf_future_offset').value * 1e9)

        self._x = self.get_parameter('initial_x').value
        self._y = self.get_parameter('initial_y').value
        self._yaw = self.get_parameter('initial_yaw').value

        self._vx = 0.0
        self._wz = 0.0
        self._last_cmd = self.get_clock().now()
        self._last_time = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f'fake base at ({self._x:.2f}, {self._y:.2f}, {self._yaw:.2f})')

    def _on_cmd_vel(self, msg: Twist):
        self._vx = msg.linear.x
        self._wz = msg.angular.z
        self._last_cmd = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()

        if (now - self._last_cmd).nanoseconds / 1e9 > self._cmd_timeout:
            self._vx = 0.0
            self._wz = 0.0

        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            return

        self._yaw += self._wz * dt
        self._x += self._vx * math.cos(self._yaw) * dt
        self._y += self._vx * math.sin(self._yaw) * dt

        quat_z = math.sin(self._yaw / 2.0)
        quat_w = math.cos(self._yaw / 2.0)

        stamp = now.to_msg()
        stamp.sec += int((stamp.nanosec + self._tf_offset_ns) // 1_000_000_000)
        stamp.nanosec = int((stamp.nanosec + self._tf_offset_ns) % 1_000_000_000)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz
        # Modest covariance so AMCL trusts the motion model without treating
        # this as ground truth.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.02
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = quat_z
        tf.transform.rotation.w = quat_w
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FakeBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

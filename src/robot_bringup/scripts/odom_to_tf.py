#!/usr/bin/env python3
"""Broadcast odom -> base_link from the chassis odometry topic.

micro-ROS firmware normally publishes nav_msgs/Odometry but cannot easily
broadcast TF.  Nav2 needs the transform, not just the topic, so this node
turns /odom into odom -> base_link.

    ros2 run robot_bringup odom_to_tf.py
    ros2 launch robot_bringup robot_bringup.launch.py publish_odom_tf:=true

Do not run it together with fake_base, which broadcasts the same transform,
or with firmware that already broadcasts odom -> base_link.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._warned_frame = False
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self.get_logger().info(
            f'broadcasting {self._odom_frame} -> {self._base_frame} '
            f'from {self.get_parameter("odom_topic").value}')

    def _on_odom(self, msg):
        if (not self._warned_frame
                and msg.header.frame_id not in ('', self._odom_frame)):
            # The message frame name does not matter for TF, but it is worth
            # reporting once so the firmware can be fixed.
            self.get_logger().warning(
                f'odom header.frame_id is {msg.header.frame_id!r}; '
                f'broadcasting as {self._odom_frame!r}')
            self._warned_frame = True

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            # Firmware without time sync leaves the stamp at zero, which tf2
            # rejects.  Fall back to the Pi clock rather than publishing a
            # transform nobody can use.
            stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
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

#!/usr/bin/env python3
"""ROS 2 node that decodes QR codes and publishes the payload.

Subscribes:
    /camera/image_raw  (sensor_msgs/Image)
Publishes:
    /qr_detected       (std_msgs/String)

Offline check against a saved image:
    ros2 run qr_scanner qr_node --image /path/to/qr.png
"""

import argparse
import sys

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .decoder import decode_qr


class QrScannerNode(Node):
    def __init__(self, image_topic, publish_topic, repeat=False, rate=5.0,
                 max_width=640, repeat_interval=2.0):
        super().__init__('qr_scanner')
        self._bridge = None
        self._last = None
        self._last_pub_ns = None
        self._repeat = repeat
        self._min_interval_ns = int(1e9 / rate) if rate > 0 else 0
        self._last_stamp_ns = None
        self._max_width = max_width
        self._repeat_interval_ns = int(repeat_interval * 1e9)
        self._publisher = self.create_publisher(String, publish_topic, 10)
        self._subscription = self.create_subscription(
            Image, image_topic, self._on_image, 10)
        self.get_logger().info(
            f'listening on {image_topic}, publishing {publish_topic}, '
            f'decoding at most {rate if rate > 0 else "unlimited"} Hz')

    def _on_image(self, msg):
        # Decoding every frame at 30 fps saturates the Pi; QR codes stay in
        # view long enough that a few Hz is plenty.
        if self._min_interval_ns:
            stamp_ns = self.get_clock().now().nanoseconds
            if (self._last_stamp_ns is not None
                    and stamp_ns - self._last_stamp_ns < self._min_interval_ns):
                return
            self._last_stamp_ns = stamp_ns

        from cv_bridge import CvBridge

        if self._bridge is None:
            self._bridge = CvBridge()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # malformed or unsupported encoding
            self.get_logger().warning(f'image conversion failed: {exc}')
            return

        try:
            text = decode_qr(frame, self._max_width)
        except Exception as exc:
            # Belt and braces: decoder.py already isolates OpenCV errors, but
            # the QR node must survive anything a camera frame can cause.
            self.get_logger().warning(
                f'decode failed: {exc}', throttle_duration_sec=5.0)
            return
        if not text:
            return
        now_ns = self.get_clock().now().nanoseconds
        same_as_last = text == self._last
        if not self._repeat and same_as_last:
            # Re-publish a payload that is still in view, but slowly.  The
            # mission state machine may not be waiting for this tag the
            # first time it is seen (it often appears while the robot is
            # still driving towards the rack), so a payload published once
            # and never again would be missed.
            if (self._repeat_interval_ns == 0
                    or (self._last_pub_ns is not None
                        and now_ns - self._last_pub_ns
                        < self._repeat_interval_ns)):
                return
        self._last = text
        self._last_pub_ns = now_ns
        self._publisher.publish(String(data=text))
        if same_as_last:
            self.get_logger().debug(f'QR still in view: {text}')
        else:
            self.get_logger().info(f'QR detected: {text}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', help='decode one image file and exit')
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--publish-topic', default='/qr_detected')
    parser.add_argument('--rate', type=float, default=5.0,
                        help='max decode attempts per second (0 = unlimited)')
    parser.add_argument('--max-width', type=int, default=640,
                        help='downscale frames wider than this before decoding')
    parser.add_argument('--repeat', action='store_true',
                        help='publish the same payload every frame')
    parser.add_argument('--repeat-interval', type=float, default=2.0,
                        help='seconds between re-publishes of a payload '
                             'that is still in view (0 = only on change)')
    parsed, ros_args = parser.parse_known_args(args)

    if parsed.image:
        frame = cv2.imread(parsed.image)
        if frame is None:
            print(f'cannot read {parsed.image}')
            return 1
        text = decode_qr(frame, parsed.max_width)
        print(text if text else 'NO_QR_FOUND')
        return 0 if text else 1

    rclpy.init(args=ros_args)
    node = QrScannerNode(
        parsed.image_topic, parsed.publish_topic, parsed.repeat,
        parsed.rate, parsed.max_width, parsed.repeat_interval)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

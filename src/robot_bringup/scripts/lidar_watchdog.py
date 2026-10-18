#!/usr/bin/env python3
"""Keep the MS200 lidar streaming by re-sending its run command when it stops.

The vendor driver only sends SET_RUN_MODE (0xA2/0x81) from Activate(), and
Activate() is only reached after the driver has already received scan data:
SetRotationSpeed() skips it while valid_data_ is zero.  An MS200 that powers
up in stop mode therefore never starts -- no data, no Activate, no data.

This node watches /scan.  When scans stop for stall_timeout seconds it sends
the same speed + run frames the SDK would send, repeating until data returns.
It leaves a healthy lidar alone: no writes at all while /scan is flowing.

    ros2 run robot_bringup lidar_watchdog.py
    ros2 launch robot_bringup robot_bringup.launch.py
"""

import os
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _frame(cmd, payload):
    """Build one 0xF5A5 command frame with the XOR check and 0x31F2 tail."""
    head = bytes([0xA5, 0xF5, cmd, 0xC1, len(payload)]) + bytes(payload)
    check = 0
    for byte in head:
        check ^= byte
    return head + bytes([check, 0x31, 0xF2])


SPEED_FRAME = _frame(0xA1, [0x0A, 0x00])   # 10 Hz
RUN_FRAME = _frame(0xA2, [0x81])           # SET_RUN_MODE: start


class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('port', '/dev/oradar')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('stall_timeout', 5.0)
        self.declare_parameter('wake_burst', 8)
        self.declare_parameter('wake_period', 1.0)
        self.declare_parameter('retry_wait', 20.0)

        self._port = self.get_parameter('port').value
        self._stall_timeout = float(self.get_parameter('stall_timeout').value)
        self._wake_burst_count = int(self.get_parameter('wake_burst').value)
        self._wake_period = float(self.get_parameter('wake_period').value)
        self._retry_wait = float(self.get_parameter('retry_wait').value)
        baud = int(self.get_parameter('baudrate').value)
        self._baud = getattr(termios, f'B{baud}', termios.B230400)

        self._last_scan = time.monotonic()
        self._last_burst = -1.0e9
        self._waking = False
        self._recovering = False
        self._seen_scan = False

        topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, topic, self._on_scan, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'watching {topic}; will wake {self._port} after '
            f'{self._stall_timeout:.1f} s without scans')

    def _on_scan(self, _msg):
        self._last_scan = time.monotonic()
        if not self._seen_scan:
            self._seen_scan = True
            self.get_logger().info('lidar stream is up')
        if self._recovering:
            self._recovering = False
            self.get_logger().info('lidar stream recovered')

    def _tick(self):
        if self._waking:
            return
        now = time.monotonic()
        if now - self._last_scan < self._stall_timeout:
            return
        if now - self._last_burst < self._retry_wait:
            return
        self._last_burst = now
        self._recovering = True
        self.get_logger().warning(
            f'no /scan for {now - self._last_scan:.1f} s; '
            f'sending wake burst to {self._port}')
        threading.Thread(target=self._wake_burst, daemon=True).start()

    def _wake_burst(self):
        self._waking = True
        try:
            for _ in range(self._wake_burst_count):
                if time.monotonic() - self._last_scan < 0.5:
                    break
                self._send_pair()
                time.sleep(self._wake_period)
        finally:
            self._waking = False

    def _send_pair(self):
        try:
            fd = os.open(self._port,
                         os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            self.get_logger().warning(
                f'cannot open {self._port}: {exc}',
                throttle_duration_sec=30.0)
            return
        try:
            # TCSANOW, not tty.setraw()'s default TCSAFLUSH: flushing input
            # would discard scan bytes the driver has not read yet.
            try:
                tty.setraw(fd, termios.TCSANOW)
            except TypeError:      # Python < 3.12 has no `when` argument
                tty.setraw(fd)
            attrs = termios.tcgetattr(fd)
            attrs[4] = attrs[5] = self._baud
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            os.write(fd, SPEED_FRAME)
            time.sleep(0.05)
            os.write(fd, RUN_FRAME)
        except OSError as exc:
            self.get_logger().warning(
                f'write to {self._port} failed: {exc}',
                throttle_duration_sec=30.0)
        finally:
            os.close(fd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdog()
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

#!/usr/bin/env python3
"""Drive a desk rehearsal of the whole mission.

Publishes the QR payloads a real field would provide: START at the beginning,
RACKx_<tag> whenever game_manager is waiting at a rack, and END at the end.
It also logs every /game_state transition with a timestamp, so the rehearsal
produces a timeline that can be compared with the 3-minute limit.

Run game_manager first, then this script:

    ros2 run game_manager game_manager
    python3 tools/mission_supervisor.py

The script exits when the state reaches DONE or after --timeout seconds.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

RACK_TAGS = {
    'A': 'RACKA_4X6M',
    'B': 'RACKB_7K2P',
    'C': 'RACKC_9Q1L',
    'D': 'RACKD_3W8N',
}


class Supervisor(Node):
    def __init__(self, rack_order, timeout):
        super().__init__('mission_supervisor')
        self._rack_order = rack_order
        self._rack_index = 0
        self._timeout = timeout
        self._start = time.monotonic()
        self._started = False
        self._state = None
        self._state_since = None
        self._done = False

        self._qr_pub = self.create_publisher(String, '/qr_detected', 10)
        self.create_subscription(String, '/game_state', self._on_state, 10)
        self.create_subscription(Int32, '/game_score', self._on_score, 10)
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'supervisor up, rack order {rack_order}, timeout {timeout}s')

    def _elapsed(self):
        return time.monotonic() - self._start

    def _on_state(self, msg):
        state = msg.data
        if state == self._state:
            return
        now = self._elapsed()
        if self._state is not None and self._state_since is not None:
            held = now - self._state_since
        else:
            held = 0.0
        self.get_logger().info(
            f'[{now:6.1f}s] {self._state} -> {state} '
            f'(previous state held {held:.1f}s)')
        self._state = state
        self._state_since = now

        if state == 'WAIT_RACK_QR':
            self._send_rack_qr()
        elif state == 'WAIT_END_QR':
            self._send('END')
        elif state == 'DONE':
            self._done = True

    def _on_score(self, msg):
        self.get_logger().info(f'[{self._elapsed():6.1f}s] score {msg.data}')

    def _send_rack_qr(self):
        if self._rack_index >= len(self._rack_order):
            self.get_logger().warning('no racks left but WAIT_RACK_QR entered')
            return
        rack = self._rack_order[self._rack_index]
        self._rack_index += 1
        self._send(RACK_TAGS.get(rack, f'RACK{rack}_TEST'))

    def _send(self, payload):
        # Wait for the subscriber so the first QR is not lost to discovery.
        for _ in range(50):
            if self._qr_pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        self._qr_pub.publish(String(data=payload))
        self.get_logger().info(f'[{self._elapsed():6.1f}s] sent {payload}')

    def _tick(self):
        if not self._started and self._elapsed() > 3.0:
            self._started = True
            self._send('START')
        if self._done or self._elapsed() > self._timeout:
            raise SystemExit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rack-order', default='A,B,C,D')
    parser.add_argument('--timeout', type=float, default=200.0)
    args = parser.parse_args()

    rclpy.init()
    node = Supervisor(args.rack_order.split(','), args.timeout)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

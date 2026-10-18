#!/usr/bin/env python3
"""Mission state machine for the DASE7503 logistics game.

The whole match is driven by two inputs: QR codes from ``/qr_detected`` and
navigation results from Nav2's ``navigate_to_pose`` action.  The robot never
"sees" the field by itself except through those two channels.

State flow::

    WAIT_START --START--> DRIVE_TO_RACK --arrived--> WAIT_RACK_QR
        --RACKx_xxxx--> LIFT_UP --lifted--> DRIVE_TO_DEST --arrived-->
        LIFT_DOWN --lowered--> next rack ...
        ... after the last rack --> WAIT_END_QR --END--> DONE

Run without Nav2 (dry run) to test the logic on a desk::

    ros2 run game_manager game_manager --ros-args -p use_nav2:=false

In dry-run mode every "drive" simply takes ``dry_run_travel_time`` seconds,
so QR events can be injected by hand::

    ros2 topic pub --once /qr_detected std_msgs/msg/String \
        "{data: 'START'}"
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # allows dry-run testing on machines without Nav2
    NavigateToPose = None

WAIT_START = 'WAIT_START'
DRIVE_TO_RACK = 'DRIVE_TO_RACK'
WAIT_RACK_QR = 'WAIT_RACK_QR'
LIFT_UP = 'LIFT_UP'
DRIVE_TO_DEST = 'DRIVE_TO_DEST'
LIFT_DOWN = 'LIFT_DOWN'
WAIT_END_QR = 'WAIT_END_QR'
DONE = 'DONE'

RACK_LETTERS = ('A', 'B', 'C', 'D')
POINTS_PER_SHIPMENT = 5


def yaw_to_quaternion(yaw):
    """Return the (x, y, z, w) quaternion for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class GameManager(Node):
    """Runs one match: START QR -> 4 racks -> destination -> END QR."""

    def __init__(self):
        super().__init__('game_manager')

        self.declare_parameter('use_nav2', True)
        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('dry_run_travel_time', 3.0)
        self.declare_parameter('mission_timeout', 180.0)
        self.declare_parameter('rack_scan_timeout', 15.0)
        self.declare_parameter('end_scan_timeout', 20.0)
        self.declare_parameter('lift_time', 1.5)
        self.declare_parameter('rack_order', 'A,B,C,D')
        self.declare_parameter('require_end_qr', True)

        # The END plate sits at the destination point, so the camera sees it
        # on every delivery.  Treating a mid-mission END as "stop" would end
        # the match after the first rack, so it is ignored unless asked for.
        self.declare_parameter('end_qr_stops_mission', False)
        # Only accept the QR of the rack we drove to.  A plate from the
        # neighbouring rack can enter the frame at the approach pose.
        self.declare_parameter('strict_rack_match', True)
        # A rack QR first seen on the way in stays valid this long, so the
        # robot does not have to re-read a plate it is already parked under.
        self.declare_parameter('rack_qr_cache_time', 30.0)

        self.declare_parameter('start_x', 2.6)
        self.declare_parameter('start_y', 0.4)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('destination_x', 0.15)
        self.declare_parameter('destination_y', 1.0)
        self.declare_parameter('destination_yaw', 3.1416)
        for letter in RACK_LETTERS:
            key = letter.lower()
            self.declare_parameter(f'rack_{key}_x', 0.8)
            self.declare_parameter(f'rack_{key}_y', 1.6)
            self.declare_parameter(f'rack_{key}_yaw', 0.0)

        # Optional per-delivery drop points inside the destination zone,
        # "x,y,yaw;x,y,yaw;...".  Each delivery uses the next one, so the
        # robot does not drive into the rack it dropped last time.
        self.declare_parameter('destination_points', '')

        self._use_nav2 = bool(self.get_parameter('use_nav2').value)
        self._set_initial_pose = bool(
            self.get_parameter('set_initial_pose').value)
        self._dry_run_time = float(
            self.get_parameter('dry_run_travel_time').value)
        self._mission_timeout = float(
            self.get_parameter('mission_timeout').value)
        self._rack_scan_timeout = float(
            self.get_parameter('rack_scan_timeout').value)
        self._end_scan_timeout = float(
            self.get_parameter('end_scan_timeout').value)
        self._lift_time = float(self.get_parameter('lift_time').value)
        self._require_end_qr = bool(
            self.get_parameter('require_end_qr').value)
        self._end_qr_stops_mission = bool(
            self.get_parameter('end_qr_stops_mission').value)
        self._strict_rack_match = bool(
            self.get_parameter('strict_rack_match').value)
        self._rack_qr_cache_time = float(
            self.get_parameter('rack_qr_cache_time').value)
        self._destination_points = self._parse_destination_points(
            str(self.get_parameter('destination_points').value))

        order = str(self.get_parameter('rack_order').value)
        self._rack_order = [letter.strip().upper()
                            for letter in order.split(',')
                            if letter.strip().upper() in RACK_LETTERS]

        self._state = WAIT_START
        self._state_pub = self.create_publisher(String, '/game_state', 10)
        self._score_pub = self.create_publisher(Int32, '/game_score', 10)
        self._lift_pub = self.create_publisher(String, '/lift_cmd', 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(
            String, '/qr_detected', self._on_qr, 10)

        self._nav_client = None
        if self._use_nav2:
            if NavigateToPose is None:
                self.get_logger().error(
                    'nav2_msgs is not installed; falling back to dry run')
                self._use_nav2 = False
            else:
                self._nav_client = ActionClient(
                    self, NavigateToPose, 'navigate_to_pose')

        self._mailbox = []       # pending QR payloads, drained in _tick
        self._mission_start = None
        self._deadline = None
        self._arrive_at = None
        self._queue = []
        self._current_rack = None
        self._delivered = []
        self._goal_handle = None
        self._dest_attempts = 0
        self._rack_attempts = 0
        self._rack_qr_cache = {}  # letter -> (payload, clock ns)

        self._set_state(WAIT_START)
        self._publish_score()
        self.create_timer(0.1, self._on_tick)

        mode = 'Nav2' if self._use_nav2 else 'DRY RUN (no Nav2)'
        self.get_logger().info(f'game_manager ready in {mode} mode; '
                               f'waiting for START QR')

    # ------------------------------------------------------------------ QR

    def _on_qr(self, msg):
        # Decode happens in another node; queue here so the state machine
        # changes in one place (the timer) instead of inside a callback.
        self._mailbox.append(msg.data.strip())
        if len(self._mailbox) > 20:
            del self._mailbox[:-20]

    def _drain_mailbox(self):
        while self._mailbox:
            text = self._mailbox.pop(0)
            self._handle_qr(text)

    def _handle_qr(self, text):
        if not text:
            return
        if text == 'START':
            if self._state == WAIT_START:
                self._begin_mission()
            else:
                self.get_logger().info('START QR ignored (mission already on)')
        elif text == 'END':
            if self._state == WAIT_END_QR:
                self._finish('END QR scanned')
            elif self._state == DONE:
                pass
            elif self._state == WAIT_START:
                self.get_logger().info('END QR ignored (mission not started)')
            elif self._end_qr_stops_mission:
                # Opt-in emergency stop; never on for the real field.
                self.get_logger().warning('END QR seen mid-mission, finishing')
                self._finish('END QR scanned early')
            else:
                # The END plate is at the destination, so it is in view on
                # every delivery.  It must not end the match early.
                self.get_logger().info(
                    f'END QR seen while {self._state}, ignored')
        elif text.startswith('RACK'):
            self._handle_rack_qr(text)
        else:
            self.get_logger().warn(f'unknown QR payload: {text!r}')

    def _handle_rack_qr(self, text):
        # RACKA_4X6M -> letter 'A'
        letter = text[4:5].upper() if len(text) > 4 else ''
        if letter not in RACK_LETTERS:
            self.get_logger().warn(f'not a rack QR: {text!r}')
            return

        # Keep every rack tag we see.  The camera usually reads the plate
        # while the robot is still driving towards the rack, before the
        # state machine is waiting for it; the cached copy is reused on
        # arrival so a plate that is now out of view is not lost.
        self._rack_qr_cache[letter] = (
            text, self.get_clock().now().nanoseconds)

        if self._state != WAIT_RACK_QR:
            self.get_logger().debug(f'{text} seen while {self._state}, cached')
            return
        if self._current_rack and letter != self._current_rack:
            if self._strict_rack_match:
                self.get_logger().warning(
                    f'at rack {self._current_rack} but scanned {letter}; '
                    f'ignoring it')
                return
            self.get_logger().warn(
                f'expected rack {self._current_rack} but scanned '
                f'{letter}; using the scanned one for delivery record')
            self._current_rack = letter
        self.get_logger().info(f'rack QR confirmed: {text}')
        self._lift_up()

    # --------------------------------------------------------------- states

    def _set_state(self, state):
        self._state = state
        self._state_pub.publish(String(data=state))
        self.get_logger().info(f'state -> {state}')

    def _begin_mission(self):
        self._mission_start = self.get_clock().now()
        self._delivered = []
        self._queue = list(self._rack_order)
        self._dest_attempts = 0
        self._rack_qr_cache.clear()
        self._publish_score()
        self.get_logger().info(
            f'START scanned; {len(self._queue)} racks queued: '
            f'{", ".join(self._queue)}')
        if self._set_initial_pose:
            self._publish_initial_pose()
        self._next_rack()

    def _next_rack(self):
        if self._mission_timed_out():
            self._finish('mission timeout')
            return
        if not self._queue:
            if self._require_end_qr:
                self._set_state(WAIT_END_QR)
                self._deadline = self._now_plus(self._end_scan_timeout)
                self.get_logger().info(
                    'all racks delivered; waiting for END QR')
            else:
                self._finish('all racks delivered')
            return
        self._current_rack = self._queue.pop(0)
        self._rack_attempts = 0
        self._set_state(DRIVE_TO_RACK)
        key = self._current_rack.lower()
        self._drive_to(
            self._param(f'rack_{key}_x'),
            self._param(f'rack_{key}_y'),
            self._param(f'rack_{key}_yaw'),
        )

    def _lift_up(self):
        self._set_state(LIFT_UP)
        self._lift_pub.publish(String(data='up'))
        self._deadline = self._now_plus(self._lift_time)
        self.get_logger().info(
            f'lift up for rack {self._current_rack}')

    def _drive_to_destination(self):
        self._set_state(DRIVE_TO_DEST)
        self._drive_to(*self._destination_pose())

    def _lift_down(self):
        self._set_state(LIFT_DOWN)
        self._lift_pub.publish(String(data='down'))
        self._deadline = self._now_plus(self._lift_time)

    def _delivery_done(self):
        if self._current_rack and self._current_rack not in self._delivered:
            self._delivered.append(self._current_rack)
        self._publish_score()
        self.get_logger().info(
            f'shipment delivered ({len(self._delivered)} total, '
            f'score {len(self._delivered) * POINTS_PER_SHIPMENT})')
        self._current_rack = None
        self._dest_attempts = 0
        self._next_rack()

    def _finish(self, reason):
        if self._state == DONE:
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._lift_pub.publish(String(data='down'))
        self._set_state(DONE)
        self.get_logger().info(
            f'mission finished: {reason}; '
            f'{len(self._delivered)} shipments, '
            f'score {len(self._delivered) * POINTS_PER_SHIPMENT}')

    # -------------------------------------------------------------- driving

    def _drive_to(self, x, y, yaw):
        if self._state == DRIVE_TO_RACK:
            self._dest_attempts = 0
        self.get_logger().info(
            f'driving to ({x:.2f}, {y:.2f}, {yaw:.2f})')
        if not self._use_nav2:
            self._arrive_at = self._now_plus(self._dry_run_time)
            return
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'navigate_to_pose server not available; aborting drive')
            self._on_arrival_failed()
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)
        self._nav_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self._pose(
            self._param('start_x'),
            self._param('start_y'),
            self._param('start_yaw')).pose
        for i in range(6):
            msg.pose.covariance[i * 7] = 0.25 if i < 3 else 0.07
        self._initial_pose_pub.publish(msg)
        self.get_logger().info('initial pose published')

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('navigation goal rejected')
            self._on_arrival_failed()
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_arrival()
        else:
            self.get_logger().warn(f'navigation failed (status={status})')
            self._on_arrival_failed()

    def _on_arrival(self):
        if self._state == DRIVE_TO_RACK:
            self._set_state(WAIT_RACK_QR)
            self._deadline = self._now_plus(self._rack_scan_timeout)
            self.get_logger().info(
                f'at rack {self._current_rack}; waiting for its QR')
            cached = self._recent_rack_qr(self._current_rack)
            if cached is not None:
                self.get_logger().info(
                    f'rack QR read on approach: {cached}')
                self._handle_rack_qr(cached)
        elif self._state == DRIVE_TO_DEST:
            self._lift_down()
        else:
            self.get_logger().warn(f'arrival in unexpected state {self._state}')

    def _on_arrival_failed(self):
        if self._state == DRIVE_TO_RACK:
            self._rack_attempts += 1
            if self._rack_attempts < 2:
                # A single action-server timeout on a loaded Pi must not
                # cost a whole rack; try the same goal once more.
                self.get_logger().warn(
                    f'cannot reach rack {self._current_rack}; retrying once')
                key = self._current_rack.lower()
                self._drive_to(
                    self._param(f'rack_{key}_x'),
                    self._param(f'rack_{key}_y'),
                    self._param(f'rack_{key}_yaw'))
            else:
                self.get_logger().warn(
                    f'giving up on rack {self._current_rack}')
                self._next_rack()
        elif self._state == DRIVE_TO_DEST:
            self._dest_attempts += 1
            if self._dest_attempts < 2:
                self.get_logger().warn('retrying destination once')
                self._drive_to(*self._destination_pose())
            else:
                self._finish('cannot reach destination')

    # ---------------------------------------------------------------- timer

    def _on_tick(self):
        self._drain_mailbox()
        if self._state == DONE:
            return
        now = self.get_clock().now()

        if self._state == WAIT_START:
            return
        if self._mission_timed_out():
            self._finish('mission timeout')
            return

        if self._state in (DRIVE_TO_RACK, DRIVE_TO_DEST):
            if self._arrive_at is not None and now >= self._arrive_at:
                self._arrive_at = None
                self._on_arrival()
        elif self._state == WAIT_RACK_QR:
            if now >= self._deadline:
                self.get_logger().warn(
                    f'no rack QR within {self._rack_scan_timeout:.0f} s; '
                    f'skipping rack {self._current_rack}')
                self._next_rack()
        elif self._state in (LIFT_UP, LIFT_DOWN):
            if now >= self._deadline:
                if self._state == LIFT_UP:
                    self._drive_to_destination()
                else:
                    self._delivery_done()
        elif self._state == WAIT_END_QR:
            if now >= self._deadline:
                self._finish('END QR not seen in time')

    # -------------------------------------------------------------- helpers

    def _mission_timed_out(self):
        if self._mission_start is None:
            return False
        elapsed = (self.get_clock().now() - self._mission_start).nanoseconds
        return elapsed >= self._mission_timeout * 1e9

    def _now_plus(self, seconds):
        return self.get_clock().now() + Duration(seconds=seconds)

    def _destination_pose(self):
        """Return (x, y, yaw) for the current delivery."""
        if self._destination_points:
            index = len(self._delivered) % len(self._destination_points)
            return self._destination_points[index]
        return (self._param('destination_x'),
                self._param('destination_y'),
                self._param('destination_yaw'))

    def _recent_rack_qr(self, letter):
        entry = self._rack_qr_cache.get(letter)
        if entry is None:
            return None
        payload, stamp_ns = entry
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age > self._rack_qr_cache_time:
            return None
        return payload

    def _parse_destination_points(self, spec):
        points = []
        for chunk in spec.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            values = [value.strip() for value in chunk.split(',')]
            if len(values) != 3:
                self.get_logger().error(
                    f'bad destination point {chunk!r}, expected "x,y,yaw"')
                continue
            try:
                points.append(tuple(float(value) for value in values))
            except ValueError:
                self.get_logger().error(
                    f'bad destination point {chunk!r}, expected "x,y,yaw"')
        return points

    def _param(self, name):
        return float(self.get_parameter(name).value)

    def _publish_score(self):
        self._score_pub.publish(
            Int32(data=len(self._delivered) * POINTS_PER_SHIPMENT))


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
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

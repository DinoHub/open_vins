"""
PX4-offboard figure-8 flight controller for OpenVINS evaluation.

State machine (see class docstring) that streams mavros setpoints, engages
OFFBOARD + arms, climbs to a target altitude relative to wherever the vehicle
currently is (no assumption about absolute world spawn pose), flies a
Figure8Trajectory for a configured number of loops, then holds position.

Safety posture: never lands or disarms unless `land_after_completion` is
explicitly set true, and any loss of armed/OFFBOARD mid-flight immediately
freezes the last commanded setpoint (no automatic re-arm/re-engage).
"""

from enum import auto, Enum
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from vo_eval_tools.trajectory import Figure8Trajectory, Figure8WeaveTrajectory


class FlightState(Enum):
    WAIT_FCU_CONNECT = auto()
    WAIT_POSE = auto()
    PRE_STREAM = auto()
    ENGAGE = auto()
    CLIMB = auto()
    STABILIZE = auto()
    FIGURE8 = auto()
    DONE = auto()
    ABORT_HOLD = auto()


def yaw_from_quaternion(q):
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Send position + velocity + yaw, ignore acceleration and yaw-rate.
# Verified against the installed mavros_msgs/msg/PositionTarget.msg constants:
# IGNORE_AFX=64, IGNORE_AFY=128, IGNORE_AFZ=256, IGNORE_YAW_RATE=2048 -> 2496.
TYPE_MASK_POS_VEL_YAW = (
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)

PARAM_DEFAULTS = {
    'mavros_ns': '/px4vision_custom_0/mavros',
    'tick_rate_hz': 20.0,
    'pre_stream_duration_s': 5.0,
    'offboard_confirm_timeout_s': 5.0,
    'climb_height_m': 1.0,
    'climb_rate_mps': 0.3,
    'settle_duration_s': 3.0,
    'half_length_m': 2.5,
    'half_width_m': 1.5,
    'loop_period_s': 20.0,
    'num_loops': 10,
    'ramp_duration_s': 3.0,
    'pattern_yaw_offset_rad': 0.0,
    'weave_height_m': 0.0,
    'weave_cycles_per_loop': 2,
    'max_speed_warn_mps': 2.0,
    'land_after_completion': False,
    'max_offboard_retries': 1,
}


class Figure8OffboardController(Node):

    def __init__(self):
        super().__init__('figure8_offboard_controller')

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAM_DEFAULTS}

        # Best-effort QoS to talk to mavros: a best-effort subscriber is
        # compatible with either a reliable or best-effort mavros publisher,
        # and matches the QoS convention already used against this same
        # mavros setpoint_raw plugin group elsewhere in this workspace
        # (attitude_pid's pid_controller_node.cpp).
        self._mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        ns = self.p['mavros_ns']

        self._state = None
        self._pose = None

        self.sub_state = self.create_subscription(
            State, f'{ns}/state', self._on_state, self._mavros_qos)
        self.sub_pose = self.create_subscription(
            PoseStamped, f'{ns}/local_position/pose', self._on_pose, self._mavros_qos)

        self.pub_setpoint = self.create_publisher(
            PositionTarget, f'{ns}/setpoint_raw/local', self._mavros_qos)
        self.pub_status = self.create_publisher(String, '~/status', status_qos)

        self.cli_arming = self.create_client(CommandBool, f'{ns}/cmd/arming')
        self.cli_set_mode = self.create_client(SetMode, f'{ns}/set_mode')

        self.flight_state = FlightState.WAIT_FCU_CONNECT
        self._abort_reason = None
        self._state_entry_time = None
        self._pre_stream_count = 0
        self._engage_calls_issued = False
        self._engage_attempt = 0
        self._hold_xyz = None
        self._hold_yaw = 0.0
        self._climb_target_z = None
        self._climb_start_z = None
        self._climb_start_time = None
        self._figure8 = None
        self._figure8_total = None
        self._last_setpoint = None
        self._done_status_published = False
        self._completed_loops = 0

        max_speed = self._analytic_max_speed()
        self.get_logger().info(
            f"Figure-8 params: half_length={self.p['half_length_m']}m "
            f"half_width={self.p['half_width_m']}m loop_period={self.p['loop_period_s']}s "
            f"num_loops={self.p['num_loops']} -> analytic max horizontal speed "
            f'= {max_speed:.2f} m/s')
        if max_speed > self.p['max_speed_warn_mps']:
            self.get_logger().warn(
                f'Analytic max speed {max_speed:.2f} m/s exceeds max_speed_warn_mps '
                f"({self.p['max_speed_warn_mps']} m/s) - consider a longer loop_period_s "
                f'or smaller half_length_m/half_width_m.')

        if self.p['weave_height_m'] > 0.0:
            max_vspeed = self._analytic_max_vertical_speed()
            self.get_logger().info(
                f"Weave enabled: weave_height={self.p['weave_height_m']}m "
                f"({self.p['weave_cycles_per_loop']} cycles/loop) "
                f'-> analytic max vertical speed = {max_vspeed:.2f} m/s')
            if max_vspeed > self.p['max_speed_warn_mps']:
                self.get_logger().warn(
                    f'Analytic max vertical speed {max_vspeed:.2f} m/s exceeds '
                    f"max_speed_warn_mps ({self.p['max_speed_warn_mps']} m/s) - consider a "
                    f'smaller weave_height_m or fewer weave_cycles_per_loop.')

        self._state_entry_time = self._now()
        period = 1.0 / self.p['tick_rate_hz']
        self.timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------- helpers

    def _analytic_max_speed(self):
        omega = 2.0 * math.pi / self.p['loop_period_s']
        return omega * math.hypot(self.p['half_length_m'], 2.0 * self.p['half_width_m'])

    def _analytic_max_vertical_speed(self):
        omega = 2.0 * math.pi / self.p['loop_period_s']
        weave_amplitude = self.p['weave_height_m'] / 2.0
        weave_omega = self.p['weave_cycles_per_loop'] * omega
        return weave_amplitude * weave_omega

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _enter(self, new_state: FlightState):
        self.get_logger().info(f'{self.flight_state.name} -> {new_state.name}')
        self.flight_state = new_state
        self._state_entry_time = self._now()
        self._publish_status(new_state.name)

    def _enter_abort(self, reason):
        if self.flight_state == FlightState.ABORT_HOLD:
            return
        self._abort_reason = reason
        self.get_logger().error(f'ABORT: {reason}')
        self.flight_state = FlightState.ABORT_HOLD
        self._state_entry_time = self._now()
        self._publish_status(f'ABORTED:{reason}')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _update_loop_count(self, elapsed):
        completed = min(int(elapsed // self.p['loop_period_s']), self.p['num_loops'])
        if completed != self._completed_loops:
            self._completed_loops = completed
            self.get_logger().info(
                f"Figure-8: completed loop {completed}/{self.p['num_loops']}")

    def _time_in_state(self):
        return self._now() - self._state_entry_time

    def _make_setpoint(self, x, y, z, vx, vy, vz, yaw):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK_POS_VEL_YAW
        msg.position.x, msg.position.y, msg.position.z = float(x), float(y), float(z)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(vx), float(vy), float(vz)
        msg.yaw = float(yaw)
        return msg

    def _publish_setpoint(self, x, y, z, vx, vy, vz, yaw):
        self._last_setpoint = (x, y, z, vx, vy, vz, yaw)
        self.pub_setpoint.publish(self._make_setpoint(x, y, z, vx, vy, vz, yaw))

    def _republish_last_setpoint(self):
        if self._last_setpoint is not None:
            self._publish_setpoint(*self._last_setpoint)

    # -------------------------------------------------------------- topics

    def _on_state(self, msg: State):
        self._state = msg
        if self.flight_state in (FlightState.CLIMB, FlightState.STABILIZE, FlightState.FIGURE8):
            if not msg.armed or msg.mode != 'OFFBOARD':
                self._enter_abort(
                    f'lost armed/OFFBOARD during flight (armed={msg.armed}, mode={msg.mode})')

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    # ---------------------------------------------------------------- tick

    def _tick(self):
        handler = getattr(self, f'_tick_{self.flight_state.name.lower()}', None)
        if handler is None:
            self.get_logger().error(f'No tick handler for state {self.flight_state}')
            return
        handler()

    def _tick_wait_fcu_connect(self):
        if self._state is not None and self._state.connected:
            self._enter(FlightState.WAIT_POSE)

    def _tick_wait_pose(self):
        if self._pose is not None:
            pos = self._pose.pose.position
            self._hold_xyz = (pos.x, pos.y, pos.z)
            self._hold_yaw = yaw_from_quaternion(self._pose.pose.orientation)
            self._pre_stream_count = 0
            self._enter(FlightState.PRE_STREAM)

    def _tick_pre_stream(self):
        x, y, z = self._hold_xyz
        self._publish_setpoint(x, y, z, 0.0, 0.0, 0.0, self._hold_yaw)
        self._pre_stream_count += 1
        needed = int(self.p['pre_stream_duration_s'] * self.p['tick_rate_hz'])
        if self._pre_stream_count >= needed:
            self._engage_calls_issued = False
            self._engage_attempt = 0
            self._enter(FlightState.ENGAGE)

    def _tick_engage(self):
        x, y, z = self._hold_xyz
        self._publish_setpoint(x, y, z, 0.0, 0.0, 0.0, self._hold_yaw)

        if not self._engage_calls_issued:
            self._engage_calls_issued = True
            self._engage_attempt += 1
            self._issue_engage_calls()

        if self._state is not None and self._state.armed and self._state.mode == 'OFFBOARD':
            self._climb_start_z = z
            self._climb_target_z = z + self.p['climb_height_m']
            self._enter(FlightState.CLIMB)
            return

        if self._time_in_state() > self.p['offboard_confirm_timeout_s']:
            if self._engage_attempt <= self.p['max_offboard_retries']:
                self.get_logger().warn(
                    f'OFFBOARD/arm not confirmed, retrying (attempt {self._engage_attempt + 1})')
                self._engage_calls_issued = False
                self._state_entry_time = self._now()
            else:
                self._enter_abort('OFFBOARD/arm not confirmed after max_offboard_retries')

    def _issue_engage_calls(self):
        if not self.cli_set_mode.service_is_ready():
            self.get_logger().warn(
                'set_mode service not ready yet, will retry next engage attempt')
            self._engage_calls_issued = False
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.cli_set_mode.call_async(req)
        future.add_done_callback(self._on_set_mode_response)

    def _on_set_mode_response(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'set_mode service call failed: {exc}')
            return
        if not resp.mode_sent:
            self.get_logger().warn(
                'set_mode(OFFBOARD) not accepted by mavros, will rely on retry/timeout')
            return
        if not self.cli_arming.service_is_ready():
            self.get_logger().warn('arming service not ready yet')
            return
        req = CommandBool.Request()
        req.value = True
        future2 = self.cli_arming.call_async(req)
        future2.add_done_callback(self._on_arming_response)

    def _on_arming_response(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'arming service call failed: {exc}')
            return
        if not resp.success:
            self.get_logger().warn('arming request not accepted, will rely on retry/timeout')

    def _tick_climb(self):
        elapsed = self._time_in_state()
        total_climb_time = abs(self.p['climb_height_m']) / max(self.p['climb_rate_mps'], 1e-3)
        frac = min(elapsed / total_climb_time, 1.0) if total_climb_time > 0 else 1.0
        x, y = self._hold_xyz[0], self._hold_xyz[1]
        z = self._climb_start_z + frac * (self._climb_target_z - self._climb_start_z)
        vz = self.p['climb_rate_mps'] if frac < 1.0 else 0.0
        self._publish_setpoint(x, y, z, 0.0, 0.0, vz, self._hold_yaw)
        if frac >= 1.0:
            self._enter(FlightState.STABILIZE)

    def _tick_stabilize(self):
        x, y = self._hold_xyz[0], self._hold_xyz[1]
        z = self._climb_target_z
        self._publish_setpoint(x, y, z, 0.0, 0.0, 0.0, self._hold_yaw)
        if self._time_in_state() >= self.p['settle_duration_s']:
            if self.p['weave_height_m'] > 0.0:
                self._figure8 = Figure8WeaveTrajectory(
                    half_length_m=self.p['half_length_m'],
                    half_width_m=self.p['half_width_m'],
                    loop_period_s=self.p['loop_period_s'],
                    center_xyz=(x, y, z),
                    start_yaw_rad=self._hold_yaw,
                    weave_height_m=self.p['weave_height_m'],
                    weave_cycles_per_loop=self.p['weave_cycles_per_loop'],
                    pattern_yaw_offset_rad=self.p['pattern_yaw_offset_rad'],
                )
            else:
                self._figure8 = Figure8Trajectory(
                    half_length_m=self.p['half_length_m'],
                    half_width_m=self.p['half_width_m'],
                    loop_period_s=self.p['loop_period_s'],
                    center_xyz=(x, y, z),
                    start_yaw_rad=self._hold_yaw,
                    pattern_yaw_offset_rad=self.p['pattern_yaw_offset_rad'],
                )
            self._figure8_total = self._figure8.total_duration(self.p['num_loops'])
            self._completed_loops = 0
            self._enter(FlightState.FIGURE8)

    def _tick_figure8(self):
        elapsed = self._time_in_state()
        total = self._figure8_total
        self._update_loop_count(elapsed)
        if elapsed >= total:
            self._enter(FlightState.DONE)
            return

        # Note: sample() also returns a tangent-to-curve yaw, but we deliberately
        # don't use it - atan2(vy, vx) is ill-conditioned near the figure-8's
        # crossing point and loop start/end (where local vx passes through zero
        # while vy is near its peak), producing sharp near-90-degree heading
        # snaps twice per loop. Holding yaw fixed avoids that entirely and is
        # unnecessary to drop for a position-accuracy test anyway.
        x, y, z, vx, vy, vz, _yaw = self._figure8.sample(elapsed)

        ramp = self.p['ramp_duration_s']
        if ramp > 0.0:
            blend = max(0.0, min(1.0, min(elapsed, total - elapsed) / ramp))
        else:
            blend = 1.0
        vx *= blend
        vy *= blend
        vz *= blend

        self._publish_setpoint(x, y, z, vx, vy, vz, self._hold_yaw)

    def _tick_done(self):
        if not self._done_status_published:
            self._done_status_published = True
            self._publish_status('COMPLETE')
            self.get_logger().info('Figure-8 evaluation flight COMPLETE, holding position.')
            if self.p['land_after_completion']:
                self._request_land()
        self._republish_last_setpoint()

    def _request_land(self):
        if not self.cli_set_mode.service_is_ready():
            self.get_logger().warn('set_mode service not ready for land request')
            return
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.cli_set_mode.call_async(req)

    def _tick_abort_hold(self):
        self._republish_last_setpoint()


def main(args=None):
    rclpy.init(args=args)
    node = Figure8OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

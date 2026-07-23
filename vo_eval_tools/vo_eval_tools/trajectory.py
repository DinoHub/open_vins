"""
Pure-math figure-8 (lemniscate) trajectory generator.

No ROS dependency so it can be unit tested standalone. Produces a closed,
periodic curve in a local XY plane that starts and ends at the local origin,
then rotates it by a fixed yaw and translates it to a world-frame center.
Velocity is the analytic time-derivative of position (not a finite-difference
approximation), so position and velocity feed-forward setpoints sent to the
flight controller are always mutually consistent.
"""

import math


class Figure8Trajectory:
    """
    Lemniscate-style figure-8 parametrized by elapsed time.

    Local-frame parametrization (before rotation/translation):
        x(t) = half_length * cos(omega*t) - half_length
        y(t) = half_width  * sin(2*omega*t)
    where omega = 2*pi / loop_period_s.

    x(0) = y(0) = 0, and both are periodic with period loop_period_s, so the
    curve starts and ends exactly at the local origin every loop.
    """

    def __init__(self, half_length_m, half_width_m, loop_period_s,
                 center_xyz, start_yaw_rad, pattern_yaw_offset_rad=0.0):
        if half_length_m <= 0.0 or half_width_m <= 0.0:
            raise ValueError('half_length_m and half_width_m must be positive')
        if loop_period_s <= 0.0:
            raise ValueError('loop_period_s must be positive')

        self.half_length = half_length_m
        self.half_width = half_width_m
        self.loop_period = loop_period_s
        self.omega = 2.0 * math.pi / loop_period_s

        self.center_x, self.center_y, self.center_z = center_xyz
        self.yaw_rot = start_yaw_rad + pattern_yaw_offset_rad
        self._cos_r = math.cos(self.yaw_rot)
        self._sin_r = math.sin(self.yaw_rot)

    def _local_position(self, t):
        x = self.half_length * math.cos(self.omega * t) - self.half_length
        y = self.half_width * math.sin(2.0 * self.omega * t)
        return x, y

    def _local_velocity(self, t):
        vx = -self.half_length * self.omega * math.sin(self.omega * t)
        vy = 2.0 * self.half_width * self.omega * math.cos(2.0 * self.omega * t)
        return vx, vy

    def _rotate(self, x, y):
        wx = self._cos_r * x - self._sin_r * y
        wy = self._sin_r * x + self._cos_r * y
        return wx, wy

    def sample(self, t):
        """
        Sample world-frame ENU position/velocity/yaw at local time t (s).

        Returns (x, y, z, vx, vy, vz, yaw_rad).
        """
        lx, ly = self._local_position(t)
        lvx, lvy = self._local_velocity(t)
        wx, wy = self._rotate(lx, ly)
        wvx, wvy = self._rotate(lvx, lvy)

        x = self.center_x + wx
        y = self.center_y + wy
        z = self.center_z

        if wvx != 0.0 or wvy != 0.0:
            yaw = math.atan2(wvy, wvx)
        else:
            yaw = self.yaw_rot

        return x, y, z, wvx, wvy, 0.0, yaw

    def max_horizontal_speed(self):
        """
        Exact maximum horizontal speed (m/s) attained anywhere on the curve.

        Derived analytically: speed^2(t)/omega^2 is a convex quadratic in
        sin^2(omega*t), so its maximum over one period occurs at one of the
        two endpoints sin^2=0 or sin^2=1, giving
            max_speed = omega * hypot(half_length, 2*half_width).
        """
        return self.omega * math.hypot(self.half_length, 2.0 * self.half_width)

    def total_duration(self, num_loops):
        return num_loops * self.loop_period


class Figure8WeaveTrajectory(Figure8Trajectory):
    """
    Figure8Trajectory with an added vertical weave.

    Same horizontal lemniscate as the base class; z additionally oscillates:
        z(t) = center_z + (weave_height_m / 2) * sin(weave_cycles_per_loop * omega * t)

    weave_cycles_per_loop should be a positive integer so the weave closes
    cleanly at the end of every lap, matching the horizontal curve's own
    per-loop periodicity (z(0) == z(loop_period) == center_z either way).
    """

    def __init__(self, half_length_m, half_width_m, loop_period_s, center_xyz, start_yaw_rad,
                 weave_height_m, weave_cycles_per_loop=2, pattern_yaw_offset_rad=0.0):
        super().__init__(
            half_length_m, half_width_m, loop_period_s, center_xyz, start_yaw_rad,
            pattern_yaw_offset_rad)
        if weave_height_m < 0.0:
            raise ValueError('weave_height_m must be >= 0')
        if weave_cycles_per_loop <= 0:
            raise ValueError('weave_cycles_per_loop must be positive')

        self.weave_height = weave_height_m
        self.weave_amplitude = weave_height_m / 2.0
        self.weave_cycles_per_loop = weave_cycles_per_loop
        self.weave_omega = weave_cycles_per_loop * self.omega

    def sample(self, t):
        x, y, _z, vx, vy, _vz, yaw = super().sample(t)
        z = self.center_z + self.weave_amplitude * math.sin(self.weave_omega * t)
        vz = self.weave_amplitude * self.weave_omega * math.cos(self.weave_omega * t)
        return x, y, z, vx, vy, vz, yaw

    def max_vertical_speed(self):
        """Exact maximum |vz| (m/s): the sinusoid's amplitude * angular rate."""
        return self.weave_amplitude * self.weave_omega

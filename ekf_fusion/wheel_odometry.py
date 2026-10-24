"""Wheel odometry from encoder ticks (no ROS imports, so it can be tested on its own).

A real robot never knows its wheel radii and wheel separation exactly. This class takes
the *believed* geometry, which can be a little off from the true one, and integrates the
encoder ticks the same way a motor driver board would."""
import math


class WheelOdometry:
    def __init__(self, r_left, r_right, separation, ticks_per_rev):
        self.r_left, self.r_right = r_left, r_right
        self.b = separation
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0
        self._last = None          # (t, ticks_left, ticks_right)

    def ticks(self, wheel_angle):
        """Encoder count for a wheel angle in rad (what the encoder would report)."""
        return math.floor(wheel_angle / (2 * math.pi) * self.ticks_per_rev)

    def update(self, t, angle_left, angle_right):
        tl, tr = self.ticks(angle_left), self.ticks(angle_right)
        if self._last is None:
            self._last = (t, tl, tr)
            return False
        t0, tl0, tr0 = self._last
        dt = t - t0
        if dt <= 0.0:
            return False
        rad_per_tick = 2 * math.pi / self.ticks_per_rev
        dl = (tl - tl0) * rad_per_tick * self.r_left
        dr = (tr - tr0) * rad_per_tick * self.r_right
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / self.b
        # midpoint integration: heading halfway through the step
        self.x += ds * math.cos(self.theta + 0.5 * dth)
        self.y += ds * math.sin(self.theta + 0.5 * dth)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
        self.v, self.w = ds / dt, dth / dt
        self._last = (t, tl, tr)
        return True

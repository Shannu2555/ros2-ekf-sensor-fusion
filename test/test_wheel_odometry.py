import math

from ekf_fusion.wheel_odometry import WheelOdometry


def drive(odo, wl, wr, seconds, dt=0.01, start=(0.0, 0.0, 0.0)):
    """Spin the wheels at wl, wr rad/s and feed the encoder angles in."""
    t, al, ar = start
    odo.update(t, al, ar)                           # first reading only sets the reference
    for _ in range(round(seconds / dt)):
        t, al, ar = t + dt, al + wl * dt, ar + wr * dt
        odo.update(t, al, ar)
    return t, al, ar


def test_straight_line_with_perfect_calibration():
    odo = WheelOdometry(0.05, 0.05, 0.30, 100000)
    drive(odo, 6.0, 6.0, 10.0)                      # 0.3 m/s for 10 s
    assert abs(odo.x - 3.0) < 1e-3 and abs(odo.y) < 1e-6 and abs(odo.theta) < 1e-6


def test_turn_in_place_quarter_turn():
    odo = WheelOdometry(0.05, 0.05, 0.30, 100000)
    w = (math.pi / 2) * 0.15 / 0.05 / 2.0           # wheel rate for a 90 deg turn in 2 s
    drive(odo, -w, w, 2.0)
    assert abs(odo.theta - math.pi / 2) < 1e-3 and math.hypot(odo.x, odo.y) < 1e-4


def test_radius_mismatch_bends_a_straight_line():
    # equal wheel speeds, but the software thinks the right wheel is smaller -> it "sees" a right turn
    odo = WheelOdometry(0.050125, 0.049875, 0.30, 100000)
    drive(odo, 6.0, 6.0, 10.0)
    expected = (0.049875 - 0.050125) * 60.0 / 0.30  # (dr - dl) / b over 60 rad of wheel rotation
    assert abs(odo.theta - expected) < 1e-3 and odo.theta < 0


def test_encoder_quantisation_is_bounded():
    odo = WheelOdometry(0.05, 0.05, 0.30, 2048)
    drive(odo, 6.0, 6.0, 10.0)
    tick = 2 * math.pi / 2048 * 0.05
    assert abs(odo.x - 3.0) < 2 * tick

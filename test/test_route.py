import math

import numpy as np

from ekf_fusion.route import PurePursuit, square_laps


def test_laps_length_and_closure():
    p = square_laps(side=3.0, laps=5, radius=0.6)
    length = np.sum(np.linalg.norm(np.diff(p, axis=1), axis=0))
    lap = 4 * (3.0 - 2 * 0.6) + 2 * math.pi * 0.6
    assert abs(length - 5 * lap) < 0.05
    assert np.allclose(p[:, 0], [0, 0]) and np.allclose(p[:, -1], [0, 0], atol=1e-9)


def test_path_has_no_jumps():
    p = square_laps(3.0, 2, 0.6)
    steps = np.linalg.norm(np.diff(p, axis=1), axis=0)
    assert steps.max() < 0.02


def test_pure_pursuit_follows_the_route_with_a_perfect_robot():
    path = square_laps(3.0, 1, 0.6)
    pp = PurePursuit(path, lookahead=0.35, v=0.3)
    x = y = th = 0.0
    dt, worst = 0.05, 0.0
    for _ in range(4000):
        v, w = pp.command(x, y, th)
        if pp.done():
            break
        x, y, th = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt
        worst = max(worst, float(np.min(np.linalg.norm(path - np.array([[x], [y]]), axis=0))))
    assert pp.done()
    assert worst < 0.05

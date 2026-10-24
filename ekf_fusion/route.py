"""The route the robot drives: laps of a rounded square, plus a tiny pure pursuit to follow it (no ROS)."""
import math

import numpy as np


def square_laps(side=3.0, laps=5, radius=0.6, spacing=0.01):
    """Counter-clockwise laps of a square with rounded corners. The robot starts at the origin,
    in the middle of the bottom edge, facing +x. Returns a (2, M) array of points."""
    h, r = side / 2.0, radius
    pieces = []

    def line(x0, y0, x1, y1):
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / spacing)))
        s = np.arange(n) / n
        pieces.append(np.vstack([x0 + s * (x1 - x0), y0 + s * (y1 - y0)]))

    def arc(cx, cy, a0):
        n = max(1, int(round(r * math.pi / 2 / spacing)))
        a = a0 + np.arange(n) / n * math.pi / 2
        pieces.append(np.vstack([cx + r * np.cos(a), cy + r * np.sin(a)]))

    for _ in range(laps):
        line(0, 0, h - r, 0)
        arc(h - r, r, -math.pi / 2)
        line(h, r, h, side - r)
        arc(h - r, side - r, 0.0)
        line(h - r, side, -h + r, side)
        arc(-h + r, side - r, math.pi / 2)
        line(-h, side - r, -h, r)
        arc(-h + r, r, math.pi)
        line(-h + r, 0, 0, 0)
    pieces.append(np.zeros((2, 1)))
    return np.hstack(pieces)


class PurePursuit:
    """Follows the path with a fixed lookahead. Searches forward only, so the laps don't get
    confused with each other where they overlap."""

    def __init__(self, path, lookahead=0.35, v=0.3, w_max=1.2, window=150):
        self.path, self.Ld, self.v, self.w_max, self.win = path, lookahead, v, w_max, window
        d = np.linalg.norm(np.diff(path, axis=1), axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.idx = 0

    def progress(self):
        return self.s[self.idx] / self.s[-1]

    def done(self):
        return self.idx >= self.path.shape[1] - 3

    def command(self, x, y, theta):
        lo, hi = self.idx, min(self.idx + self.win, self.path.shape[1])
        d = np.linalg.norm(self.path[:, lo:hi] - np.array([[x], [y]]), axis=0)
        self.idx = lo + int(np.argmin(d))
        if self.done():
            return 0.0, 0.0
        j = min(int(np.searchsorted(self.s, self.s[self.idx] + self.Ld)), self.path.shape[1] - 1)
        dx, dy = self.path[0, j] - x, self.path[1, j] - y
        ly = -math.sin(theta) * dx + math.cos(theta) * dy           # lookahead point, robot frame
        w = self.v * 2.0 * ly / self.Ld ** 2
        return self.v, float(np.clip(w, -self.w_max, self.w_max))

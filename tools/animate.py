"""Replays a logged run as a video: ground truth, wheel odometry and the fused EKF driving the laps.
Every frame comes straight from the CSV.

  python3 tools/animate.py results/slippery.csv docs/media/slippery_replay.mp4 --gif docs/media/slippery_replay.gif
"""
import argparse
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter, PillowWriter  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from analyze import PATCH, errors, load  # noqa: E402

TRACKS = [("truth", "Ground truth", "#222222"), ("wheel", "Wheel odometry", "#d62728"),
          ("ekf_fused", "EKF: wheels + IMU", "#1f77b4")]


def robot_marker(ax, color):
    return ax.plot([], [], marker=(3, 0, 0), ms=13, color=color, zorder=5)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--gif", default=None)
    ap.add_argument("--speed", type=float, default=8.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--title", default="Same robot, same 5 laps, icy corner top right")
    ap.add_argument("--ffmpeg", default=None)
    a = ap.parse_args()
    if a.ffmpeg:
        plt.rcParams["animation.ffmpeg_path"] = a.ffmpeg

    r = load(a.csv)
    t = r["t"]
    frames = np.arange(0.0, t[-1], a.speed / a.fps)
    idx = np.searchsorted(t, frames).clip(0, len(t) - 1)
    hold = a.fps * 2                                   # 2 s still frame at the end
    idx = np.concatenate([idx, np.full(hold, len(t) - 1)])
    err = {k: errors(r, k)[0] for k, *_ in TRACKS[1:]}
    lap = 4 * (3.0 - 1.2) + 2 * math.pi * 0.6

    plt.rcParams.update({"font.size": 13, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(7.2, 7.2), dpi=150)
    ax = fig.add_axes([0.08, 0.16, 0.88, 0.67])
    xs = np.concatenate([r[k][0] for k, *_ in TRACKS])
    ys = np.concatenate([r[k][1] for k, *_ in TRACKS])
    pad = 0.4
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect("equal")
    ax.grid(True, color="#e8e8e8", lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if "slippery" in os.path.basename(a.csv):
        ax.add_patch(Rectangle(PATCH[:2], *PATCH[2:], color="#9fd3f5", alpha=0.6, lw=0))
        ax.text(PATCH[0] + PATCH[2] / 2, PATCH[1] + PATCH[3] + 0.08, "icy patch", ha="center",
                fontsize=11, color="#2b7bb9")
    fig.text(0.08, 0.95, a.title, fontsize=15, weight="bold")
    fig.text(0.08, 0.915, "Gazebo + ROS 2 robot_localization, replayed from the logged run", fontsize=11,
             color="#555555")
    lines, marks = {}, {}
    for k, label, c in TRACKS:
        lines[k] = ax.plot([], [], color=c, lw=2.4 if k == "truth" else 1.9, label=label)[0]
        marks[k] = robot_marker(ax, c)
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=3, fontsize=10.5, frameon=False,
              handlelength=1.6, columnspacing=1.2)
    readout = fig.text(0.08, 0.05, "", fontsize=13, family="DejaVu Sans Mono")

    def draw(i):
        j = idx[i]
        for k, *_ in TRACKS:
            x, y, yaw = r[k]
            lines[k].set_data(x[:j + 1], y[:j + 1])
            marks[k].set_data([x[j]], [y[j]])
            marks[k].set_marker((3, 0, math.degrees(yaw[j]) - 90))
        d = r["dist"][j]
        readout.set_text(f"lap {min(int(d // lap) + 1, 5)}/5   {d:5.1f} m driven\n"
                         f"error   wheel odometry {err['wheel'][j]:5.2f} m   EKF + IMU {err['ekf_fused'][j]:5.2f} m")
        return list(lines.values()) + list(marks.values()) + [readout]

    writer = FFMpegWriter(fps=a.fps, codec="libx264", bitrate=6000,
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with writer.saving(fig, a.out, dpi=150):
        for i in range(len(idx)):
            draw(i)
            writer.grab_frame()
    print("video:", a.out, len(idx), "frames")

    if a.gif:
        fig.set_dpi(70)
        gw = PillowWriter(fps=12)
        step = max(1, a.fps // 12)
        with gw.saving(fig, a.gif, dpi=70):
            for i in range(0, len(idx), step):
                draw(i)
                gw.grab_frame()
        print("gif:", a.gif)


if __name__ == "__main__":
    main()

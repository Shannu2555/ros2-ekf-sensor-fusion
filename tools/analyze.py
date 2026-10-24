"""CSV logs -> metrics table + plots.

  python3 tools/analyze.py results docs/media

Every estimate is lined up with ground truth by message timestamp (interpolated onto the
ground-truth stamps), so logging delay doesn't show up as error."""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

RUNS = [("floor_exact_calib", "Normal floor, perfect calibration"), ("floor", "Normal floor"),
        ("slippery", "Icy corner"), ("slippery_exact_calib", "Icy corner, perfect calibration")]
EST = [("wheel", "Wheel odometry", "#d62728", "-"),
       ("ekf_wheel", "EKF, wheels only", "#ff9f40", "--"),
       ("ekf_fused", "EKF, wheels + IMU", "#1f77b4", "-")]
PATCH = (1.25 - 0.6, 2.75 - 0.6, 1.2, 1.2)        # slippery patch in the world (x0, y0, w, h)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def load(fn):
    d = np.genfromtxt(fn, delimiter=",", names=True)
    moving = np.nonzero(np.hypot(np.diff(d["truth_x"]), np.diff(d["truth_y"])) > 1e-4)[0]
    d = d[max(moving[0] - 10, 0):]                 # drop the standstill before the robot starts
    t = d["truth_stamp"]
    keep = np.concatenate([[True], np.diff(t) > 0])  # one row per new ground-truth message
    d, t = d[keep], t[keep]
    out = {"t": t - t[0], "truth": (d["truth_x"], d["truth_y"], d["truth_yaw"])}
    for k, *_ in EST:
        st = d[f"{k}_stamp"]
        u = np.concatenate([[True], np.diff(st) > 0])
        s = st[u]
        out[k] = (np.interp(t, s, d[f"{k}_x"][u]), np.interp(t, s, d[f"{k}_y"][u]),
                  wrap(np.interp(t, s, np.unwrap(d[f"{k}_yaw"][u]))))
    tx, ty, _ = out["truth"]
    out["dist"] = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(tx), np.diff(ty)))])
    return out


def errors(run, k):
    tx, ty, tyaw = run["truth"]
    x, y, yaw = run[k]
    return np.hypot(x - tx, y - ty), np.degrees(np.abs(wrap(yaw - tyaw)))


def metrics(runs):
    rows = []
    for name, label in RUNS:
        if name not in runs:
            continue
        r = runs[name]
        for k, est, *_ in EST:
            pe, he = errors(r, k)
            rows.append((label, est, r["dist"][-1], r["t"][-1], pe[-1], np.sqrt(np.mean(pe ** 2)), pe.max(),
                         he[-1], 100 * pe[-1] / r["dist"][-1]))
    return rows


def table(rows):
    lines = ["| Run | Estimate | Distance [m] | Final position error [m] | RMSE [m] | Max error [m] "
             "| Final heading error [deg] | Final error / distance |",
             "|---|---|---|---|---|---|---|---|"]
    for label, est, dist, _, fin, rmse, mx, hfin, pct in rows:
        lines.append(f"| {label} | {est} | {dist:.1f} | {fin:.2f} | {rmse:.2f} | {mx:.2f} | {hfin:.1f} | {pct:.1f} % |")
    return "\n".join(lines)


def style(ax):
    ax.grid(True, color="#e5e5e5", lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def plot_paths(ax, r, title, patch):
    if patch:
        ax.add_patch(Rectangle(PATCH[:2], *PATCH[2:], color="#9fd3f5", alpha=0.6, lw=0, label="icy patch"))
    ax.plot(*r["truth"][:2], color="#222222", lw=2.2, label="Ground truth")
    for k, label, c, ls in EST:
        if k == "ekf_wheel":
            continue                                   # sits right on top of the raw wheel line
        ax.plot(*r[k][:2], color=c, ls=ls, lw=1.6, label=label)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=12, loc="left")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    style(ax)


def plot_error(ax, r, title):
    for k, label, c, ls in EST:
        pe, _ = errors(r, k)
        ax.plot(r["dist"], pe, color=c, ls=ls, lw=1.6, label=label)
    ax.set_title(title, fontsize=12, loc="left")
    ax.set_xlabel("distance driven [m]")
    ax.set_ylabel("position error [m]")
    style(ax)


def main(res_dir, media_dir):
    os.makedirs(media_dir, exist_ok=True)
    runs = {n: load(os.path.join(res_dir, f"{n}.csv")) for n, _ in RUNS
            if os.path.exists(os.path.join(res_dir, f"{n}.csv"))}
    rows = metrics(runs)
    md = table(rows)
    with open(os.path.join(res_dir, "metrics.md"), "w") as f:
        f.write(md + "\n")
    print(md)

    plt.rcParams.update({"font.size": 10.5, "font.family": "DejaVu Sans"})
    for name, label in RUNS:
        if name not in runs:
            continue
        r = runs[name]
        fig, axs = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1, 1.25]})
        plot_paths(axs[0], r, f"{label}: where each estimate thinks the robot went", "slippery" in name)
        plot_error(axs[1], r, "Position error vs ground truth")
        axs[0].legend(loc="upper left", fontsize=9, frameon=False)
        axs[1].legend(loc="upper left", fontsize=9, frameon=False)
        fig.tight_layout()
        fig.savefig(os.path.join(media_dir, f"{name}.png"), dpi=150)
        plt.close(fig)

    # heading error, all runs
    fig, axs = plt.subplots(1, len(runs), figsize=(5 * len(runs), 4), sharey=True)
    for ax, (name, label) in zip(np.atleast_1d(axs), [x for x in RUNS if x[0] in runs]):
        r = runs[name]
        for k, est, c, ls in EST:
            ax.plot(r["dist"], errors(r, k)[1], color=c, ls=ls, lw=1.5, label=est)
        ax.set_title(label, fontsize=11, loc="left")
        ax.set_xlabel("distance driven [m]")
        style(ax)
    np.atleast_1d(axs)[0].set_ylabel("heading error [deg]")
    np.atleast_1d(axs)[0].legend(fontsize=9, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(media_dir, "heading_error.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "results", sys.argv[2] if len(sys.argv) > 2 else "docs/media")

"""AUV waypoint demo: moveOnPathAsync around a square with a dive, then moveToZAsync + hoverAsync.

Records the realised trajectory while the autopilot runs and writes a plot
(top view, depth, speed, heading) plus an .npz next to it.

    python waypoint_demo.py [--speed 0.6] [--side 6.0] [--dive 1.5] [--out DIR]
"""
try:
    import setup_path
except ImportError:
    pass
import furosim

import argparse
import os
import threading
import time

import numpy as np


def sample_state(client):
    s = client.getAuvState()
    k = s.kinematics_estimated
    p = k.position
    v = k.linear_velocity
    _, _, yaw = furosim.to_eularian_angles(k.orientation)
    return [s.timestamp * 1e-9, p.x_val, p.y_val, p.z_val, v.x_val, v.y_val, v.z_val, yaw]


class Recorder:
    """Samples the AUV state on its own RPC connection (msgpackrpc clients are not thread-safe)."""

    def __init__(self, main_client, rate_hz=20.0):
        self.main_client = main_client
        self.rec_client = furosim.AuvClient()
        self.rec_client.confirmConnection()
        self.period = 1.0 / rate_hz
        self.rows = []
        self.marks = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            self.rows.append(sample_state(self.rec_client))
            time.sleep(self.period)

    def start(self):
        self._thread.start()

    def mark(self, label):
        self.marks.append((sample_state(self.main_client)[0], label))

    def stop(self):
        self._stop.set()
        self._thread.join()
        return np.array(self.rows)


def min_distance_to(traj_xyz, point):
    return float(np.min(np.linalg.norm(traj_xyz - point, axis=1)))


def plot(traj, waypoints, marks, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = traj[:, 0] - traj[0, 0]
    x, y, z = traj[:, 1], traj[:, 2], traj[:, 3]
    speed = np.linalg.norm(traj[:, 4:7], axis=1)
    yaw_deg = np.degrees(traj[:, 7])
    wp = np.array(waypoints)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax = axes[0, 0]
    ax.plot(y, x, "-", lw=1.5, label="AUV")
    ax.plot(wp[:, 1], wp[:, 0], "ro", ms=8, label="waypoints")
    for i, w in enumerate(wp):
        ax.annotate(f"WP{i+1}", (w[1], w[0]), textcoords="offset points", xytext=(6, 6))
    ax.plot(y[0], x[0], "gs", ms=8, label="start")
    ax.set_xlabel("East y [m]")
    ax.set_ylabel("North x [m]")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_title("Top view (NED)")

    ax = axes[0, 1]
    ax.plot(t, z, lw=1.5)
    for w in wp:
        ax.axhline(w[2], color="r", ls=":", lw=0.8)
    ax.invert_yaxis()
    ax.set_xlabel("t [s]")
    ax.set_ylabel("z (down) [m]")
    ax.grid(True)
    ax.set_title("Depth (dotted = waypoint z)")

    ax = axes[1, 0]
    ax.plot(t, speed, lw=1.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|v| [m/s]")
    ax.grid(True)
    ax.set_title("Speed")

    ax = axes[1, 1]
    ax.plot(t, yaw_deg, lw=1.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("yaw [deg]")
    ax.grid(True)
    ax.set_title("Heading")

    t0 = traj[0, 0]
    for ts, label in marks:
        for ax in (axes[0, 1], axes[1, 0], axes[1, 1]):
            ax.axvline(ts - t0, color="k", ls="--", lw=0.8)
        axes[0, 1].annotate(label, (ts - t0, np.min(z)), rotation=90, fontsize=8, va="bottom")

    fig.suptitle("FURoSim AUV waypoint demo: moveOnPathAsync -> moveToZAsync -> hoverAsync")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"plot saved: {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.6, help="surge speed cap [m/s]")
    ap.add_argument("--side", type=float, default=6.0, help="square side length [m]")
    ap.add_argument("--dive", type=float, default=1.5, help="extra depth on the far side [m]")
    ap.add_argument("--timeout", type=float, default=180.0, help="per-command timeout [s]")
    ap.add_argument("--out", default="auv_waypoint_demo_out")
    ap.add_argument("--release", action="store_true",
                    help="hand control back (enableApiControl False) at the end; default keeps station keeping")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    client = furosim.AuvClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.hoverAsync().get()
    time.sleep(3.0)

    s0 = sample_state(client)
    x0, y0, z0 = s0[1], s0[2], s0[3]
    L, D = args.side, args.dive
    waypoints = [
        (x0 + L, y0,     z0),
        (x0 + L, y0 + L, z0 + D),
        (x0,     y0 + L, z0 + D),
        (x0,     y0,     z0),
    ]
    path = [furosim.Vector3r(*w) for w in waypoints]
    print(f"start NED = ({x0:.2f}, {y0:.2f}, {z0:.2f}); {len(path)} waypoints, speed {args.speed} m/s")

    rec = Recorder(client)
    rec.start()

    rec.mark("moveOnPath")
    t_start = time.time()
    ok_path = client.moveOnPathAsync(path, args.speed, timeout_sec=args.timeout).get()
    print(f"moveOnPathAsync -> {ok_path} in {time.time() - t_start:.1f} s wall")

    rec.mark("moveToZ")
    z_target = z0 + 1.0
    t_start = time.time()
    ok_z = client.moveToZAsync(z_target, 0.3, timeout_sec=args.timeout).get()
    z_arrival_err = abs(sample_state(client)[3] - z_target)
    print(f"moveToZAsync({z_target:.2f}) -> {ok_z} in {time.time() - t_start:.1f} s wall, "
          f"z error at return {z_arrival_err:.3f} m")
    time.sleep(5.0)
    z_settled_err = abs(sample_state(client)[3] - z_target)
    print(f"z error after 5 s station keeping at the target: {z_settled_err:.3f} m")

    rec.mark("hover")
    client.hoverAsync().get()
    time.sleep(6.0)

    traj = rec.stop()
    if args.release:
        client.enableApiControl(False)
    else:
        print("station keeping continues (API control left enabled); pass --release to hand control back")

    xyz = traj[:, 1:4]
    print("closest approach to each waypoint [m]:")
    for i, w in enumerate(waypoints):
        print(f"  WP{i+1} {np.round(w, 2)} : {min_distance_to(xyz, np.array(w)):.3f}")
    hover_seg = traj[traj[:, 0] >= rec.marks[-1][0] + 2.0]
    if len(hover_seg):
        drift = np.linalg.norm(hover_seg[:, 1:4] - hover_seg[0, 1:4], axis=1)
        print(f"hover drift over last {hover_seg[-1,0]-hover_seg[0,0]:.1f} s: max {drift.max():.3f} m")

    np.savez(os.path.join(args.out, "trajectory.npz"), traj=traj, waypoints=np.array(waypoints),
             marks=np.array([(m[0], m[1]) for m in rec.marks], dtype=object))
    try:
        plot(traj, waypoints, rec.marks, os.path.join(args.out, "auv_waypoint_demo.png"))
    except ImportError as e:
        print(f"plot skipped ({e}); trajectory.npz saved")


if __name__ == "__main__":
    main()

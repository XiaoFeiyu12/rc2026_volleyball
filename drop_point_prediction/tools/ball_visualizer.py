#!/usr/bin/env python3
"""
Ball Detector vs Tracker + Predictor Real-time Visualization Dashboard

Subscribes:
  - /detector/ball          (camera_color_optical_frame, TF->odom)
  - /tracker/target         (odom frame, filtered position + velocity)
  - /predictor/ball_trajectory (odom frame, future trajectory)

Usage:
    python tools/ball_visualizer.py                     # default 30s window
    python tools/ball_visualizer.py --window 60.0       # 60s window
    python tools/ball_visualizer.py --no-gui            # log only, no GUI
    python tools/ball_visualizer.py --log-dir /tmp/logs
"""

import argparse
import csv
import os
import signal
import sys
import threading
from collections import deque
from datetime import datetime

import numpy as np

# ── matplotlib ──
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ── ROS2 ──
import rclpy
from rclpy.node import Node
from volleyball_interfaces.msg import Ball, BallTrajectory
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401 — register PointStamped transform

# ──────────────────────────────────────────────────────────────────────
#  ROS2 Node
# ──────────────────────────────────────────────────────────────────────


class BallVisualizerNode(Node):
    """Subscribe to detector / tracker / predictor, maintain ring buffers + CSV log"""

    def __init__(self, window_sec: float = 30.0, log_dir: str = ""):
        super().__init__("ball_visualizer")

        self.maxlen = max(int(window_sec * 100), 600)
        self.window_sec = window_sec

        # ── ring buffers ──
        self.times: deque[float] = deque(maxlen=self.maxlen)
        self.det_x: deque[float] = deque(maxlen=self.maxlen)
        self.det_y: deque[float] = deque(maxlen=self.maxlen)
        self.det_z: deque[float] = deque(maxlen=self.maxlen)
        self.trk_x: deque[float] = deque(maxlen=self.maxlen)
        self.trk_y: deque[float] = deque(maxlen=self.maxlen)
        self.trk_z: deque[float] = deque(maxlen=self.maxlen)
        self.trk_vx: deque[float] = deque(maxlen=self.maxlen)
        self.trk_vy: deque[float] = deque(maxlen=self.maxlen)
        self.trk_vz: deque[float] = deque(maxlen=self.maxlen)

        # last detector measurement (detector rate < tracker rate, hold latest)
        self._last_det_x: float | None = None
        self._last_det_y: float | None = None
        self._last_det_z: float | None = None

        # latest predicted trajectory (list of (x,y,z) in odom frame)
        self._traj_x: list[float] = []
        self._traj_y: list[float] = []
        self._traj_z: list[float] = []
        self._traj_t: float = 0.0  # timestamp of latest trajectory

        self.lock = threading.Lock()

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── subscriptions (SensorDataQoS to match publishers) ──
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self.det_sub = self.create_subscription(
            Ball, "/detector/ball", self.det_callback, sensor_qos
        )
        self.trk_sub = self.create_subscription(
            Ball, "/tracker/target", self.trk_callback, sensor_qos
        )
        self.pred_sub = self.create_subscription(
            BallTrajectory, "/predictor/ball_trajectory", self.pred_callback, sensor_qos
        )

        # ── CSV log ──
        if not log_dir:
            log_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "bag"
            )
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, f"ball_viz_{ts}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "det_x_odom", "det_y_odom", "det_z_odom",
            "trk_x", "trk_y", "trk_z", "trk_vx", "trk_vy", "trk_vz",
        ])
        self.get_logger().info(f"CSV log: {self.csv_path}")

    # ── callbacks ──────────────────────────────────────────────────────

    def det_callback(self, msg: Ball) -> None:
        """Detector -> transform camera_color_optical_frame -> odom"""
        try:
            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.point.x = float(msg.x)
            ps.point.y = float(msg.y)
            ps.point.z = float(msg.z)
            ps_odom = self.tf_buffer.transform(
                ps, "odom", timeout=rclpy.duration.Duration(seconds=0.05)
            )
            with self.lock:
                self._last_det_x = ps_odom.point.x
                self._last_det_y = ps_odom.point.y
                self._last_det_z = ps_odom.point.z
        except TransformException as e:
            self.get_logger().debug(f"TF failed (det->odom): {e}", throttle_duration_sec=5.0)

    def trk_callback(self, msg: Ball) -> None:
        """Tracker -> odom frame, record position + velocity"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.times.append(t)
            self.trk_x.append(float(msg.x))
            self.trk_y.append(float(msg.y))
            self.trk_z.append(float(msg.z))
            self.trk_vx.append(float(msg.vx))
            self.trk_vy.append(float(msg.vy))
            self.trk_vz.append(float(msg.vz))
            if self._last_det_x is not None:
                self.det_x.append(self._last_det_x)
                self.det_y.append(self._last_det_y)
                self.det_z.append(self._last_det_z)
            else:
                self.det_x.append(float("nan"))
                self.det_y.append(float("nan"))
                self.det_z.append(float("nan"))
        self.csv_writer.writerow([
            f"{t:.6f}",
            f"{self._last_det_x:.4f}" if self._last_det_x is not None else "",
            f"{self._last_det_y:.4f}" if self._last_det_y is not None else "",
            f"{self._last_det_z:.4f}" if self._last_det_z is not None else "",
            f"{msg.x:.4f}", f"{msg.y:.4f}", f"{msg.z:.4f}",
            f"{msg.vx:.4f}", f"{msg.vy:.4f}", f"{msg.vz:.4f}",
        ])

    def pred_callback(self, msg: BallTrajectory) -> None:
        """Predictor -> store latest trajectory (list of future positions in odom)"""
        if not msg.ball_trajectory:
            return
        xs, ys, zs = [], [], []
        for ball in msg.ball_trajectory:
            xs.append(float(ball.x))
            ys.append(float(ball.y))
            zs.append(float(ball.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self._traj_x = xs
            self._traj_y = ys
            self._traj_z = zs
            self._traj_t = t

    def get_data(self) -> dict:
        """Thread-safe snapshot of all buffers"""
        with self.lock:
            return {
                "times": np.array(self.times, dtype=float),
                "det_x": np.array(self.det_x, dtype=float),
                "det_y": np.array(self.det_y, dtype=float),
                "det_z": np.array(self.det_z, dtype=float),
                "trk_x": np.array(self.trk_x, dtype=float),
                "trk_y": np.array(self.trk_y, dtype=float),
                "trk_z": np.array(self.trk_z, dtype=float),
                "trk_vx": np.array(self.trk_vx, dtype=float),
                "trk_vy": np.array(self.trk_vy, dtype=float),
                "trk_vz": np.array(self.trk_vz, dtype=float),
                "traj_x": list(self._traj_x),
                "traj_y": list(self._traj_y),
                "traj_z": list(self._traj_z),
                "traj_t": self._traj_t,
            }

    def destroy_node(self) -> None:
        self.csv_file.close()
        self.get_logger().info(f"Log saved: {self.csv_path}")
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────
#  matplotlib Dashboard
# ──────────────────────────────────────────────────────────────────────


class Dashboard:
    """Real-time dashboard: position comparison / velocity / predicted trajectory"""

    COLOR_DET = "#FF6B6B"
    COLOR_TRK = "#4ECDC4"
    COLOR_PRED = "#FFD93D"
    COLOR_PAST = "#888888"

    def __init__(self, node: BallVisualizerNode, window_sec: float = 30.0):
        self.node = node
        self.window_sec = window_sec

        plt.style.use("dark_background")

        # ── main dashboard figure ──
        self.fig = plt.figure("Ball Dashboard", figsize=(17, 10))
        self.fig.patch.set_facecolor("#1a1a2e")
        self.fig.subplots_adjust(left=0.05, right=0.98, top=0.94, bottom=0.04)
        self.fig.suptitle(
            "Volleyball Ball — Detector / Tracker / Predictor  (odom frame)",
            fontsize=14, fontweight="bold", color="white",
        )

        # Layout: 4 rows x 3 cols.
        # Row 1: X Position (det+trk+pred) | Y Position | Z Position
        # Row 2: Vx (tracker)              | Vy         | Vz
        # Row 3: Residual X (det-trk)      | Residual Y | Residual Z
        # Row 4: Status bar (span 3)
        gs = self.fig.add_gridspec(4, 3, hspace=0.60, wspace=0.40,
                                   height_ratios=[2.5, 1.8, 1.8, 0.7])

        self.ax_pos_x = self.fig.add_subplot(gs[0, 0])
        self.ax_pos_y = self.fig.add_subplot(gs[0, 1])
        self.ax_pos_z = self.fig.add_subplot(gs[0, 2])
        self.ax_vel_x = self.fig.add_subplot(gs[1, 0])
        self.ax_vel_y = self.fig.add_subplot(gs[1, 1])
        self.ax_vel_z = self.fig.add_subplot(gs[1, 2])
        self.ax_res_x = self.fig.add_subplot(gs[2, 0])
        self.ax_res_y = self.fig.add_subplot(gs[2, 1])
        self.ax_res_z = self.fig.add_subplot(gs[2, 2])
        self.ax_stats = self.fig.add_subplot(gs[3, :])
        self.ax_stats.axis("off")

        self._setup_position_axes()
        self._setup_velocity_axes()
        self._setup_residual_axes()

        # ── separate zoomable trajectory window ──
        self.traj_fig = plt.figure("Trajectory View (zoomable)", figsize=(12, 6))
        self.traj_fig.patch.set_facecolor("#1a1a2e")
        gs2 = self.traj_fig.add_gridspec(1, 2, wspace=0.35)
        self.ax_traj_big_xz = self.traj_fig.add_subplot(gs2[0, 0])
        self.ax_traj_big_xy = self.traj_fig.add_subplot(gs2[0, 1])
        self._setup_big_trajectory_axes()

        # ── texts ──
        self.stats_text = self.ax_stats.text(
            0.5, 0.5, "", transform=self.ax_stats.transAxes,
            fontsize=9, fontfamily="monospace", ha="center", va="center", color="white",
        )

        # ── animation ──
        self.ani = FuncAnimation(
            self.fig, self._update, interval=100, blit=False, cache_frame_data=False,
        )
        # Maximize main window
        self._maximize_window(self.fig)
        self._maximize_window(self.traj_fig)

    # ── setup ───────────────────────────────────────────────────────────

    def _setup_position_axes(self) -> None:
        self._pos_lines = []
        for ax, title, ylabel in [
            (self.ax_pos_x, "X Position", "X (m)"),
            (self.ax_pos_y, "Y Position", "Y (m)"),
            (self.ax_pos_z, "Z Position", "Z (m)"),
        ]:
            ax.set_title(title, fontsize=11, color="white")
            ax.set_ylabel(ylabel, fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.set_facecolor("#16213e")
            (l_det,) = ax.plot([], [], color=self.COLOR_DET, lw=1.5, label="Detector")
            (l_trk,) = ax.plot([], [], color=self.COLOR_TRK, lw=1.5, label="Tracker")
            (l_pred,) = ax.plot([], [], color=self.COLOR_PRED, lw=1.0, ls="--", label="Predicted")
            ax.legend(loc="upper right", fontsize=6, framealpha=0.5)
            self._pos_lines.extend([l_det, l_trk, l_pred])

    def _setup_velocity_axes(self) -> None:
        self._vel_lines = []
        for ax, title in [
            (self.ax_vel_x, "Velocity Vx"),
            (self.ax_vel_y, "Velocity Vy"),
            (self.ax_vel_z, "Velocity Vz"),
        ]:
            ax.set_title(title, fontsize=11, color="white")
            ax.set_ylabel("m/s", fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.set_facecolor("#16213e")
            ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
            (l,) = ax.plot([], [], color="#FFD93D", lw=1.5)
            ax.legend([l], ["Tracker"], loc="upper right", fontsize=7, framealpha=0.5)
            self._vel_lines.append(l)

    def _setup_residual_axes(self) -> None:
        """Residual = detector - tracker, shows EKF smoothing amount"""
        self._res_lines = []
        COLORS_RES = ["#FF6B6B", "#FFD93D", "#6BCB77"]
        for ax, title, color in [
            (self.ax_res_x, "Residual X (det-trk)", COLORS_RES[0]),
            (self.ax_res_y, "Residual Y (det-trk)", COLORS_RES[1]),
            (self.ax_res_z, "Residual Z (det-trk)", COLORS_RES[2]),
        ]:
            ax.set_title(title, fontsize=11, color="white")
            ax.set_ylabel("delta (m)", fontsize=9)
            # xlabel omitted to avoid overlap with row below
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.set_facecolor("#16213e")
            ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
            (line,) = ax.plot([], [], color=color, linewidth=1.5)
            self._res_lines.append(line)

    def _setup_big_trajectory_axes(self) -> None:
        """Separate zoomable figure: large XZ + XY trajectory views"""
        for ax, title, xlabel, ylabel in [
            (self.ax_traj_big_xz, "Trajectory XZ (side view) — Past + Predicted", "X (m)", "Z (m)"),
            (self.ax_traj_big_xy, "Trajectory XY (top view) — Past + Predicted", "X (m)", "Y (m)"),
        ]:
            ax.set_title(title, fontsize=12, color="white")
            ax.set_xlabel(xlabel, fontsize=9)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.set_facecolor("#16213e")
            ax.set_aspect("equal", adjustable="datalim")

        # XZ view
        (self._big_xz_past,) = self.ax_traj_big_xz.plot(
            [], [], color=self.COLOR_PAST, lw=1.0, alpha=0.6, label="Past (actual tracker)")
        (self._big_xz_pred,) = self.ax_traj_big_xz.plot(
            [], [], color=self.COLOR_PRED, lw=2.5, label="Predicted (future)")
        (self._big_xz_cur,) = self.ax_traj_big_xz.plot(
            [], [], "o", color=self.COLOR_TRK, ms=8, label="Now")
        # landing marker
        (self._big_xz_land,) = self.ax_traj_big_xz.plot(
            [], [], "s", color="#FF6B6B", ms=10, label="Landing")
        self.ax_traj_big_xz.legend(loc="upper right", fontsize=8, framealpha=0.5)

        # XY view
        (self._big_xy_past,) = self.ax_traj_big_xy.plot(
            [], [], color=self.COLOR_PAST, lw=1.0, alpha=0.6, label="Past (actual tracker)")
        (self._big_xy_pred,) = self.ax_traj_big_xy.plot(
            [], [], color=self.COLOR_PRED, lw=2.5, label="Predicted (future)")
        (self._big_xy_cur,) = self.ax_traj_big_xy.plot(
            [], [], "o", color=self.COLOR_TRK, ms=8, label="Now")
        (self._big_xy_land,) = self.ax_traj_big_xy.plot(
            [], [], "s", color="#FF6B6B", ms=10, label="Landing")
        self.ax_traj_big_xy.legend(loc="upper right", fontsize=8, framealpha=0.5)

        self.traj_fig.canvas.mpl_connect("close_event", lambda e: self._on_traj_close())

    def _on_traj_close(self) -> None:
        """Re-open trajectory window if user closes it"""
        pass  # window can be re-shown but we keep it simple

    @staticmethod
    def _maximize_window(fig: plt.Figure) -> None:
        """Maximize figure window (TkAgg on Linux)"""
        try:
            win = fig.canvas.manager.window
            try:
                win.wm_attributes("-zoomed", True)   # Tk on Linux
            except Exception:
                try:
                    win.state("zoomed")               # Tk on Windows
                except Exception:
                    win.geometry("1400x900+0+0")      # fallback: large size
        except Exception:
            pass

    # ── update ──────────────────────────────────────────────────────────

    def _update(self, frame: int) -> list:
        data = self.node.get_data()
        times = data["times"]
        if len(times) < 2:
            return []

        t_ref = times[-1]
        t_rel = times - t_ref
        mask = t_rel >= -self.window_sec
        t_plot = t_rel[mask]

        # ── position comparison (det + trk + pred overlay) ──
        pos_axes = [
            (self.ax_pos_x, self._pos_lines[0], self._pos_lines[1], self._pos_lines[2],
             data["det_x"], data["trk_x"]),
            (self.ax_pos_y, self._pos_lines[3], self._pos_lines[4], self._pos_lines[5],
             data["det_y"], data["trk_y"]),
            (self.ax_pos_z, self._pos_lines[6], self._pos_lines[7], self._pos_lines[8],
             data["det_z"], data["trk_z"]),
        ]
        for ax, l_det, l_trk, l_pred, det_arr, trk_arr in pos_axes:
            valid = ~np.isnan(det_arr[mask])
            l_det.set_data(t_plot[valid], det_arr[mask][valid]) if valid.any() else l_det.set_data([], [])
            l_trk.set_data(t_plot, trk_arr[mask])
            # predicted future: dashed line extending from current tracker position
            traj_x_arr = np.array(data["traj_x"])
            if len(traj_x_arr) > 1 and len(trk_arr[mask]) > 0:
                # which axis? detect by comparing references
                if ax is self.ax_pos_x:
                    pred_arr = traj_x_arr
                elif ax is self.ax_pos_y:
                    pred_arr = np.array(data["traj_y"])
                else:
                    pred_arr = np.array(data["traj_z"])
                # time axis: start at t=0, step according to predict_step
                dt = 0.02  # approximate predict step
                pred_t = np.arange(len(pred_arr)) * dt
                l_pred.set_data(pred_t, pred_arr)
            else:
                l_pred.set_data([], [])
            ax.set_xlim(-self.window_sec, max(2, len(data["traj_x"]) * 0.02 + 0.5))
            all_for_ylim = [det_arr[mask][valid] if valid.any() else None, trk_arr[mask]]
            self._auto_ylim(ax, all_for_ylim)

        # ── velocity ──
        vel_pairs = [
            (self._vel_lines[0], data["trk_vx"], self.ax_vel_x),
            (self._vel_lines[1], data["trk_vy"], self.ax_vel_y),
            (self._vel_lines[2], data["trk_vz"], self.ax_vel_z),
        ]
        for line, arr, ax in vel_pairs:
            line.set_data(t_plot, arr[mask])
            ax.set_xlim(-self.window_sec, 2)
            self._auto_ylim(ax, [arr[mask]])

        # ── residual (det - trk) ──
        res_pairs = [
            (self._res_lines[0], data["det_x"], data["trk_x"], self.ax_res_x),
            (self._res_lines[1], data["det_y"], data["trk_y"], self.ax_res_y),
            (self._res_lines[2], data["det_z"], data["trk_z"], self.ax_res_z),
        ]
        for line, det_arr, trk_arr, ax in res_pairs:
            diff = det_arr - trk_arr
            valid = ~np.isnan(diff[mask])
            if valid.any():
                line.set_data(t_plot[valid], diff[mask][valid])
                ax.set_ylim(np.nanmin(diff[mask][valid]) - 0.1, np.nanmax(diff[mask][valid]) + 0.1)
            else:
                line.set_data([], [])
            ax.set_xlim(-self.window_sec, 2)

        # ── update big trajectory window ──
        self._update_trajectory_views(data, mask)

        # ── status bar ──
        self._update_stats(data, mask)

        artists = (list(self._pos_lines) + list(self._vel_lines) + list(self._res_lines)
                   + [self.stats_text])
        return artists

    def _update_trajectory_views(self, data: dict, mask: np.ndarray) -> None:
        """Update the big zoomable trajectory window"""
        if not plt.fignum_exists(self.traj_fig.number):
            return

        trk_x = data["trk_x"][mask]
        trk_y = data["trk_y"][mask]
        trk_z = data["trk_z"][mask]
        traj_x = np.array(data["traj_x"])
        traj_y = np.array(data["traj_y"])
        traj_z = np.array(data["traj_z"])

        has_past = len(trk_x) > 1
        has_pred = len(traj_x) > 1

        # XZ big
        if has_past:
            self._big_xz_past.set_data(trk_x, trk_z)
            self._big_xz_cur.set_data([trk_x[-1]], [trk_z[-1]])
        else:
            self._big_xz_past.set_data([], [])
            self._big_xz_cur.set_data([], [])
        if has_pred:
            self._big_xz_pred.set_data(traj_x, traj_z)
            self._big_xz_land.set_data([traj_x[-1]], [traj_z[-1]])
            x_all = np.concatenate([trk_x[-50:], traj_x]) if has_past else traj_x
            z_all = np.concatenate([trk_z[-50:], traj_z]) if has_past else traj_z
            self._auto_ylim_spatial(self.ax_traj_big_xz, x_all, z_all, pad=0.12)
        elif has_past:
            self._big_xz_pred.set_data([], [])
            self._big_xz_land.set_data([], [])
            self._auto_ylim_spatial(self.ax_traj_big_xz, trk_x, trk_z, pad=0.12)

        # XY big
        if has_past:
            self._big_xy_past.set_data(trk_x, trk_y)
            self._big_xy_cur.set_data([trk_x[-1]], [trk_y[-1]])
        else:
            self._big_xy_past.set_data([], [])
            self._big_xy_cur.set_data([], [])
        if has_pred:
            self._big_xy_pred.set_data(traj_x, traj_y)
            self._big_xy_land.set_data([traj_x[-1]], [traj_y[-1]])
            x_all = np.concatenate([trk_x[-50:], traj_x]) if has_past else traj_x
            y_all = np.concatenate([trk_y[-50:], traj_y]) if has_past else traj_y
            self._auto_ylim_spatial(self.ax_traj_big_xy, x_all, y_all, pad=0.12)
        elif has_past:
            self._big_xy_pred.set_data([], [])
            self._big_xy_land.set_data([], [])
            self._auto_ylim_spatial(self.ax_traj_big_xy, trk_x, trk_y, pad=0.12)

        self.traj_fig.canvas.draw_idle()

    def _update_stats(self, data: dict, mask: np.ndarray) -> None:
        """Bottom status bar"""
        trk_x = data["trk_x"][mask]
        trk_y = data["trk_y"][mask]
        trk_z = data["trk_z"][mask]
        det_x = data["det_x"][mask]
        trk_vx = data["trk_vx"][mask]
        trk_vy = data["trk_vy"][mask]
        trk_vz = data["trk_vz"][mask]

        valid_det = ~np.isnan(det_x)
        has_det = valid_det.any()
        has_trk = len(trk_x) > 0
        has_pred = len(data["traj_x"]) > 1
        has_vel = has_trk and (np.any(np.abs(trk_vx) > 1e-6)
                               or np.any(np.abs(trk_vy) > 1e-6)
                               or np.any(np.abs(trk_vz) > 1e-6))

        det_s = "* DETECTOR" if has_det else "- detector (no data)"
        trk_s = "* TRACKER" if has_trk else "- tracker (no data)"
        prd_s = "* PREDICT" if has_pred else "- predict (no data)"
        vel_s = "* VEL" if has_vel else "- vel"

        if not has_trk and not has_det:
            self.stats_text.set_text(
                "Waiting for data...\nMake sure detector + tracker + predictor are running")
            return

        cur_det_x = det_x[-1] if valid_det[-1] else float("nan")
        cur_trk_x = trk_x[-1] if has_trk else float("nan")
        cur_trk_y = trk_y[-1] if has_trk else float("nan")
        cur_trk_z = trk_z[-1] if has_trk else float("nan")
        speed = np.sqrt(trk_vx[-1]**2 + trk_vy[-1]**2 + trk_vz[-1]**2) if has_trk else float("nan")

        text = (
            f"Sources: {det_s}  {trk_s}  {prd_s}  {vel_s}\n"
            f"Tracker: x={cur_trk_x:.3f} y={cur_trk_y:.3f} z={cur_trk_z:.3f} m  |  "
            f"|V|={speed:.3f} m/s  (vx={trk_vx[-1]:.3f} vy={trk_vy[-1]:.3f} vz={trk_vz[-1]:.3f})\n"
            f"Detector (raw): x={cur_det_x:.3f} m"
        )
        self.stats_text.set_text(text)

    # ── helpers ─────────────────────────────────────────────────────────

    @staticmethod
    def _auto_ylim(ax: plt.Axes, arrays: list, pad: float = 0.2) -> None:
        vals = []
        for arr in arrays:
            if arr is not None and len(arr) > 0:
                v = arr[~np.isnan(arr)]
                if len(v) > 0:
                    vals.extend(v.tolist())
        if vals:
            ymin, ymax = min(vals), max(vals)
            dy = max(ymax - ymin, 0.5)
            ax.set_ylim(ymin - dy * pad, ymax + dy * pad)

    @staticmethod
    def _auto_ylim_spatial(ax: plt.Axes, x_arr: np.ndarray, y_arr: np.ndarray, pad: float = 0.15) -> None:
        """Auto-scale both X and Y for spatial plots, keeping aspect ratio"""
        if len(x_arr) == 0 or len(y_arr) == 0:
            return
        xmin, xmax = np.min(x_arr), np.max(x_arr)
        ymin, ymax = np.min(y_arr), np.max(y_arr)
        dx = max(xmax - xmin, 0.5)
        dy = max(ymax - ymin, 0.5)
        ax.set_xlim(xmin - dx * pad, xmax + dx * pad)
        ax.set_ylim(ymin - dy * pad, ymax + dy * pad)


# ──────────────────────────────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────────────────────────────


def ros_spin(node: Node) -> None:
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Ball Detector vs Tracker + Predictor real-time dashboard"
    )
    parser.add_argument("--window", type=float, default=30.0,
                        help="Time window in seconds (default: 30)")
    parser.add_argument("--no-gui", action="store_true", help="Log CSV only, no GUI")
    parser.add_argument("--log-dir", type=str, default="",
                        help="CSV output directory (default: VolleyballRobot/bag/)")
    args = parser.parse_args()

    rclpy.init()
    node = BallVisualizerNode(window_sec=args.window, log_dir=args.log_dir)

    if args.no_gui:
        print(f"Headless mode: logging to CSV -> {node.csv_path}")
        print("Press Ctrl+C to stop...")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    print(f"CSV log: {node.csv_path}")
    print("Close window to exit...")

    dashboard = Dashboard(node, window_sec=args.window)

    def on_close(event=None):
        print("\nShutting down...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    dashboard.fig.canvas.mpl_connect("close_event", on_close)
    signal.signal(signal.SIGINT, lambda sig, frame: on_close())
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

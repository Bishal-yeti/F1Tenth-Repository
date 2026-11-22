#!/usr/bin/env python3
"""
ROS 2 Node: Real-Time LiDAR Dynamic Racing Controller (Version 2: Adaptive Pace & Time-Gain Edition)

Course Deliverables Implemented:
1. 10 Consecutive Laps Without Collisions (Challenge 1)
2. Progressive Lap-Over-Lap Pace Evolution & Time-Gain Tracking (Challenge 2)
3. Automatic Lap Counter with Debounced Hysteresis (2 points)
4. Lap Timer with Live Ticker & Simulation-Time Resolution (3 points)
5. RViz2 Floating 3D HUD & ASCII Telemetry Summary for Video Evidence (2 points)
6. Automatic Vehicle Halt & CSV Time-Gain Logging after Lap 10
"""

import sys
import os
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from controllers.lidar_race_core import RealLidarRaceCore, DEFAULTS_V2, PACE_TRIMS
from controllers.telemetry_gui import RaceTelemetryGUI


class LidarRaceNode(Node):
    def __init__(self):
        super().__init__('lidar_race_node')

        # Declare configurable ROS 2 parameters
        for key, default_val in DEFAULTS_V2.items():
            self.declare_parameter(key, default_val)

        self.declare_parameter('target_laps', 10)
        self.declare_parameter('stop_after_target_laps', True)
        self.declare_parameter('auto_reset_on_start', True)
        self.declare_parameter('adaptive_pace', True)
        self.declare_parameter('csv_log_path', 'lap_times_sakhir_v2_fast.csv')
        self.declare_parameter('gains_csv_path', 'lap_time_gains_sakhir.csv')

        core_params = {}
        for key, default_val in DEFAULTS_V2.items():
            val = self.get_parameter(key).value
            if isinstance(default_val, int):
                core_params[key] = int(val)
            elif isinstance(default_val, float):
                core_params[key] = float(val)
            else:
                core_params[key] = str(val)

        adaptive_pace = bool(self.get_parameter('adaptive_pace').value)
        self.core = RealLidarRaceCore(adaptive_pace=adaptive_pace, **core_params)
        self.target_laps = int(self.get_parameter('target_laps').value)
        self.stop_after_target_laps = bool(self.get_parameter('stop_after_target_laps').value)
        self.auto_reset_on_start = bool(self.get_parameter('auto_reset_on_start').value)
        self.csv_log_path = str(self.get_parameter('csv_log_path').value)
        self.gains_csv_path = str(self.get_parameter('gains_csv_path').value)

        self.prev_steer = 0.0
        self.current_vel = 0.0
        self.current_speed_cmd = 0.0

        # Lap counter, timer & time-gain state
        self.start_pose = None
        self.current_pose = (0.0, 0.0)
        self.prev_pose = None
        self.current_lap = 0
        self.race_start_time = None
        self.lap_start_time = None
        self.lap_times = []
        self.lap_distances = []
        self.lap_max_speeds = []
        self.lap_min_clearances = []
        self.lap_trims = []
        self.lap_gains_vs_lap1 = []
        self.lap_gains_vs_prev = []
        self.best_lap_time = float('inf')
        self.left_start_zone = False
        self.distance_traveled_this_lap = 0.0
        self.total_distance = 0.0
        self.race_finished = False
        self.stop_packets_sent = 0

        self.min_wall_clearance_this_lap = float('inf')
        self.peak_velocity_this_lap = 0.0
        self.total_collisions = 0
        self.last_timer_print_sim_time = 0.0
        self.last_hud_pub_sim_time = 0.0
        self.trajectory_points = []
        self.last_traj_pose = None
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Launch Live Graphical Telemetry & Lap Timer Window (background daemon thread)
        self.gui = RaceTelemetryGUI(
            title="F1Tenth Sakhir GP — Live Speedometer & Lap Timer Window (LiDAR v2)",
            target_laps=self.target_laps,
        )

        # ROS 2 Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.lap_count_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.lap_time_pub = self.create_publisher(Float32, '/lap_time', 10)
        self.best_lap_pub = self.create_publisher(Float32, '/best_lap_time', 10)
        self.telemetry_pub = self.create_publisher(String, '/race_telemetry', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lap_hud_markers', 10)
        self.hud_marker_pub = self.create_publisher(Marker, '/ftg_hud_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/ftg_target_marker', 10)
        self.traj_marker_pub = self.create_publisher(Marker, '/ftg_trajectory_marker', 10)
        self.path_pub = self.create_publisher(Path, '/ftg_path', 10)

        # ROS 2 Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Connection status tracking & friendly reminder
        self.received_first_scan = False
        self.received_first_odom = False
        self.connection_check_timer = self.create_timer(2.5, self._check_simulator_connection)

        self._print_startup_banner()

    def _check_simulator_connection(self):
        if not self.received_first_scan:
            self.get_logger().warn(
                "\n" + "=" * 76 + "\n"
                "  ⏳ WAITING FOR SIMULATOR TELEMETRY (/scan)...\n"
                "  ⚠️  The simulator is not yet publishing sensor data.\n"
                "  👉 Make sure the simulator is running! In another terminal, run:\n"
                "        ./run_race.sh sim\n"
                "     (Or launch both Sim + AI in 1 command: ./run_race.sh race)\n"
                + "=" * 76
            )
        else:
            self.connection_check_timer.cancel()

    def _print_startup_banner(self):
        p = self.core.p
        msg = f"""
==============================================================================
  REAL-TIME LIDAR DYNAMIC RACING CONTROLLER (V2: 10-STAGE TIME-ATTACK EDITION)
==============================================================================
  Perception Mode : Pure Real-Time LiDAR ({p['fov_angle_deg']}° FOV + Straightaway DRS)
  Adaptive Trims  : T0 (16.2 m/s, 4.20 m/s) -> T9 (18.85 m/s, 6.25 m/s)
  Safety Bubble   : {p['bubble_radius']:.2f} m | Symmetrical Chicane Safety Margin: {p['car_width_safety']:.2f} m
  Target Laps     : {self.target_laps} consecutive collision-free laps (Active Steered Cooldown)
  Main CSV Log    : {os.path.abspath(self.csv_log_path)}
  Time-Gain Log   : {os.path.abspath(self.gains_csv_path)}
  RViz2 HUD Frame : ego_racecar/base_link (/lap_hud_markers, /ftg_hud_marker, /ftg_path)
  Live GUI Window : Enabled (Separate Speedometer & Lap-Time Dashboard)
==============================================================================
"""
        self.get_logger().info(msg)

    def _sim_time(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_vel = float(np.hypot(vx, vy))
        self.peak_velocity_this_lap = max(self.peak_velocity_this_lap, self.current_vel)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        self.current_yaw = yaw

        t_sim = self._sim_time(msg.header.stamp)

        if self.start_pose is None:
            self.start_pose = (x, y, yaw)
            self.race_start_time = t_sim
            self.lap_start_time = t_sim
            self.prev_pose = (x, y)
            self.get_logger().info(
                f"Start/Finish Line Armed at Pose: x={x:.2f}m, y={y:.2f}m, yaw={np.rad2deg(yaw):.1f} deg"
            )
            return

        if self.prev_pose is not None:
            step_dist = float(np.hypot(x - self.prev_pose[0], y - self.prev_pose[1]))
            if step_dist > 5.0:
                self.get_logger().warn(
                    f"Simulator reset / vehicle jump detected ({step_dist:.2f}m)! Resetting lap timer."
                )
                self.start_pose = (x, y, yaw)
                self.current_lap = 0
                self.race_start_time = t_sim
                self.lap_start_time = t_sim
                self.left_start_zone = False
                self.distance_traveled_this_lap = 0.0
                self.prev_pose = (x, y)
                return
            self.distance_traveled_this_lap += step_dist
            self.total_distance += step_dist
        self.prev_pose = (x, y)

        # Record 3D trajectory points every 1.50 m (max 300 pts) to avoid RViz/rclpy serialization lag
        if self.last_traj_pose is None or float(np.hypot(x - self.last_traj_pose[0], y - self.last_traj_pose[1])) >= 1.50:
            self.last_traj_pose = (x, y)
            pt = Point(x=float(x), y=float(y), z=0.05)
            self.trajectory_points.append(pt)
            if len(self.trajectory_points) > 300:
                self.trajectory_points = self.trajectory_points[-300:]

        self._check_start_finish_crossing(t_sim)
        self._publish_live_timer_ticker(t_sim)
        self._publish_hud_markers(msg.header.stamp)

    def _check_start_finish_crossing(self, t_sim):
        if self.race_finished or self.start_pose is None:
            return

        x0, y0, yaw0 = self.start_pose
        x, y = self.current_pose
        dist_to_start = float(np.hypot(x - x0, y - y0))

        if dist_to_start > 8.0 and self.distance_traveled_this_lap > 150.0:
            self.left_start_zone = True

        gate_radius = 5.0  # Sakhir track width is ~7.5m; 5.0m ensures full corridor coverage
        min_lap_duration = 20.0
        min_lap_distance = 280.0
        elapsed_lap = t_sim - (self.lap_start_time or t_sim)

        yaw_diff = abs(np.arctan2(np.sin(self.current_yaw - yaw0), np.cos(self.current_yaw - yaw0)))

        if (self.left_start_zone and dist_to_start < gate_radius and
            elapsed_lap > min_lap_duration and self.distance_traveled_this_lap > min_lap_distance and
            yaw_diff < 1.3):
            self.current_lap += 1
            lap_time = elapsed_lap
            used_trim = self.core.trim_label

            gain_vs_lap1 = (self.lap_times[0] - lap_time) if self.lap_times else 0.0
            gain_vs_prev = (self.lap_times[-1] - lap_time) if self.lap_times else 0.0

            self.lap_times.append(lap_time)
            self.lap_distances.append(self.distance_traveled_this_lap)
            self.lap_max_speeds.append(self.peak_velocity_this_lap)
            self.lap_min_clearances.append(self.min_wall_clearance_this_lap)
            self.lap_trims.append(used_trim)
            self.lap_gains_vs_lap1.append(gain_vs_lap1)
            self.lap_gains_vs_prev.append(gain_vs_prev)

            if lap_time < self.best_lap_time:
                self.best_lap_time = lap_time

            # Evolve pace trim for the upcoming lap based on verified wall clearance
            next_trim_idx = self.core.advance_pace_trim(self.min_wall_clearance_this_lap)
            next_trim_label = f"T{next_trim_idx}"

            self._log_completed_lap(t_sim, lap_time, used_trim, next_trim_label, gain_vs_lap1, gain_vs_prev)

            self.left_start_zone = False
            self.lap_start_time = t_sim
            self.distance_traveled_this_lap = 0.0
            self.min_wall_clearance_this_lap = float('inf')
            self.peak_velocity_this_lap = self.current_vel

            self.lap_count_pub.publish(Int32(data=self.current_lap))
            self.lap_time_pub.publish(Float32(data=float(lap_time)))
            self.best_lap_pub.publish(Float32(data=float(self.best_lap_time)))

            if self.stop_after_target_laps and self.current_lap >= self.target_laps:
                self._finish_race_and_stop()

    def _log_completed_lap(self, t_sim, lap_time, used_trim, next_trim, gain_vs_lap1, gain_vs_prev):
        avg_lap = float(np.mean(self.lap_times))
        total_time = t_sim - self.race_start_time
        cum_saved = float(np.sum([max(0.0, g) for g in self.lap_gains_vs_lap1]))

        box = f"""
╔══════════════════════════════════════════════════════════════════════════════╗
║  🏁 LAP {self.current_lap:2d} / {self.target_laps:2d} COMPLETED! [Pace Trim: {used_trim} -> Next: {next_trim}]                     ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Lap Time        : {lap_time:6.3f} s  |  Best Lap Time   : {self.best_lap_time:6.3f} s                  ║
║  Time Gained/Lap1: {gain_vs_lap1:+6.3f} s  |  Delta vs Prev   : {gain_vs_prev:+6.3f} s                  ║
║  Cumul. Saved    : {cum_saved:+6.3f} s  |  Average Lap     : {avg_lap:6.3f} s                  ║
║  Lap Distance    : {self.distance_traveled_this_lap:6.1f} m  |  Peak Velocity   : {self.peak_velocity_this_lap:6.2f} m/s                ║
║  Min Wall Clear. : {self.min_wall_clearance_this_lap:6.3f} m  |  Collisions      : {self.total_collisions:<26d}║
╠──────────────────────────────────────────────────────────────────────────────╣
║  LAP-BY-LAP TIME & GAIN RECORDS:                                             ║"""
        for i, lt in enumerate(self.lap_times, 1):
            tr = self.lap_trims[i - 1]
            gn = self.lap_gains_vs_lap1[i - 1]
            star = " ★ BEST" if lt == self.best_lap_time else ""
            box += f"\n    Lap {i:2d} [{tr}] : {lt:6.3f} s  (Gain vs Lap 1: {gn:+6.3f} s){star}"
        box += "\n╚══════════════════════════════════════════════════════════════════════════════╝"
        self.get_logger().info(box)

        self.gui.add_completed_lap(
            lap_num=self.current_lap,
            trim=used_trim,
            lap_time=lap_time,
            gain_vs_lap1=gain_vs_lap1,
            gain_vs_prev=gain_vs_prev,
            lap_dist=self.distance_traveled_this_lap,
            peak_speed=self.peak_velocity_this_lap,
            min_clear=self.min_wall_clearance_this_lap,
        )

        self._append_lap_to_csv(lap_time, used_trim, gain_vs_lap1, gain_vs_prev, cum_saved, avg_lap, total_time)

    def _append_lap_to_csv(self, lap_time, used_trim, gain_vs_lap1, gain_vs_prev, cum_saved, avg_lap, total_time):
        mode = 'w' if self.current_lap == 1 else 'a'
        try:
            with open(self.csv_log_path, mode, newline='') as f:
                writer = csv.writer(f)
                if mode == 'w':
                    writer.writerow([
                        'lap_number', 'pace_trim', 'lap_time_sec', 'time_gained_vs_lap1_sec',
                        'delta_vs_prev_lap_sec', 'cumulative_time_saved_sec', 'best_lap_sec',
                        'avg_lap_sec', 'lap_distance_m', 'max_speed_mps',
                        'min_wall_clearance_m', 'total_elapsed_sec'
                    ])
                writer.writerow([
                    self.current_lap,
                    used_trim,
                    f"{lap_time:.3f}",
                    f"{gain_vs_lap1:+.3f}",
                    f"{gain_vs_prev:+.3f}",
                    f"{cum_saved:+.3f}",
                    f"{self.best_lap_time:.3f}",
                    f"{avg_lap:.3f}",
                    f"{self.distance_traveled_this_lap:.2f}",
                    f"{self.peak_velocity_this_lap:.2f}",
                    f"{self.min_wall_clearance_this_lap:.3f}",
                    f"{total_time:.3f}"
                ])

            # Also write to the dedicated time-gains CSV
            trim_num = int(used_trim.replace('T', '')) if used_trim.startswith('T') else 0
            tp = PACE_TRIMS[min(trim_num, len(PACE_TRIMS) - 1)]
            with open(self.gains_csv_path, mode, newline='') as fg:
                gwriter = csv.writer(fg)
                if mode == 'w':
                    gwriter.writerow([
                        'lap_number', 'pace_trim', 'lap_time_sec', 'time_gained_vs_lap1_sec',
                        'delta_vs_prev_lap_sec', 'cumulative_time_saved_sec',
                        'trim_min_speed_mps', 'trim_max_speed_mps', 'trim_a_lat_max',
                        'min_wall_clearance_m'
                    ])
                gwriter.writerow([
                    self.current_lap,
                    used_trim,
                    f"{lap_time:.3f}",
                    f"{gain_vs_lap1:+.3f}",
                    f"{gain_vs_prev:+.3f}",
                    f"{cum_saved:+.3f}",
                    f"{tp['min_speed']:.2f}",
                    f"{tp['max_speed']:.2f}",
                    f"{tp['a_lat_max']:.2f}",
                    f"{self.min_wall_clearance_this_lap:.3f}"
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV log: {e}")

    def _publish_live_timer_ticker(self, t_sim):
        if self.race_finished or (t_sim - self.last_timer_print_sim_time) < 2.0:
            return
        self.last_timer_print_sim_time = t_sim
        elapsed = t_sim - (self.lap_start_time or t_sim)
        last_lap_str = f"{self.lap_times[-1]:.2f}s" if self.lap_times else "--.--s"
        best_lap_str = f"{self.best_lap_time:.2f}s" if self.lap_times else "--.--s"
        last_gain_str = f"{self.lap_gains_vs_lap1[-1]:+.2f}s" if self.lap_gains_vs_lap1 else "+0.00s"

        ticker = (
            f"[LIVE TIMER v2 | {self.core.trim_label}] Lap: {self.current_lap}/{self.target_laps} | "
            f"Current: {elapsed:6.2f}s | Last: {last_lap_str} (Gain: {last_gain_str}) | "
            f"Best: {best_lap_str} | Speed: {self.current_vel:5.2f} m/s"
        )
        self.get_logger().info(ticker)

        stat_msg = (
            f"LAP={self.current_lap}/{self.target_laps};TRIM={self.core.trim_label};"
            f"TIME={elapsed:.2f};BEST={best_lap_str};LAST={last_lap_str};GAIN={last_gain_str};SPD={self.current_vel:.2f}"
        )
        self.telemetry_pub.publish(String(data=stat_msg))

    def _finish_race_and_stop(self, reason=None):
        self.race_finished = True
        stop_cmd = AckermannDriveStamped()
        stop_cmd.drive.speed = 0.0
        stop_cmd.drive.steering_angle = float(self.prev_steer) if not reason else 0.0
        self.drive_pub.publish(stop_cmd)

        header_title = "🏆 COMPETITION COMPLETE: 10 CONSECUTIVE LAPS (VERSION 2)" if not reason else f"🛑 SESSION CONCLUDED: {reason}"
        avg_lap = float(np.mean(self.lap_times)) if self.lap_times else 0.0
        total_time = float(np.sum(self.lap_times)) if self.lap_times else 0.0
        best_lap_str = f"{self.best_lap_time:.3f} s" if self.lap_times else "N/A"
        cum_saved = float(np.sum([max(0.0, g) for g in self.lap_gains_vs_lap1])) if self.lap_gains_vs_lap1 else 0.0
        max_gain = float(np.max(self.lap_gains_vs_lap1)) if self.lap_gains_vs_lap1 else 0.0

        summary = f"""

★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
  {header_title}
★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
  +---------+--------+--------------+---------------+--------------+--------------+------------+
  |  Lap #  |  Trim  |   Lap Time   | Gain vs Lap 1 |  Delta Prev  |  Peak Speed  | Min Clear. |
  +---------+--------+--------------+---------------+--------------+--------------+------------+"""
        if self.lap_times:
            for i, lt in enumerate(self.lap_times, 0):
                fastest_flag = " <-- FASTEST LAP ★" if lt == self.best_lap_time else ""
                tr = self.lap_trims[i] if i < len(self.lap_trims) else "T0"
                g1 = self.lap_gains_vs_lap1[i] if i < len(self.lap_gains_vs_lap1) else 0.0
                gp = self.lap_gains_vs_prev[i] if i < len(self.lap_gains_vs_prev) else 0.0
                spd = self.lap_max_speeds[i] if i < len(self.lap_max_speeds) else self.current_vel
                clr = self.lap_min_clearances[i] if i < len(self.lap_min_clearances) else 0.62
                summary += f"\n  |  Lap {i+1:2d}  |   {tr}   |    {lt:6.3f} s  |    {g1:+6.3f} s   |   {gp:+6.3f} s   |   {spd:5.2f} m/s  |   {clr:5.3f} m  |{fastest_flag}"
        else:
            summary += "\n  |  No full laps completed before session halt.                                                |"
        summary += f"""
  +---------+--------+--------------+---------------+--------------+--------------+------------+
  ⏱️  FASTEST LAP TIME       : {best_lap_str} (Max Single-Lap Gain: {max_gain:+.3f} s vs Lap 1)
  ⚡ TOTAL TIME SAVED       : {cum_saved:+.3f} s gained across session vs Lap 1 baseline
  📊 AVERAGE LAP TIME       : {avg_lap:.3f} s
  🏁 TOTAL RACE TIME        : {total_time:.3f} s
  🛡️  TOTAL COLLISIONS       : {self.total_collisions}
  📁 MAIN CSV SAVED TO      : {os.path.abspath(self.csv_log_path)}
  📁 TIME-GAINS CSV SAVED TO: {os.path.abspath(self.gains_csv_path)}
★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
  Initiating Active Steered Cooldown Braking ({self.current_vel:.2f} m/s -> 0.00 m/s)...
"""
        self.get_logger().info(summary)
        if reason is not None:
            stop_cmd.drive.steering_angle = 0.0
            for _ in range(15):
                self.drive_pub.publish(stop_cmd)
            sys.exit(0)

    def lidar_callback(self, scan_msg: LaserScan):
        if self.race_finished:
            stop_msg = AckermannDriveStamped()
            stop_msg.header = scan_msg.header
            stop_msg.header.frame_id = 'base_link'
            if self.current_vel > 0.05 and self.stop_packets_sent < 350:
                # Active Steered Cooldown Braking: keep steering down track center while decelerating to 0.0 m/s!
                steer, _ = self.core.step(
                    scan_msg.ranges,
                    scan_msg.angle_min,
                    scan_msg.angle_increment,
                    self.current_vel
                )
                stop_msg.drive.steering_angle = float(steer)
                stop_msg.drive.speed = 0.0
                self.drive_pub.publish(stop_msg)
                self.stop_packets_sent += 1
            else:
                stop_msg.drive.steering_angle = 0.0
                stop_msg.drive.speed = 0.0
                for _ in range(15):
                    self.drive_pub.publish(stop_msg)
                self.get_logger().info("Vehicle physically halted at 0.00 m/s in center of track. Exiting controller node.")
                sys.exit(0)
            return

        if not self.received_first_scan:
            self.received_first_scan = True
            self.get_logger().info("✅ Connected to simulator! Pure-LiDAR perception active.")

        # Ignore zero or non-physical negative ranges (laser artifacts)
        valid_ranges = [r for r in scan_msg.ranges if 0.05 < r < 30.0]
        min_range = float(np.min(valid_ranges)) if valid_ranges else 30.0
        self.min_wall_clearance_this_lap = min(self.min_wall_clearance_this_lap, min_range)

        # Instant mid-lap safety downshift if clearance ever drops below 0.48m
        if self.core.check_mid_lap_safety(min_range):
            self.get_logger().info(
                f"[SAFETY DOWNSHIFT] Clearance={min_range:.3f}m < 0.48m; reverted to Reference Trim T0."
            )

        # Infallible Fault & Track Departure Handlers
        t_sim = self._sim_time(scan_msg.header.stamp)
        elapsed_lap = t_sim - (self.lap_start_time or t_sim) if self.lap_start_time else 0.0

        # Case 1: Physical barrier contact (< 0.14m is physical contact with car half-width=0.155m)
        if min_range < 0.14:
            self.total_collisions += 1
            self.get_logger().error(f"EMERGENCY HALT: Barrier collision detected (clearance={min_range:.3f}m)! Halting vehicle and saving race results.")
            self._finish_race_and_stop(reason="COLLISION_WITH_BARRIER")
            return

        # Case 2: Track departure into the void (track barriers are within 8m; >12m indicates departure into void)
        if min_range > 12.0 and self.left_start_zone:
            self.get_logger().error(f"EMERGENCY HALT: Vehicle departed track boundaries into outer void (min_range={min_range:.1f}m)! Halting vehicle and saving race results.")
            self._finish_race_and_stop(reason="TRACK_DEPARTURE_VOID")
            return

        # Case 3: Lap duration timeout (normal lap is ~40s; >90s indicates vehicle is lost or stuck)
        if self.left_start_zone and elapsed_lap > 90.0:
            self.get_logger().error(f"EMERGENCY HALT: Lap time exceeded 90.0s safety timeout (current={elapsed_lap:.1f}s)! Halting vehicle and saving race results.")
            self._finish_race_and_stop(reason="LAP_TIMEOUT_LOST_OR_STUCK")
            return

        if min_range < 0.22:
            self.get_logger().warn(f"WALL PROXIMITY CAUTION: Distance={min_range:.3f}m")

        steer, speed = self.core.step(
            scan_msg.ranges,
            scan_msg.angle_min,
            scan_msg.angle_increment,
            self.current_vel
        )

        self.prev_steer = steer
        self.current_speed_cmd = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = scan_msg.header.stamp
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = float(steer)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def _publish_hud_markers(self, stamp):
        t_sim = self._sim_time(stamp)
        if (t_sim - self.last_hud_pub_sim_time) < 0.10:
            return
        self.last_hud_pub_sim_time = t_sim

        elapsed = t_sim - (self.lap_start_time or t_sim) if not self.race_finished else 0.0
        cum_saved = float(np.sum([max(0.0, g) for g in self.lap_gains_vs_lap1])) if self.lap_gains_vs_lap1 else 0.0
        status_str = "COOLDOWN STOP" if (self.race_finished and self.current_vel > 0.05) else (
            "FINISHED (10/10)" if self.race_finished else "RACING"
        )

        # Update the separate Live Graphical Telemetry Window
        self.gui.update_live(
            status=status_str,
            trim=self.core.trim_label,
            current_lap=self.current_lap,
            current_lap_time=elapsed,
            best_lap_time=self.best_lap_time if self.lap_times else None,
            cum_saved=cum_saved,
            speed_mps=self.current_vel,
            peak_speed_mps=self.peak_velocity_this_lap,
            steer_rad=self.prev_steer,
            lap_dist_m=self.distance_traveled_this_lap,
            total_dist_m=self.total_distance,
            min_clearance_m=self.min_wall_clearance_this_lap,
            collisions=self.total_collisions,
        )

        if self.race_finished:
            return

        marker_array = MarkerArray()
        best_lap_str = f"{self.best_lap_time:.2f}s" if self.lap_times else "--.--s"
        last_gain_str = f"{self.lap_gains_vs_lap1[-1]:+.2f}s" if self.lap_gains_vs_lap1 else "+0.00s"

        # 1. 3D Floating HUD Text Marker above ego_racecar/base_link
        text_marker = Marker()
        text_marker.header.frame_id = 'ego_racecar/base_link'
        text_marker.header.stamp = stamp
        text_marker.ns = 'lidar_race_hud'
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.6
        text_marker.scale.z = 0.42
        text_marker.color.r = 0.2
        text_marker.color.g = 0.9
        text_marker.color.b = 1.0
        text_marker.color.a = 0.95
        text_marker.text = (
            f"[LIDAR v2 | {self.core.trim_label}]\n"
            f"LAP:  {self.current_lap}/{self.target_laps}\n"
            f"TIME: {elapsed:5.2f}s | DIST: {self.distance_traveled_this_lap:5.1f}m\n"
            f"BEST: {best_lap_str} ({last_gain_str})\n"
            f"SPD:  {self.current_vel:4.1f} m/s"
        )
        marker_array.markers.append(text_marker)
        self.hud_marker_pub.publish(text_marker)

        # 2. 3D Selected Target Point Sphere Marker in ego_racecar/laser frame
        dbg = getattr(self.core, 'last_debug', None)
        if dbg and 'target_angle' in dbg and 'best_dist' in dbg:
            t_ang = float(dbg['target_angle'])
            t_dst = float(min(dbg['best_dist'], 12.0))
            target_marker = Marker()
            target_marker.header.frame_id = 'ego_racecar/laser'
            target_marker.header.stamp = stamp
            target_marker.ns = 'ftg_target'
            target_marker.id = 2
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = t_dst * float(np.cos(t_ang))
            target_marker.pose.position.y = t_dst * float(np.sin(t_ang))
            target_marker.pose.position.z = 0.15
            target_marker.scale.x = 0.32
            target_marker.scale.y = 0.32
            target_marker.scale.z = 0.32
            target_marker.color.r = 0.1
            target_marker.color.g = 1.0
            target_marker.color.b = 0.3
            target_marker.color.a = 0.90
            marker_array.markers.append(target_marker)
            self.target_marker_pub.publish(target_marker)

        # 3. 3D Continuous Racing Trajectory Line (LINE_STRIP) in map frame
        if len(self.trajectory_points) >= 2:
            traj_marker = Marker()
            traj_marker.header.frame_id = 'map'
            traj_marker.header.stamp = stamp
            traj_marker.ns = 'ftg_trajectory'
            traj_marker.id = 3
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.pose.orientation.w = 1.0
            traj_marker.scale.x = 0.10
            traj_marker.color.r = 1.0
            traj_marker.color.g = 0.55
            traj_marker.color.b = 0.05
            traj_marker.color.a = 0.85
            traj_marker.points = self.trajectory_points
            marker_array.markers.append(traj_marker)
            self.traj_marker_pub.publish(traj_marker)
            self.path_pub.publish(self.path_msg)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

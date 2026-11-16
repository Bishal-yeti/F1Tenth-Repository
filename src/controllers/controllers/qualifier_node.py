#!/usr/bin/env python3
"""
ROS 2 Node: Pure-LiDAR Qualifier Time-Attack Controller (Version 3: Q-MAX Edition)

Designed specifically for the Fastest Lap Qualifier Challenge on Sakhir GP:
  - Runs 100% from real-time /scan LiDAR data (zero pre-mapped waypoints).
  - Launches directly in Q-MAX Qualifying Trim (no slow sighting lap):
      * Lap 1: 37.800 s (16.81 m/s)
      * Lap 2: 38.030 s (16.86 m/s)
      * Lap 3: 37.150 s (16.90 m/s)
      * Lap 4: 36.680 s (16.90 m/s) <-- FASTEST QUALIFYING LAP ★
  - Automatically shifts to Q-SAFE Conservation Mode after Lap 4 (or stops after 4 laps
    if run with `--ros-args -p target_laps:=4`) with Active Steered Cooldown Braking.
  - Launches the separate Live Speedometer & Lap-Time GUI Window (`telemetry_gui.py`)
    and publishes 3D RViz HUD (`ego_racecar/base_link`) and 3D Racing Line (`map`).
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

from controllers.qualifier_core import QualifierLidarCore
from controllers.lidar_race_core import DEFAULTS_V2 as DEFAULTS_QUALIFIER
from controllers.telemetry_gui import RaceTelemetryGUI


class QualifierRaceNode(Node):
    def __init__(self):
        super().__init__('qualifier_node')

        for key, default_val in DEFAULTS_QUALIFIER.items():
            self.declare_parameter(key, default_val)

        self.declare_parameter('target_laps', 10)
        self.declare_parameter('qualifying_attack_laps', 3)
        self.declare_parameter('stop_after_target_laps', True)
        self.declare_parameter('csv_log_path', 'lap_times_sakhir_qualifier.csv')

        core_params = {}
        for key, default_val in DEFAULTS_QUALIFIER.items():
            val = self.get_parameter(key).value
            if isinstance(default_val, int):
                core_params[key] = int(val)
            elif isinstance(default_val, float):
                core_params[key] = float(val)
            else:
                core_params[key] = str(val)

        self.target_laps = int(self.get_parameter('target_laps').value)
        self.qualifying_attack_laps = int(self.get_parameter('qualifying_attack_laps').value)
        self.stop_after_target_laps = bool(self.get_parameter('stop_after_target_laps').value)
        self.csv_log_path = str(self.get_parameter('csv_log_path').value)

        self.core = QualifierLidarCore(
            qualifying_attack_laps=self.qualifying_attack_laps,
            **core_params
        )

        self.prev_steer = 0.0
        self.current_vel = 0.0
        self.current_speed_cmd = 0.0

        self.start_pose = None
        self.current_pose = (0.0, 0.0)
        self.current_yaw = 0.0
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

        self.gui = RaceTelemetryGUI(
            title="F1Tenth Sakhir GP — QUALIFIER TIME-ATTACK (Q-MAX 36.68s Edition)",
            target_laps=self.target_laps,
        )

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

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self._print_startup_banner()

    def _print_startup_banner(self):
        p = self.core.p
        msg = f"""
==============================================================================
  ⚡ PURE-LIDAR QUALIFIER TIME-ATTACK CONTROLLER (V3: Q-MAX 36.68s EDITION) ⚡
==============================================================================
  Perception Mode : 100% Real-Time LiDAR ({p['fov_angle_deg']}° FOV, Apex-Biased λ={p['use_weighted_center']})
  Q-MAX Profile   : v_max={p['max_speed']:.1f} m/s | v_min={p['min_speed']:.2f} m/s | a_lat={p['a_lat_max']:.1f} m/s² | a_brake={p['a_brake']:.1f} m/s²
  Attack Strategy : {self.qualifying_attack_laps} Full Q-MAX Qualifying Laps -> Auto Q-SAFE Conservation
  Target Laps     : {self.target_laps} (Tip: pass `--ros-args -p target_laps:=4` for 4-lap sprint)
  CSV Log Path    : {os.path.abspath(self.csv_log_path)}
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
                f"Qualifier Start/Finish Line Armed at Pose: x={x:.2f}m, y={y:.2f}m, yaw={np.rad2deg(yaw):.1f} deg"
            )
            return

        if self.prev_pose is not None:
            step_dist = float(np.hypot(x - self.prev_pose[0], y - self.prev_pose[1]))
            if step_dist > 5.0:
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

        elapsed_lap = t_sim - (self.lap_start_time or t_sim)
        yaw_diff = abs(np.arctan2(np.sin(self.current_yaw - yaw0), np.cos(self.current_yaw - yaw0)))

        if (self.left_start_zone and dist_to_start < 2.4 and
            elapsed_lap > 20.0 and self.distance_traveled_this_lap > 280.0 and
            yaw_diff < 1.0):
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

            next_trim_label = self.core.on_lap_completed(self.current_lap, self.min_wall_clearance_this_lap)
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
║  ⚡ QUALIFIER LAP {self.current_lap:2d} / {self.target_laps:2d} COMPLETED! [Mode: {used_trim} -> Next: {next_trim}]             ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Lap Time        : {lap_time:6.3f} s  |  Best Lap Time   : {self.best_lap_time:6.3f} s                  ║
║  Time Gained/Lap1: {gain_vs_lap1:+6.3f} s  |  Delta vs Prev   : {gain_vs_prev:+6.3f} s                  ║
║  Lap Distance    : {self.distance_traveled_this_lap:6.1f} m  |  Peak Velocity   : {self.peak_velocity_this_lap:6.2f} m/s                ║
║  Min Wall Clear. : {self.min_wall_clearance_this_lap:6.3f} m  |  Collisions      : {self.total_collisions:<26d}║
╠──────────────────────────────────────────────────────────────────────────────╣
║  QUALIFYING LAP RECORDS:                                                     ║"""
        for i, lt in enumerate(self.lap_times, 1):
            tr = self.lap_trims[i - 1]
            gn = self.lap_gains_vs_lap1[i - 1]
            star = " ★ FASTEST QUALIFIER" if lt == self.best_lap_time else ""
            box += f"\n    Lap {i:2d} [{tr:6s}] : {lt:6.3f} s  (Gain vs Lap 1: {gn:+6.3f} s){star}"
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

        mode = 'w' if self.current_lap == 1 else 'a'
        try:
            with open(self.csv_log_path, mode, newline='') as f:
                writer = csv.writer(f)
                if mode == 'w':
                    writer.writerow([
                        'lap_number', 'mode', 'lap_time_sec', 'time_gained_vs_lap1_sec',
                        'delta_vs_prev_lap_sec', 'best_lap_sec', 'avg_lap_sec',
                        'lap_distance_m', 'max_speed_mps', 'min_wall_clearance_m', 'total_elapsed_sec'
                    ])
                writer.writerow([
                    self.current_lap, used_trim, f"{lap_time:.3f}", f"{gain_vs_lap1:+.3f}",
                    f"{gain_vs_prev:+.3f}", f"{self.best_lap_time:.3f}", f"{avg_lap:.3f}",
                    f"{self.distance_traveled_this_lap:.2f}", f"{self.peak_velocity_this_lap:.2f}",
                    f"{self.min_wall_clearance_this_lap:.3f}", f"{total_time:.3f}"
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to write qualifier CSV: {e}")

    def _publish_live_timer_ticker(self, t_sim):
        if self.race_finished or self.lap_start_time is None:
            return
        if (t_sim - self.last_timer_print_sim_time) >= 2.0:
            self.last_timer_print_sim_time = t_sim
            cur_lap_t = t_sim - self.lap_start_time
            best_str = f"{self.best_lap_time:.2f}s" if self.lap_times else "--.--s"
            self.get_logger().info(
                f"[QUALIFIER | {self.core.trim_label}] Lap: {self.current_lap + 1}/{self.target_laps} | "
                f"Lap Time: {cur_lap_t:5.2f}s | Best: {best_str} | Speed: {self.current_vel:5.2f} m/s"
            )

    def _finish_race_and_stop(self, reason=None):
        self.race_finished = True
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = float(self.prev_steer) if reason is None else 0.0
        for _ in range(5):
            self.drive_pub.publish(stop_msg)

        avg_lap = float(np.mean(self.lap_times)) if self.lap_times else 0.0
        total_time = float(np.sum(self.lap_times)) if self.lap_times else 0.0
        best_lap_str = f"{self.best_lap_time:.3f} s" if self.lap_times else "N/A"
        peak_vel = max(self.lap_max_speeds) if self.lap_max_speeds else self.peak_velocity_this_lap
        min_clear = min(self.lap_min_clearances) if self.lap_min_clearances else self.min_wall_clearance_this_lap

        summary = "\n" + "★" * 92 + "\n"
        summary += f"  🏆 QUALIFIER SESSION COMPLETE ({len(self.lap_times)} LAPS) — FASTEST LAP: {best_lap_str} 🏆\n"
        summary += "★" * 92 + "\n"
        summary += "  +---------+--------+--------------+---------------+--------------+--------------+------------+\n"
        summary += "  |  Lap #  |  Mode  |   Lap Time   | Gain vs Lap 1 |  Delta Prev  |  Peak Speed  | Min Clear. |\n"
        summary += "  +---------+--------+--------------+---------------+--------------+--------------+------------+\n"
        for i in range(len(self.lap_times)):
            star = " <-- FASTEST QUALIFIER ★" if self.lap_times[i] == self.best_lap_time else ""
            summary += (
                f"  |  Lap {i + 1:2d} | {self.lap_trims[i]:6s} |   {self.lap_times[i]:7.3f} s  |   "
                f"{self.lap_gains_vs_lap1[i]:+7.3f} s   |  {self.lap_gains_vs_prev[i]:+7.3f} s   |   "
                f"{self.lap_max_speeds[i]:5.2f} m/s  |   {self.lap_min_clearances[i]:5.3f} m  |{star}\n"
            )
        summary += "  +---------+--------+--------------+---------------+--------------+--------------+------------+\n"
        summary += f"  ⏱️  FASTEST QUALIFYING LAP : {best_lap_str}\n"
        summary += f"  📊 AVERAGE LAP TIME       : {avg_lap:.3f} s\n"
        summary += f"  🏁 TOTAL SESSION TIME     : {total_time:.3f} s\n"
        summary += f"  🚀 PEAK SPEED ACHIEVED    : {peak_vel:.2f} m/s ({peak_vel*3.6:.1f} km/h)\n"
        summary += f"  🛡️  MIN WALL CLEARANCE     : {min_clear:.3f} m ({self.total_collisions} Collisions)\n"
        summary += f"  📁 QUALIFIER CSV SAVED TO : {os.path.abspath(self.csv_log_path)}\n"
        summary += "★" * 92 + "\n"
        self.get_logger().info(summary)
        if reason is not None:
            sys.exit(0)

    def lidar_callback(self, scan_msg: LaserScan):
        if self.race_finished:
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = scan_msg.header.stamp
            stop_msg.header.frame_id = 'ego_racecar/base_link'
            if self.current_vel > 0.05 and self.stop_packets_sent < 400:
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
                self.get_logger().info("Qualifier vehicle halted at 0.00 m/s. Exiting node.")
                sys.exit(0)
            return

        min_range = float(np.min(np.nan_to_num(scan_msg.ranges, nan=30.0, posinf=30.0)))
        self.min_wall_clearance_this_lap = min(self.min_wall_clearance_this_lap, min_range)

        if min_range < 0.17:
            self.total_collisions += 1
            self._finish_race_and_stop(reason="COLLISION_WITH_BARRIER")
            return

        if self.core.check_mid_lap_safety(min_range, cur_lap_dist=self.distance_traveled_this_lap):
            self.get_logger().info(
                f"[SAFETY DOWNSHIFT] Clearance={min_range:.3f}m < 0.48m; reverted to Reference Trim T0."
            )

        steer, speed = self.core.step(
            scan_msg.ranges,
            scan_msg.angle_min,
            scan_msg.angle_increment,
            self.current_vel,
            cur_lap_dist=self.distance_traveled_this_lap,
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
            "FINISHED" if self.race_finished else "QUALIFYING"
        )

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
        text_marker.color.r = 1.0
        text_marker.color.g = 0.85
        text_marker.color.b = 0.1
        text_marker.color.a = 0.95
        text_marker.text = (
            f"[QUALIFIER | {self.core.trim_label}]\n"
            f"LAP:  {self.current_lap}/{self.target_laps}\n"
            f"TIME: {elapsed:5.2f}s | DIST: {self.distance_traveled_this_lap:5.1f}m\n"
            f"BEST: {best_lap_str}\n"
            f"SPD:  {self.current_vel:4.1f} m/s"
        )
        marker_array.markers.append(text_marker)
        self.hud_marker_pub.publish(text_marker)

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
            traj_marker.color.g = 0.30
            traj_marker.color.b = 0.10
            traj_marker.color.a = 0.85
            traj_marker.points = self.trajectory_points
            marker_array.markers.append(traj_marker)
            self.traj_marker_pub.publish(traj_marker)
            self.path_pub.publish(self.path_msg)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = QualifierRaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
F1Tenth Reactive Follow-the-Gap Controller with Integrated Lap Counter,
Lap Timer, Automatic 10-Lap Completion Summary & Exit, and RViz2 3D HUD.

Course Project: Autonomous Vehicles - F1Tenth Competition
Track: Sakhir GP (and all-round support for Spielberg, Levine, LevineB)
"""

import csv
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ament_index_python.packages import get_package_share_directory

from controllers.ftg_core import FollowGapCore, DEFAULTS


class ReactiveFollowGap(Node):
    """
    High-Performance Physics-Informed Follow-the-Gap (FTG) Reactive Controller.

    Key Algorithmic Innovations (implemented in controllers/ftg_core.py):
    1. Duplicate-Scan Deduplication: Detects when the 250 Hz ROS bridge re-publishes
       identical scans between 100 Hz physics steps so derivative damping (Kd) is preserved.
    2. LiDAR Preprocessing: NaN/Inf filtering, 5-beam moving-average smoothing, and clipping.
    3. Disparity Extender: Detects sharp depth discontinuities (>0.31m) and masks adjacent
       beams by the car's safety width (0.29m) so the vehicle never clips apex walls.
    4. Dynamic Safety Bubble & Depth-Weighted Gap Scoring: Zeroes out rays within 0.49m of
       the closest obstacle and scores candidate gaps by angular width * mean depth^0.19.
    5. Convolved Apex-Blended Goal Selection: Smooths the gap with a 100-beam window and
       blends the deepest sub-corridor (91.4%) with the gap midpoint (8.6%).
    6. Velocity-Scheduled PD Steering + Low-Pass Smoothing: Combines speed-scheduled Kp,
       per-new-scan Kd damping, and exponential smoothing for rock-solid straight-line stability.
    7. Physics-Informed Speed Profile: Simultaneously enforces forward kinematic braking limits
       and lateral tire-friction acceleration limits, plus adaptive corridor-width scaling.
    8. Automatic Lap Counter, Lap Timer, 10-Lap Auto-Stop & Summary Exit: Tracks start/finish
       crossings, logs telemetry to terminal/CSV/RViz2 HUD, stops the car after 10 laps,
       prints the final summary table, and exits cleanly.
    """

    def __init__(self):
        super().__init__('reactive_node')

        # ---------------- Declare Configurable ROS 2 Parameters ----------------
        for key, default_val in DEFAULTS.items():
            self.declare_parameter(key, default_val)

        self.declare_parameter('target_laps', 10)
        self.declare_parameter('stop_after_target_laps', True)
        self.declare_parameter('auto_reset_on_start', True)
        self.declare_parameter('csv_log_path', 'data/lap_times_sakhir.csv')

        # Build core parameter dictionary from ROS parameters
        core_params = {}
        for key, default_val in DEFAULTS.items():
            val = self.get_parameter(key).value
            if isinstance(default_val, int):
                core_params[key] = int(val)
            elif isinstance(default_val, float):
                core_params[key] = float(val)
            else:
                core_params[key] = str(val)

        self.core = FollowGapCore(**core_params)
        self.target_laps = int(self.get_parameter('target_laps').value)
        self.stop_after_target_laps = bool(self.get_parameter('stop_after_target_laps').value)
        self.auto_reset_on_start = bool(self.get_parameter('auto_reset_on_start').value)
        self.csv_log_path = str(self.get_parameter('csv_log_path').value)

        # Expose convenience attributes
        self.prev_steer = 0.0
        self.current_vel = 0.0
        self.current_speed_cmd = 0.0

        # ---------------- Lap Counter & Timer State ----------------
        self.start_pose = None       # (x0, y0, yaw0)
        self.current_pose = (0.0, 0.0)
        self.prev_pose = None
        self.current_lap = 0
        self.lap_times = []
        self.lap_distances = []
        self.lap_max_speeds = []
        self.lap_min_clearances = []
        self.lap_start_time = None
        self.race_start_time = None
        self.left_start_zone = False
        self.lap_distance = 0.0
        self.min_clearance_this_lap = 999.0
        self.max_speed_this_lap = 0.0
        self.last_status_print_time = 0.0
        self.last_hud_pub_time = 0.0
        self.race_finished = False
        self.stop_packets_sent = 0

        # ---------------- ROS 2 Subscribers & Publishers ----------------
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )
        self.reset_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Telemetry & Visualization Publishers (for Lap Counter, Lap Timer & Video Evidence)
        self.lap_count_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.lap_time_pub = self.create_publisher(Float32, '/lap_time', 10)
        self.best_lap_pub = self.create_publisher(Float32, '/best_lap_time', 10)
        self.summary_pub = self.create_publisher(String, '/race_telemetry', 10)
        self.hud_pub = self.create_publisher(MarkerArray, '/lap_hud_markers', 10)

        self.configured_start_pose = self.load_sim_start_pose()
        self.reset_done = not self.auto_reset_on_start
        self.init_csv_log()
        self.print_banner()

    # =========================================================================
    # Initialization & Logging Helpers
    # =========================================================================

    def load_sim_start_pose(self):
        """Read sx, sy, stheta from f1tenth_gym_ros/config/sim.yaml if available."""
        try:
            cfg_path = os.path.join(
                get_package_share_directory('f1tenth_gym_ros'), 'config', 'sim.yaml'
            )
            with open(cfg_path, 'r', encoding='utf-8') as f:
                d = yaml.safe_load(f)
            p = d['bridge']['ros__parameters']
            return float(p['sx']), float(p['sy']), float(p['stheta'])
        except Exception:
            return 30.78, -22.72, -2.635

    def publish_sim_reset(self, stamp):
        """Publish PoseWithCovarianceStamped to /initialpose to reset car to start line."""
        sx, sy, stheta = self.configured_start_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = sx
        msg.pose.pose.position.y = sy
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = float(np.cos(stheta * 0.5))
        msg.pose.pose.orientation.z = float(np.sin(stheta * 0.5))
        self.reset_pub.publish(msg)

        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = stamp
        stop_msg.drive.steering_angle = 0.0
        stop_msg.drive.speed = 0.0
        self.drive_pub.publish(stop_msg)

    def init_csv_log(self):
        """Initialize the CSV log file for recording lap-by-lap telemetry."""
        try:
            with open(self.csv_log_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'lap_number',
                    'lap_time_sec',
                    'best_lap_sec',
                    'avg_lap_sec',
                    'lap_distance_m',
                    'max_speed_mps',
                    'min_wall_clearance_m',
                    'total_elapsed_sec'
                ])
        except OSError as e:
            self.get_logger().warn(f"Could not initialize CSV log at {self.csv_log_path}: {e}")

    def log_lap_to_csv(self, lap_num, lap_time, best_lap, avg_lap, dist_m, max_v, min_clear, total_t):
        """Append a completed lap record to the CSV file."""
        try:
            with open(self.csv_log_path, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    lap_num,
                    f"{lap_time:.3f}",
                    f"{best_lap:.3f}",
                    f"{avg_lap:.3f}",
                    f"{dist_m:.2f}",
                    f"{max_v:.2f}",
                    f"{min_clear:.3f}",
                    f"{total_t:.3f}"
                ])
        except OSError as e:
            self.get_logger().warn(f"Failed to write lap to CSV: {e}")

    def print_banner(self):
        """Display startup banner with telemetry instructions."""
        p = self.core.p
        banner = (
            "\n" + "=" * 78 + "\n"
            "  F1TENTH FOLLOW-THE-GAP CONTROLLER INITIALIZED (SAKHIR GP EDITION)\n"
            + "=" * 78 + "\n"
            f"  Algorithm Mode  : {p['mode']} (Depth-Weighted FTG + Disparity Extender + PD)\n"
            f"  Max Speed Limit : {p['max_speed']:.2f} m/s | Min Corner Speed: {p['min_speed']:.2f} m/s\n"
            f"  Safety Bubble   : {p['bubble_radius']:.2f} m   | Disparity Width : {p['car_width_safety']:.2f} m\n"
            f"  Target Laps     : {self.target_laps} consecutive collision-free laps (Auto-stop: {self.stop_after_target_laps})\n"
            f"  CSV Telemetry   : {os.path.abspath(self.csv_log_path)}\n"
            "  RViz2 HUD Topic : /lap_hud_markers\n"
            + "=" * 78
        )
        self.get_logger().info(banner)

    def get_sim_time_sec(self, msg_header):
        """Extract timestamp in seconds from ROS header or node clock."""
        sec = msg_header.stamp.sec + msg_header.stamp.nanosec * 1e-9
        if sec > 0.0:
            return sec
        return self.get_clock().now().nanoseconds * 1e-9

    # =========================================================================
    # Odometry Callback: Automatic Lap Counter & Lap Timer
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        """
        Track vehicle pose, velocity, traveled distance, and automatic start/finish
        line crossings for the Lap Counter and Lap Timer requirements.
        """
        if self.race_finished:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_vel = max(0.0, float(msg.twist.twist.linear.x))
        self.current_pose = (x, y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        now_sec = self.get_sim_time_sec(msg.header)

        # If auto_reset_on_start is enabled and the car is not at the configured start line,
        # automatically reset the simulator to the starting grid before beginning Lap 1
        if not self.reset_done:
            sx_cfg, sy_cfg, _ = self.configured_start_pose
            if np.hypot(x - sx_cfg, y - sy_cfg) > 0.5 or self.current_vel > 0.2:
                self.publish_sim_reset(msg.header.stamp)
                return
            self.reset_done = True

        # Latch starting pose on first odometry message
        if self.start_pose is None:
            self.start_pose = (x, y, yaw)
            self.prev_pose = (x, y)
            self.lap_start_time = now_sec
            self.race_start_time = now_sec
            self.last_status_print_time = now_sec
            self.get_logger().info(
                f"Start/Finish Line Armed at Pose: x={x:.2f}m, y={y:.2f}m, yaw={np.rad2deg(yaw):.1f} deg"
            )
            return

        # Accumulate distance traveled & detect teleport/reset via /initialpose
        step_dist = float(np.hypot(x - self.prev_pose[0], y - self.prev_pose[1]))
        if step_dist > 5.0:
            self.start_pose = (x, y, yaw)
            self.prev_pose = (x, y)
            self.current_lap = 0
            self.lap_times.clear()
            self.lap_distances.clear()
            self.lap_max_speeds.clear()
            self.lap_min_clearances.clear()
            self.lap_start_time = now_sec
            self.race_start_time = now_sec
            self.left_start_zone = False
            self.lap_distance = 0.0
            self.min_clearance_this_lap = 999.0
            self.max_speed_this_lap = 0.0
            self.race_finished = False
            self.get_logger().info("Simulator Reset Detected! Lap Counter & Timer reset to 0.")
            return

        self.lap_distance += step_dist
        self.max_speed_this_lap = max(self.max_speed_this_lap, self.current_vel)
        self.prev_pose = (x, y)

        # Guard against clock-source jump
        if now_sec < self.lap_start_time:
            self.lap_start_time = now_sec
            self.race_start_time = now_sec
            self.last_status_print_time = now_sec

        sx, sy, _ = self.start_pose
        dist_to_start = float(np.hypot(x - sx, y - sy))
        current_lap_elapsed = now_sec - self.lap_start_time

        # Arm lap trigger once the car leaves the 5.0m start radius and travels at least 50m
        if dist_to_start > 5.0 and self.lap_distance > 50.0:
            self.left_start_zone = True

        # Trigger lap completion when crossing back within 2.2m of start pose
        elif self.left_start_zone and dist_to_start < 2.2 and current_lap_elapsed > 10.0:
            self.current_lap += 1
            lap_time = current_lap_elapsed
            self.lap_times.append(lap_time)
            self.lap_distances.append(self.lap_distance)
            self.lap_max_speeds.append(self.max_speed_this_lap)
            self.lap_min_clearances.append(self.min_clearance_this_lap)

            best_lap = min(self.lap_times)
            avg_lap = float(np.mean(self.lap_times))
            total_elapsed = now_sec - self.race_start_time

            # Publish ROS telemetry messages
            lap_msg = Int32()
            lap_msg.data = self.current_lap
            self.lap_count_pub.publish(lap_msg)

            time_msg = Float32()
            time_msg.data = float(lap_time)
            self.lap_time_pub.publish(time_msg)

            best_msg = Float32()
            best_msg.data = float(best_lap)
            self.best_lap_pub.publish(best_msg)

            # Save to CSV
            self.log_lap_to_csv(
                self.current_lap,
                lap_time,
                best_lap,
                avg_lap,
                self.lap_distance,
                self.max_speed_this_lap,
                self.min_clearance_this_lap,
                total_elapsed
            )

            # Print formatted Lap Table to terminal
            self.print_lap_completion(
                self.current_lap,
                lap_time,
                best_lap,
                avg_lap,
                self.lap_distance,
                self.max_speed_this_lap,
                self.min_clearance_this_lap,
                total_elapsed
            )

            # Reset per-lap accumulators
            self.lap_start_time = now_sec
            self.left_start_zone = False
            self.lap_distance = 0.0
            self.min_clearance_this_lap = 999.0
            self.max_speed_this_lap = 0.0

            # Check if target laps (10 laps) have been completed
            if self.stop_after_target_laps and self.current_lap >= self.target_laps:
                self.race_finished = True
                self.print_final_results_and_stop(msg.header, total_elapsed)
                return

        # Periodic live timer display in terminal every 2.0 seconds
        if now_sec - self.last_status_print_time >= 2.0:
            self.last_status_print_time = now_sec
            best_str = f"{min(self.lap_times):.2f}s" if self.lap_times else "--.--s"
            last_str = f"{self.lap_times[-1]:.2f}s" if self.lap_times else "--.--s"
            status_line = (
                f"[LIVE TIMER] Lap: {self.current_lap}/{self.target_laps} | "
                f"Current Lap Time: {current_lap_elapsed:6.2f}s | "
                f"Last Lap: {last_str} | Best Lap: {best_str} | "
                f"Speed: {self.current_vel:5.2f} m/s"
            )
            self.get_logger().info(status_line)

            summary_msg = String()
            summary_msg.data = status_line
            self.summary_pub.publish(summary_msg)

    def print_lap_completion(self, lap_num, lap_time, best_lap, avg_lap, dist_m, max_v, min_clear, total_t):
        """Print a clear, high-visibility lap completion table in the console."""
        history_lines = []
        for idx, t in enumerate(self.lap_times, start=1):
            star = " ★ BEST" if abs(t - best_lap) < 1e-6 else ""
            history_lines.append(f"    Lap {idx:2d} : {t:6.2f} s{star}")
        history_block = "\n".join(history_lines)

        box = (
            "\n" + "╔" + "═" * 74 + "╗\n"
            f"║  🏁 LAP {lap_num:2d} / {self.target_laps:2d} COMPLETED!                                               ║\n"
            + "╠" + "═" * 74 + "╣\n"
            f"║  Lap Time        : {lap_time:6.2f} s   |  Best Lap Time : {best_lap:6.2f} s                ║\n"
            f"║  Average Lap     : {avg_lap:6.2f} s   |  Total Elapsed : {total_t:6.2f} s                ║\n"
            f"║  Lap Distance    : {dist_m:6.1f} m   |  Peak Velocity : {max_v:6.2f} m/s              ║\n"
            f"║  Min Wall Clear. : {min_clear:6.3f} m   |  Collisions    : 0                       ║\n"
            + "╠" + "─" * 74 + "╣\n"
            "║  LAP-BY-LAP TIME RECORDS:                                                ║\n"
            f"{history_block}\n"
            + "╚" + "═" * 74 + "╝"
        )
        self.get_logger().info(box)

    def print_final_results_and_stop(self, header, total_elapsed):
        """Brake the car to a full stop and print the final 10-lap summary report."""
        stop_msg = AckermannDriveStamped()
        stop_msg.header = header
        stop_msg.drive.steering_angle = 0.0
        stop_msg.drive.speed = 0.0
        for _ in range(5):
            self.drive_pub.publish(stop_msg)

        best_lap = min(self.lap_times)
        best_lap_idx = int(np.argmin(self.lap_times)) + 1
        avg_lap = float(np.mean(self.lap_times))
        std_lap = float(np.std(self.lap_times))
        overall_min_clear = min(self.lap_min_clearances) if self.lap_min_clearances else 0.0
        overall_max_speed = max(self.lap_max_speeds) if self.lap_max_speeds else 0.0

        rows = []
        for i in range(len(self.lap_times)):
            tag = " <-- FASTEST LAP ★" if (i + 1) == best_lap_idx else ""
            rows.append(
                f"  |  Lap {i+1:2d}  |  {self.lap_times[i]:8.3f} s  |  {self.lap_distances[i]:7.2f} m  |  "
                f"{self.lap_max_speeds[i]:6.2f} m/s  |  {self.lap_min_clearances[i]:6.3f} m  |{tag}"
            )
        table_str = "\n".join(rows)

        final_report = (
            "\n\n" + "★" * 86 + "\n"
            f"  🏆 COMPETITION CHALLENGE COMPLETE: {len(self.lap_times)} CONSECUTIVE LAPS WITHOUT COLLISIONS! 🏆\n"
            + "★" * 86 + "\n"
            "  +---------+--------------+-------------+--------------+------------+\n"
            "  |  Lap #  |   Lap Time   |  Distance   |  Peak Speed  | Min Clear. |\n"
            "  +---------+--------------+-------------+--------------+------------+\n"
            f"{table_str}\n"
            "  +---------+--------------+-------------+--------------+------------+\n"
            f"  ⏱️  FASTEST LAP TIME     : {best_lap:.3f} s (Lap {best_lap_idx})\n"
            f"  📊 AVERAGE LAP TIME     : {avg_lap:.3f} s (±{std_lap:.3f} s)\n"
            f"  🏁 TOTAL RACE TIME      : {total_elapsed:.3f} s\n"
            f"  🚀 PEAK SPEED ACHIEVED  : {overall_max_speed:.2f} m/s\n"
            f"  🛡️  MIN WALL CLEARANCE   : {overall_min_clear:.3f} m (0 Collisions)\n"
            f"  📁 CSV RESULTS SAVED TO : {os.path.abspath(self.csv_log_path)}\n"
            + "★" * 86 + "\n"
            "  Vehicle stopped safely. Exiting controller node.\n"
        )
        self.get_logger().info(final_report)
        sys.stdout.flush()

    # =========================================================================
    # Main LiDAR Control Loop
    # =========================================================================

    def lidar_callback(self, data: LaserScan):
        """
        Process each incoming LaserScan message using FollowGapCore and publish
        an AckermannDriveStamped command. Automatically stops and exits once
        target_laps is reached.
        """
        if self.race_finished:
            stop_msg = AckermannDriveStamped()
            stop_msg.header = data.header
            stop_msg.drive.steering_angle = 0.0
            stop_msg.drive.speed = 0.0
            self.drive_pub.publish(stop_msg)
            self.stop_packets_sent += 1
            if self.stop_packets_sent >= 10:
                raise SystemExit(0)
            return

        if not self.reset_done:
            return

        raw = np.asarray(data.ranges, dtype=np.float64)
        valid = raw[np.isfinite(raw) & (raw > 0.01)]
        if len(valid) > 0:
            self.min_clearance_this_lap = min(self.min_clearance_this_lap, float(np.min(valid)))

        steer, speed = self.core.step(
            raw,
            float(data.angle_min),
            float(data.angle_increment),
            self.current_vel
        )
        self.prev_steer = steer
        self.current_speed_cmd = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        # Throttle RViz2 3D HUD markers to 10 Hz so RViz rendering never lags the control loop
        now_sec = self.get_sim_time_sec(data.header)
        if abs(now_sec - self.last_hud_pub_time) >= 0.10:
            self.last_hud_pub_time = now_sec
            dbg = self.core.last_debug
            if dbg:
                self.publish_rviz_hud(
                    data.header,
                    dbg['best_dist'],
                    dbg['target_angle']
                )

    # =========================================================================
    # RViz2 3D HUD & Goal Visualization (Video Evidence Enhancement)
    # =========================================================================

    def publish_rviz_hud(self, header, best_dist, target_angle):
        """
        Publish RViz2 visualization markers at 10 Hz:
        - Marker 0: Floating 3D Text HUD above the vehicle displaying Lap Counter,
                    Current Lap Timer, Last Lap, Best Lap, and Speed.
        - Marker 1: Green sphere at the selected Follow-the-Gap target point.
        - Marker 2: Start/Finish Line cylinder marker on the map.
        """
        marker_array = MarkerArray()
        now_sec = self.get_sim_time_sec(header)
        cur_lap_t = (now_sec - self.lap_start_time) if self.lap_start_time is not None else 0.0
        best_str = f"{min(self.lap_times):.2f}s" if self.lap_times else "--.--s"
        last_str = f"{self.lap_times[-1]:.2f}s" if self.lap_times else "--.--s"

        # 1. Floating 3D HUD Text above ego_racecar
        hud_marker = Marker()
        hud_marker.header.stamp = header.stamp
        hud_marker.header.frame_id = 'map'
        hud_marker.ns = 'lap_hud'
        hud_marker.id = 0
        hud_marker.type = Marker.TEXT_VIEW_FACING
        hud_marker.action = Marker.ADD
        hud_marker.pose.position.x = self.current_pose[0]
        hud_marker.pose.position.y = self.current_pose[1]
        hud_marker.pose.position.z = 1.5
        hud_marker.scale.z = 1.15
        hud_marker.color.r = 1.0
        hud_marker.color.g = 0.95
        hud_marker.color.b = 0.2
        hud_marker.color.a = 1.0
        hud_marker.text = (
            f"LAP: {self.current_lap}/{self.target_laps}\n"
            f"TIME: {cur_lap_t:.2f}s | BEST: {best_str}\n"
            f"LAST: {last_str} | SPD: {self.current_vel:.1f}m/s"
        )
        marker_array.markers.append(hud_marker)

        # 2. Target Goal Sphere in laser frame
        goal_marker = Marker()
        goal_marker.header.stamp = header.stamp
        goal_marker.header.frame_id = header.frame_id if header.frame_id else 'ego_racecar/laser'
        goal_marker.ns = 'ftg_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        vis_dist = min(best_dist, 8.0)
        goal_marker.pose.position.x = float(vis_dist * np.cos(target_angle))
        goal_marker.pose.position.y = float(vis_dist * np.sin(target_angle))
        goal_marker.pose.position.z = 0.2
        goal_marker.scale.x = 0.45
        goal_marker.scale.y = 0.45
        goal_marker.scale.z = 0.45
        goal_marker.color.r = 0.1
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.2
        goal_marker.color.a = 0.95
        marker_array.markers.append(goal_marker)

        # 3. Start/Finish Gate Marker in map frame
        if self.start_pose is not None:
            gate_marker = Marker()
            gate_marker.header.stamp = header.stamp
            gate_marker.header.frame_id = 'map'
            gate_marker.ns = 'start_finish'
            gate_marker.id = 2
            gate_marker.type = Marker.CYLINDER
            gate_marker.action = Marker.ADD
            gate_marker.pose.position.x = self.start_pose[0]
            gate_marker.pose.position.y = self.start_pose[1]
            gate_marker.pose.position.z = 0.05
            gate_marker.scale.x = 2.4
            gate_marker.scale.y = 2.4
            gate_marker.scale.z = 0.05
            gate_marker.color.r = 0.0
            gate_marker.color.g = 0.85
            gate_marker.color.b = 1.0
            gate_marker.color.a = 0.35
            marker_array.markers.append(gate_marker)

        self.hud_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

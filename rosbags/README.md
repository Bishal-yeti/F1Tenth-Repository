# 📦 F1Tenth ROS 2 Rosbag Archive (`Sakhir GP — 10 Collision-Free Laps`)

This directory contains the compressed ROS 2 Humble rosbag (`zstd`) recorded from a live autonomous 10-lap run on the **Bahrain Sakhir GP (`Sakhir_map`)** circuit using `gym_bridge` + `lidar_race_node`.

## 📁 Bag Contents

- **Bag Directory**: [`rosbags/sakhir_gp_10_laps/`](sakhir_gp_10_laps/)
- **Format**: ROS 2 SQLite3 (`db3`) with `zstd` file compression (`metadata.yaml` + split `*.db3.zstd` files kept under GitHub's 50 MB recommendation)
- **Recorded Topics**:
  - `/scan` (`sensor_msgs/msg/LaserScan`) — 1080-beam LiDAR scan stream
  - `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`) — AI controller steering & speed commands
  - `/ego_racecar/odom` (`nav_msgs/msg/Odometry`) — Vehicle pose, heading & velocity
  - `/tf` & `/tf_static` (`tf2_msgs/msg/TFMessage`) — Coordinate transforms
  - `/lap_hud_markers` (`visualization_msgs/msg/MarkerArray`) — Live RViz2 lap counter & timer HUD

## ▶️ Inspect & Replay

```bash
source /opt/ros/humble/setup.bash

# Inspect topics, message counts, and duration
ros2 bag info rosbags/sakhir_gp_10_laps

# Replay the 10-lap run
ros2 bag play rosbags/sakhir_gp_10_laps
```

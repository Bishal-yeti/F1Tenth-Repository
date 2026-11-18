#!/usr/bin/env bash
# ==============================================================================
# F1Tenth One-Command Beginner Launcher
# Usage:
#   ./run_race.sh            (Interactive menu)
#   ./run_race.sh sim        (Launch F1Tenth Simulator + RViz2)
#   ./run_race.sh lidar      (Run Stable 10-Stage Adaptive LiDAR Controller)
#   ./run_race.sh qualifier  (Run Fast-Start Qualifier Controller)
#   ./run_race.sh gap        (Run Version 1 Endurance Controller)
#   ./run_race.sh bench      (Run Headless 10-Lap Physics Benchmark)
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

# Auto-detect installed ROS 2 distro
for d in humble jazzy foxy galactic iron; do
  if [ -f "/opt/ros/$d/setup.bash" ]; then
    ROS_DISTRO="$d"
    break
  fi
done

if [ -z "$ROS_DISTRO" ]; then
  echo "❌ ROS 2 is not installed yet. Run './setup_f1tenth.sh' first!"
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$REPO_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/install/setup.bash"
else
  echo "⚠️  Workspace not built yet. Running 'colcon build --symlink-install'..."
  colcon build --symlink-install
  # shellcheck disable=SC1091
  source "$REPO_DIR/install/setup.bash"
fi

ACTION="$1"
if [ -z "$ACTION" ]; then
  echo "=============================================================================="
  echo "  🏎️  F1TENTH AUTONOMOUS RACING LAUNCHER"
  echo "=============================================================================="
  echo "  [1] sim       — Launch F1Tenth Simulator + RViz2 (Terminal 1)"
  echo "  [2] lidar     — Run Version 2: Stable 10-Stage Adaptive LiDAR + GUI Window"
  echo "  [3] qualifier — Run Version 3: Fast-Start Qualifier Mode + GUI Window"
  echo "  [4] gap       — Run Version 1: Endurance Fast-Centerline Controller"
  echo "  [5] bench     — Run Headless 10-Lap Physics Benchmark (No RViz2)"
  echo "  [6] map       — Switch Track Map (Choose from all 12 official circuits)"
  echo "=============================================================================="
  read -r -p "Choose an option [1-6]: " choice
  case "$choice" in
    1) ACTION="sim" ;;
    2) ACTION="lidar" ;;
    3) ACTION="qualifier" ;;
    4) ACTION="gap" ;;
    5) ACTION="bench" ;;
    6) ACTION="map" ;;
    *) echo "❌ Invalid choice."; exit 1 ;;
  esac
fi

case "$ACTION" in
  sim)       exec ros2 launch f1tenth_gym_ros gym_bridge_launch.py ;;
  lidar)     exec ros2 run controllers lidar_race_node ;;
  qualifier) exec ros2 run controllers qualifier_node ;;
  gap)       exec ros2 run controllers gap_node ;;
  bench)     exec python3 "$REPO_DIR/benchmark_sakhir_lidar.py" ;;
  map)       exec "$REPO_DIR/switch_map.sh" ;;
  *)         echo "Usage: ./run_race.sh [sim|lidar|qualifier|gap|bench|map]"; exit 1 ;;
esac

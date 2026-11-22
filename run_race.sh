#!/usr/bin/env bash
# ==============================================================================
# 🏎️  F1Tenth Autonomous Racing Master Launcher
# Zero-Search, Step-by-Step Autonomous Racing Suite
#
# Usage:
#   ./run_race.sh            (Interactive menu with 1-click All-in-One default)
#   ./run_race.sh race       (★ ALL-IN-ONE: Sim + RViz + AI Controller in 1 command!)
#   ./run_race.sh sim        (Terminal 1: Launch Simulator + RViz2 alone)
#   ./run_race.sh lidar      (Terminal 2: Run Stable 10-Stage Adaptive AI)
#   ./run_race.sh qualifier  (Terminal 2: Run Fast-Start Qualifier Mode)
#   ./run_race.sh gap        (Terminal 2: Run Version 1 Endurance AI)
#   ./run_race.sh bench      (Headless 10-Lap Physics Benchmark - No RViz)
#   ./run_race.sh map        (Interactive 12-Track Circuit Switcher)
#   ./run_race.sh guide      (View Step-by-Step Run, Reflect & Improve Guide)
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
  echo "❌ ROS 2 is not detected. Run './setup_f1tenth.sh' first to install everything!"
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ -f "$REPO_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/install/setup.bash"
else
  echo "⚠️  Workspace not built yet. Building ROS 2 workspace (colcon build)..."
  colcon build --symlink-install
  # shellcheck disable=SC1091
  source "$REPO_DIR/install/setup.bash"
fi

# Read current active track name
ACTIVE_MAP="Sakhir_map"
if [ -f "$REPO_DIR/src/f1tenth_gym_ros/config/sim.yaml" ]; then
  ACTIVE_MAP="$(grep "map_path:" "$REPO_DIR/src/f1tenth_gym_ros/config/sim.yaml" | head -n1 | sed -e "s/.*map_path:[ ']*//" -e "s/['\"].*//" | xargs basename)"
fi

# Auto-check & fix RViz2 black map patch (WSL2 d3d12 driver + GLSL 120 dual-sampler + rviz config sync)
ensure_rviz_map_ready() {
  # 1. On WSL2, Mesa's d3d12 (OpenGL-over-DirectX12) driver fails to render Ogre's 8-bit
  #    GL_LUMINANCE8 map texture (producing a black square). Force Mesa's llvmpipe rasterizer.
  if [ -d /mnt/wslg ] || grep -qi microsoft /proc/version 2>/dev/null; then
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
  fi

  # 2. Apply single-sampler GLSL 1.20 shader fix
  local frag="/opt/ros/$ROS_DISTRO/share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag"
  local mat="/opt/ros/$ROS_DISTRO/share/rviz_rendering/ogre_media/materials/scripts/indexed_8bit_image.material"
  if [ -f "$frag" ] && ! grep -q "Single-sampler GLSL 120" "$frag"; then
    echo "🔧 Applying universal RViz2 Map shader fix (prevents black patch on fresh PCs)..."
    sudo bash -c "cat << 'EOF' > \"$frag\"
#version 120

// Single-sampler GLSL 120 shader (eliminates sampler2D/sampler1D unit 0 link conflict on all GPUs)
varying vec2 UV;
uniform sampler2D eight_bit_image;
uniform float alpha;

void main()
{
  float raw = texture2D( eight_bit_image, UV ).x * 255.0;
  vec3 rgb;
  if (raw <= 100.5) {
    float g = clamp(1.0 - (raw / 100.0), 0.0, 1.0);
    rgb = vec3(g, g, g);
  } else {
    rgb = vec3(0.804, 0.804, 0.804);
  }
  gl_FragColor = vec4( rgb, alpha );
}
EOF"
    if [ -f "$mat" ]; then
      sudo sed -i 's/^\([[:space:]]*\)param_named palette int 1/\1\/\/ param_named palette int 1/' "$mat" || true
    fi
  fi

  # 3. Sync source gym_bridge.rviz and launch files to build/ and install/ (even if symlinked to build/)
  local src_rviz="$REPO_DIR/src/f1tenth_gym_ros/launch/gym_bridge.rviz"
  local build_rviz="$REPO_DIR/build/f1tenth_gym_ros/launch/gym_bridge.rviz"
  local inst_rviz="$REPO_DIR/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz"
  if [ -f "$src_rviz" ]; then
    [ -d "$(dirname "$build_rviz")" ] && cp -f "$src_rviz" "$build_rviz" 2>/dev/null || true
    [ -d "$(dirname "$inst_rviz")" ] && cp -f "$src_rviz" "$inst_rviz" 2>/dev/null || true
  fi
  for lf in gym_bridge_launch.py race_launch.py; do
    local src_lf="$REPO_DIR/src/f1tenth_gym_ros/launch/$lf"
    local build_lf="$REPO_DIR/build/f1tenth_gym_ros/launch/$lf"
    local inst_lf="$REPO_DIR/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/$lf"
    if [ -f "$src_lf" ]; then
      [ -d "$(dirname "$build_lf")" ] && cp -f "$src_lf" "$build_lf" 2>/dev/null || true
      [ -d "$(dirname "$inst_lf")" ] && cp -f "$src_lf" "$inst_lf" 2>/dev/null || true
    fi
  done
  # 4. Sync updated Python nodes (controllers & gym_bridge) to build/ if build/ uses copied files
  if [ -d "$REPO_DIR/build/controllers/controllers" ]; then
    cp -f "$REPO_DIR/src/controllers/controllers/"*.py "$REPO_DIR/build/controllers/controllers/" 2>/dev/null || true
  fi
  if [ -d "$REPO_DIR/build/f1tenth_gym_ros/f1tenth_gym_ros" ]; then
    cp -f "$REPO_DIR/src/f1tenth_gym_ros/f1tenth_gym_ros/"*.py "$REPO_DIR/build/f1tenth_gym_ros/f1tenth_gym_ros/" 2>/dev/null || true
  fi
}

ACTION="$1"
if [ -z "$ACTION" ]; then
  echo "=============================================================================="
  echo "  🏎️  F1TENTH AUTONOMOUS RACING LAUNCHER (ZERO-SEARCH SUITE)"
  echo "=============================================================================="
  echo "  Active Track : $ACTIVE_MAP  (Use Option [7] or './switch_map.sh' to change)"
  echo "  ROS 2 Distro : $ROS_DISTRO"
  echo "------------------------------------------------------------------------------"
  echo "  HOW TO RUN:"
  echo "  Option [1] (Recommended): ONE COMMAND! Launches the Simulator, RViz2 3D HUD,"
  echo "             and Autonomous AI Controller together. No second terminal needed!"
  echo ""
  echo "  Options [2] & [3]: TWO-TERMINAL WORKFLOW (Standard ROS 2 style):"
  echo "             Terminal 1 -> Run './run_race.sh sim'"
  echo "             Terminal 2 -> Run './run_race.sh lidar'"
  echo "------------------------------------------------------------------------------"
  echo "  [1] race      ★ ALL-IN-ONE (Recommended): Sim + RViz + AI Controller (1-Click)"
  echo "  [2] sim       - Terminal 1: Launch Simulator + RViz2 alone"
  echo "  [3] lidar     - Terminal 2: Run Stable 10-Stage Adaptive AI Controller"
  echo "  [4] qualifier - Terminal 2: Run Fast-Start Qualifier AI Mode (Q-T6 -> Q-T8)"
  echo "  [5] gap       - Terminal 2: Run Version 1 Endurance AI Controller"
  echo "  [6] bench     - Run Headless 10-Lap Physics Benchmark (Fast, No GUI)"
  echo "  [7] map       - Switch Circuit Track (Choose from 12 official circuits)"
  echo "  [8] guide     - Display 'Run, Reflect & Improve' Guide & Tuning Cheatsheet"
  echo "=============================================================================="
  read -r -p "Choose an option [1-8] (Default is 1: All-in-One Race): " choice
  case "$choice" in
    ""|1) ACTION="race" ;;
    2)    ACTION="sim" ;;
    3)    ACTION="lidar" ;;
    4)    ACTION="qualifier" ;;
    5)    ACTION="gap" ;;
    6)    ACTION="bench" ;;
    7)    ACTION="map" ;;
    8)    ACTION="guide" ;;
    *)    echo "❌ Invalid choice."; exit 1 ;;
  esac
fi

case "$ACTION" in
  race)
    ensure_rviz_map_ready
    echo "=============================================================================="
    echo "  🏎️  STARTING ALL-IN-ONE AUTONOMOUS RACE (SIMULATOR + RVIZ2 + AI CONTROLLER)"
    echo "=============================================================================="
    echo "  Track : $ACTIVE_MAP"
    echo "  AI    : Stable 10-Stage Adaptive LiDAR (V2)"
    echo "  Press Ctrl+C at any time to stop everything."
    echo "=============================================================================="
    exec ros2 launch f1tenth_gym_ros race_launch.py controller:=lidar
    ;;

  sim)
    ensure_rviz_map_ready
    echo ""
    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║  🏎️  STEP 1/2: F1TENTH SIMULATOR & RVIZ2 STARTING (TERMINAL 1)                 ║"
    echo "╠══════════════════════════════════════════════════════════════════════════════╣"
    echo "║  ⚠️  IMPORTANT NOTICE:                                                       ║"
    echo "║      The racecar will NOT move yet! It is waiting at the start line for an   ║"
    echo "║      autonomous controller to publish steering and speed commands.           ║"
    echo "║                                                                              ║"
    echo "║  👉 STEP 2: OPEN A SECOND TERMINAL WINDOW (OR SPLIT PANE) AND RUN:           ║"
    echo "║         cd $REPO_DIR"
    echo "║         ./run_race.sh lidar      # Stable 10-Stage Adaptive AI (Recommended) ║"
    echo "║      OR:                                                                     ║"
    echo "║         ./run_race.sh qualifier  # Fast-Start Qualifier AI (Q-T6->Q-T8)      ║"
    echo "║         ./run_race.sh gap        # Version 1 Endurance AI                    ║"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"
    echo ""
    sleep 2
    exec ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ;;

  lidar)
    ensure_rviz_map_ready
    echo "=============================================================================="
    echo "  🏎️  LAUNCHING STABLE 10-STAGE ADAPTIVE LIDAR AI CONTROLLER (TERMINAL 2)"
    echo "=============================================================================="
    echo "  If the car does not move, ensure the simulator is running in Terminal 1:"
    echo "  👉 Run './run_race.sh sim' in another terminal"
    echo "=============================================================================="
    exec ros2 run controllers lidar_race_node
    ;;

  qualifier)
    ensure_rviz_map_ready
    echo "=============================================================================="
    echo "  ⚡ LAUNCHING FAST-START QUALIFIER TIME-ATTACK CONTROLLER (TERMINAL 2)"
    echo "=============================================================================="
    echo "  If the car does not move, ensure the simulator is running in Terminal 1:"
    echo "  👉 Run './run_race.sh sim' in another terminal"
    echo "=============================================================================="
    exec ros2 run controllers qualifier_node
    ;;

  gap)
    ensure_rviz_map_ready
    echo "=============================================================================="
    echo "  🛡️  LAUNCHING VERSION 1 ENDURANCE CENTERLINE CONTROLLER (TERMINAL 2)"
    echo "=============================================================================="
    echo "  If the car does not move, ensure the simulator is running in Terminal 1:"
    echo "  👉 Run './run_race.sh sim' in another terminal"
    echo "=============================================================================="
    exec ros2 run controllers gap_node
    ;;

  bench)
    exec python3 "$REPO_DIR/benchmark_sakhir_lidar.py"
    ;;

  map)
    exec "$REPO_DIR/switch_map.sh"
    ;;

  guide)
    if [ -f "$REPO_DIR/GUIDE_RUN_REFLECT_IMPROVE.md" ]; then
      if command -v less >/dev/null 2>&1; then
        less "$REPO_DIR/GUIDE_RUN_REFLECT_IMPROVE.md"
      else
        cat "$REPO_DIR/GUIDE_RUN_REFLECT_IMPROVE.md"
      fi
    else
      echo "Guide file: $REPO_DIR/GUIDE_RUN_REFLECT_IMPROVE.md"
    fi
    ;;

  *)
    echo "Usage: ./run_race.sh [race|sim|lidar|qualifier|gap|bench|map|guide]"
    exit 1
    ;;
esac

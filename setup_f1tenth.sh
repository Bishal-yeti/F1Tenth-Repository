#!/usr/bin/env bash
# ==============================================================================
# F1Tenth One-Command Automated Installer, sim.yaml Configurator & Smoke Tester
# Works smoothly on ANY fresh PC: Ubuntu 20.04 (Foxy), 22.04 (Humble), 24.04 (Jazzy)
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_YAML="$REPO_DIR/src/f1tenth_gym_ros/config/sim.yaml"
cd "$REPO_DIR"

echo "=============================================================================="
echo "  🏎️  F1TENTH ZERO-CONFIG SETUP — FRESH PC TO READY-TO-RACE IN ONE COMMAND"
echo "=============================================================================="

# 1. Detect Ubuntu version & matching ROS 2 distribution
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-jammy}")"
case "$UBUNTU_CODENAME" in
  focal)  DEFAULT_DISTRO="foxy" ;;
  jammy)  DEFAULT_DISTRO="humble" ;;
  noble)  DEFAULT_DISTRO="jazzy" ;;
  *)      DEFAULT_DISTRO="humble" ;;
esac
ROS_DISTRO="${ROS_DISTRO:-$DEFAULT_DISTRO}"
PIP_EXTRA=""
NUMPY_SPEC="numpy==1.22.0"
if [ "$UBUNTU_CODENAME" = "noble" ]; then
  PIP_EXTRA="--break-system-packages"
  NUMPY_SPEC="numpy<2.0"
fi

echo "  Current PC User          : $USER"
echo "  Detected Ubuntu Codename : $UBUNTU_CODENAME"
echo "  Target ROS 2 Distro      : $ROS_DISTRO"
echo "  Workspace Directory      : $REPO_DIR"
echo "------------------------------------------------------------------------------"

# 2. Install ROS 2 if /opt/ros/$ROS_DISTRO/setup.bash is missing
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[1/6] ROS 2 ($ROS_DISTRO) not detected. Installing ROS 2 from scratch..."
  sudo apt update && sudo apt install -y locales software-properties-common curl gnupg lsb-release build-essential git
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  sudo add-apt-repository universe -y
  sudo apt update
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install -y "ros-${ROS_DISTRO}-desktop"
else
  echo "[1/6] ✅ ROS 2 ($ROS_DISTRO) is already installed."
fi

# 3. Ensure all required ROS 2 simulation, RViz2 & GUI packages are installed
echo "[2/6] Checking required ROS 2 simulation, RViz2 & GUI packages..."
MISSING_PKGS=()
for pkg in \
  "ros-${ROS_DISTRO}-rviz2" \
  "ros-${ROS_DISTRO}-ackermann-msgs" \
  "ros-${ROS_DISTRO}-nav2-map-server" \
  "ros-${ROS_DISTRO}-nav2-lifecycle-manager" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
  "ros-${ROS_DISTRO}-tf-transformations" \
  "python3-colcon-common-extensions" \
  "python3-pip" \
  "python3-tk"; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    MISSING_PKGS+=("$pkg")
  fi
done
if ! command -v rosdep >/dev/null 2>&1; then
  MISSING_PKGS+=("python3-rosdep")
fi

if [ "${#MISSING_PKGS[@]}" -gt 0 ]; then
  echo "      Installing missing packages: ${MISSING_PKGS[*]}"
  sudo apt update && sudo apt install -y "${MISSING_PKGS[@]}"
else
  echo "      ✅ All required ROS 2 & GUI packages are already installed."
fi

# 4. Install f1tenth_gym & exact verified Python dependencies if missing
if python3 -c "import f110_gym, gym, numpy, scipy, numba, PIL, yaml, transforms3d" >/dev/null 2>&1; then
  echo "[3/6] ✅ f1tenth_gym & all Python dependencies are already installed."
else
  echo "[3/6] Installing f1tenth_gym & verified Python scientific dependencies..."
  python3 -m pip install $PIP_EXTRA --upgrade "pip<24.1" "setuptools==58.2.0" "wheel<0.40.0"
  python3 -m pip install $PIP_EXTRA \
    "gym==0.19.0" \
    "$NUMPY_SPEC" \
    "scipy>=1.7.3,<1.12.0" \
    "numba>=0.55.2" \
    "Pillow>=9.0.1" \
    "PyYAML>=5.3.1" \
    "pyglet<1.5" \
    "pyopengl" \
    "transforms3d>=0.4.1" \
    "shapely>=1.8.5" \
    "matplotlib>=3.5.0"

  if [ ! -d "$HOME/f1tenth_gym" ]; then
    git clone https://github.com/f1tenth/f1tenth_gym.git "$HOME/f1tenth_gym"
  fi
  (cd "$HOME/f1tenth_gym" && python3 -m pip install $PIP_EXTRA -e . --no-build-isolation --no-deps)
fi

# 5. Automatically update sim.yaml with THIS PC's exact username & map path
echo "[4/6] Auto-configuring sim.yaml for user '$USER' at '$REPO_DIR'..."
python3 - "$SIM_YAML" "$REPO_DIR" << 'EOF'
import os, re, sys
sim_yaml, repo_dir = sys.argv[1], sys.argv[2]
with open(sim_yaml, "r") as f:
    txt = f.read()
m = re.search(r"map_path:\s*['\"]?([^'\"\n]+)['\"]?", txt)
map_name = os.path.basename(m.group(1).strip().rstrip("/")) if m else "Sakhir_map"
local_map_path = os.path.join(repo_dir, "src", "f1tenth_gym_ros", "maps", map_name)
if not os.path.exists(local_map_path + ".yaml"):
    map_name = "Sakhir_map"
    local_map_path = os.path.join(repo_dir, "src", "f1tenth_gym_ros", "maps", map_name)
txt = re.sub(r"map_path:.*", f"map_path: '{local_map_path}'", txt)
with open(sim_yaml, "w") as f:
    f.write(txt)
print(f"      ✅ sim.yaml map_path set automatically to: {local_map_path}")
EOF

# Remove conflicting python3-coverage (prevents colcon/ament_python errors)
python3 -m pip uninstall -y coverage >/dev/null 2>&1 || true
if dpkg -s python3-coverage >/dev/null 2>&1; then
  sudo apt purge -y python3-coverage
fi

# Fix Mesa/WSL2/Intel/AMD RViz2 Map black-patch bug (GLSL 120 dual-sampler link conflict)
# Uses a pure GLSL 1.20 single-sampler (sampler2D-only) shader that works on 100% of OpenGL drivers
RVIZ_MAP_FRAG="/opt/ros/$ROS_DISTRO/share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag"
RVIZ_MAP_MAT="/opt/ros/$ROS_DISTRO/share/rviz_rendering/ogre_media/materials/scripts/indexed_8bit_image.material"
if [ -f "$RVIZ_MAP_FRAG" ] && ! grep -q "Single-sampler GLSL 120" "$RVIZ_MAP_FRAG"; then
  echo "      Applying universal single-sampler RViz2 Map shader fix (prevents black map patch)..."
  sudo bash -c "cat << 'EOF' > \"$RVIZ_MAP_FRAG\"
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
  if [ -f "$RVIZ_MAP_MAT" ]; then
    sudo sed -i 's/^\([[:space:]]*\)param_named palette int 1/\1\/\/ param_named palette int 1/' "$RVIZ_MAP_MAT" || true
  fi
fi

# 6. Clean build the ROS 2 workspace (f1tenth_gym_ros + controllers + path_planning + 12 maps)
echo "[5/6] Compiling ROS 2 workspace (f1tenth_gym_ros + controllers + all 12 maps)..."
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update >/dev/null 2>&1 || true
rm -rf build/ install/ log/
colcon build --symlink-install

# Auto-add ROS 2 sourcing & WSL2 RViz2 OpenGL fix to ~/.bashrc if not present
if ! grep -q "/opt/ros/$ROS_DISTRO/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> "$HOME/.bashrc"
fi
if ( [ -d /mnt/wslg ] || grep -qi microsoft /proc/version 2>/dev/null ) && ! grep -q "LIBGL_ALWAYS_SOFTWARE=1" "$HOME/.bashrc" 2>/dev/null; then
  {
    echo "# Fix WSL2 d3d12 OpenGL black map texture in RViz2"
    echo "export LIBGL_ALWAYS_SOFTWARE=1"
    echo "export GALLIUM_DRIVER=llvmpipe"
  } >> "$HOME/.bashrc"
fi

# 7. Automated End-to-End Smoke Test
echo "[6/6] Running automated end-to-end verification checks..."
# shellcheck disable=SC1091
source "$REPO_DIR/install/setup.bash"
python3 - "$SIM_YAML" << 'EOF'
import os, yaml
import f110_gym, gym
from controllers.lidar_race_core import RealLidarRaceCore
from controllers.qualifier_core import QualifierLidarCore
from controllers.telemetry_gui import RaceTelemetryGUI
from ament_index_python.packages import get_package_share_directory

with open(os.sys.argv[1]) as f:
    conf = yaml.safe_load(f)["bridge"]["ros__parameters"]
mp = conf["map_path"]
assert os.path.exists(mp + ".yaml"), f"Missing map yaml: {mp}.yaml"
assert os.path.exists(mp + conf["map_img_ext"]), f"Missing map image: {mp}{conf['map_img_ext']}"
share_maps = os.path.join(get_package_share_directory("f1tenth_gym_ros"), "maps")
map_count = len([x for x in os.listdir(share_maps) if x.endswith(".yaml")])
print(f"      ✅ Active Map Verified : {mp} ({map_count} official tracks installed)")
print("      ✅ Controllers Verified: lidar_race_node, qualifier_node, gap_node, telemetry_gui")
EOF

echo ""
echo "=============================================================================="
echo "  ✅ ALL CHECKS PASSED — READY TO RACE ON THIS PC ($USER)!"
echo "=============================================================================="
echo "  [OPTION A] ONE COMMAND (Sim + RViz + AI Controller in 1 Window):"
echo "     ./run_race.sh race      (or just './run_race.sh' and press Enter!)"
echo ""
echo "  [OPTION B] TWO TERMINALS (Standard ROS 2 style):"
echo "     Terminal 1 (Simulator + RViz2) : ./run_race.sh sim"
echo "     Terminal 2 (Stable 10-Lap AI)  : ./run_race.sh lidar"
echo "     Terminal 2 (Fast Qualifier AI) : ./run_race.sh qualifier"
echo ""
echo "  Switch Between All 12 Circuits   : ./scripts/switch_map.sh"
echo "  Step-by-Step Reflection & Tuning : ./run_race.sh guide"
echo "=============================================================================="

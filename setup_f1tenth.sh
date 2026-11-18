#!/usr/bin/env bash
# ==============================================================================
# F1Tenth One-Command Automated Installer & Workspace Builder
# Works on Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble), and Ubuntu 24.04 (Jazzy)
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

echo "=============================================================================="
echo "  🏎️  F1TENTH AUTOMATED SETUP & BUILDER (Zero-Config Beginner Installer)"
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
if [ "$UBUNTU_CODENAME" = "noble" ]; then
  PIP_EXTRA="--break-system-packages"
fi

echo "  Detected Ubuntu Codename : $UBUNTU_CODENAME"
echo "  Target ROS 2 Distro      : $ROS_DISTRO"
echo "  Workspace Directory      : $REPO_DIR"
echo "------------------------------------------------------------------------------"

# 2. Install ROS 2 if /opt/ros/$ROS_DISTRO/setup.bash is missing
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[1/5] ROS 2 ($ROS_DISTRO) not detected. Installing ROS 2 from scratch..."
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
  echo "[1/5] ✅ ROS 2 ($ROS_DISTRO) is already installed."
fi

# 3. Ensure required ROS 2 simulation & GUI packages are installed
echo "[2/5] Checking required ROS 2 simulation & GUI packages..."
MISSING_PKGS=()
for pkg in \
  "ros-${ROS_DISTRO}-ackermann-msgs" \
  "ros-${ROS_DISTRO}-nav2-map-server" \
  "ros-${ROS_DISTRO}-nav2-lifecycle-manager" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
  "python3-colcon-common-extensions" \
  "python3-rosdep" \
  "python3-pip" \
  "python3-tk"; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    MISSING_PKGS+=("$pkg")
  fi
done

if [ "${#MISSING_PKGS[@]}" -gt 0 ]; then
  echo "      Installing missing packages: ${MISSING_PKGS[*]}"
  sudo apt update && sudo apt install -y "${MISSING_PKGS[@]}"
else
  echo "      ✅ All ROS 2 simulation & GUI packages are already installed."
fi

# 4. Install f1tenth_gym if not importable
if python3 -c "import f110_gym" >/dev/null 2>&1; then
  echo "[3/5] ✅ f1tenth_gym Python physics engine is already installed."
else
  echo "[3/5] Installing f1tenth_gym & compatible Python packages..."
  python3 -m pip install $PIP_EXTRA --upgrade "pip<24.1" "setuptools==58.2.0" "wheel<0.40.0"
  python3 -m pip install $PIP_EXTRA \
    "gym==0.19.0" "numpy==1.23.5" "scipy>=1.8.0" "numba>=0.56.0" \
    "Pillow>=9.0.0" "PyYAML>=6.0" "transforms3d>=0.4.1" "shapely>=1.8.5" "matplotlib>=3.5.0"

  if [ ! -d "$HOME/f1tenth_gym" ]; then
    git clone https://github.com/f1tenth/f1tenth_gym.git "$HOME/f1tenth_gym"
  fi
  (cd "$HOME/f1tenth_gym" && python3 -m pip install $PIP_EXTRA -e . --no-build-isolation)
fi

# 5. Remove conflicting python3-coverage (prevents colcon/ament_python errors)
echo "[4/5] Cleaning conflicting packages & initializing rosdep..."
python3 -m pip uninstall -y coverage >/dev/null 2>&1 || true
if dpkg -s python3-coverage >/dev/null 2>&1; then
  sudo apt purge -y python3-coverage
fi

# Source ROS 2 and run colcon build
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update >/dev/null 2>&1 || true

echo "[5/5] Building ROS 2 workspace (f1tenth_gym_ros + controllers + all 12 maps)..."
rm -rf build/ install/ log/
colcon build --symlink-install

# Auto-add ROS 2 sourcing to ~/.bashrc if not present
if ! grep -q "/opt/ros/$ROS_DISTRO/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> "$HOME/.bashrc"
fi

echo ""
echo "=============================================================================="
echo "  ✅ SETUP COMPLETE! Zero manual sim.yaml username edits required!"
echo "=============================================================================="
echo "  Next Steps:"
echo "    Terminal 1 (Simulator + RViz2) : ./run_race.sh sim"
echo "    Terminal 2 (Stable 10-Lap AI)  : ./run_race.sh lidar"
echo "    Terminal 2 (Fast Qualifier AI) : ./run_race.sh qualifier"
echo "    Switch Between All 12 Tracks   : ./switch_map.sh"
echo "=============================================================================="

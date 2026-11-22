#!/usr/bin/env bash
# ==============================================================================
# F1Tenth One-Command Interactive Map Switcher (All 12 Bundled Maps!)
# Automatically updates sim.yaml with the verified starting pose & rebuilds!
# Usage:
#   ./switch_map.sh                 (Interactive menu)
#   ./switch_map.sh Zandvoort_map   (Direct CLI switch)
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_YAML="$REPO_DIR/src/f1tenth_gym_ros/config/sim.yaml"

# Verified drivable starting poses: "MapName|sx|sy|stheta|Description"
MAPS=(
  "Sakhir_map|30.78|-22.72|-2.635|Bahrain Sakhir GP (Default Competition Track)"
  "Catalunya_map|0.0|0.0|-2.145|Circuit de Barcelona-Catalunya (Spain)"
  "Sepang_map|0.0|0.0|0.096|Sepang International Circuit (Malaysia)"
  "Zandvoort_map|0.0|0.0|1.220|Circuit Zandvoort (Netherlands)"
  "BrandsHatch_map|0.0|0.0|0.279|Brands Hatch Circuit (UK)"
  "Budapest_map|0.0|0.0|-0.618|Hungaroring / Budapest (Hungary)"
  "Oschersleben_map|0.0|0.0|-0.248|Motorsport Arena Oschersleben (Germany)"
  "SaoPaulo_map|0.0|0.0|-1.258|Interlagos / Sao Paulo (Brazil)"
  "Spielberg_map|0.0|0.0|0.000|Red Bull Ring / Spielberg (Austria)"
  "Spielberg_obs_map|0.0|0.0|0.000|Spielberg with Obstacles"
  "levine|0.0|0.0|0.000|UPenn Levine Hall (Indoor Loop)"
  "levineB|0.0|0.0|0.000|UPenn Levine Hall Variant B"
)

SELECTED=""
if [ -n "$1" ]; then
  for entry in "${MAPS[@]}"; do
    name="${entry%%|*}"
    if [ "$name" = "$1" ] || [ "${name,,}" = "${1,,}" ] || [ "${name%_map}" = "$1" ]; then
      SELECTED="$entry"
      break
    fi
  done
  if [ -z "$SELECTED" ]; then
    echo "❌ Unknown map '$1'. Available maps:"
    for entry in "${MAPS[@]}"; do
      echo "   - ${entry%%|*}"
    done
    exit 1
  fi
else
  echo "=============================================================================="
  echo "  🗺️  F1TENTH TRACK SELECTOR — ALL 12 OFFICIAL MAPS INCLUDED"
  echo "=============================================================================="
  i=1
  for entry in "${MAPS[@]}"; do
    IFS='|' read -r name sx sy stheta desc <<< "$entry"
    printf "  [%2d] %-20s — %s\n" "$i" "$name" "$desc"
    i=$((i + 1))
  done
  echo "=============================================================================="
  read -r -p "Select track number [1-${#MAPS[@]}] (default 1 = Sakhir_map): " choice
  choice="${choice:-1}"
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#MAPS[@]}" ]; then
    echo "❌ Invalid selection."
    exit 1
  fi
  SELECTED="${MAPS[$((choice - 1))]}"
fi

IFS='|' read -r MAP_NAME SX SY STHETA DESC <<< "$SELECTED"

python3 - "$SIM_YAML" "$REPO_DIR/src/f1tenth_gym_ros/maps/$MAP_NAME" "$SX" "$SY" "$STHETA" "$REPO_DIR/src/f1tenth_gym_ros/launch/gym_bridge.rviz" "$MAP_NAME" << 'EOF'
import sys, re, os
yaml_path, full_map_path, sx, sy, stheta, rviz_path, map_name = sys.argv[1:8]
with open(yaml_path, "r") as f:
    txt = f.read()
txt = re.sub(r"map_path:.*", f"map_path: '{full_map_path}'", txt)
txt = re.sub(r"^\s*sx:.*$", f"    sx: {sx}", txt, flags=re.MULTILINE)
txt = re.sub(r"^\s*sy:.*$", f"    sy: {sy}", txt, flags=re.MULTILINE)
txt = re.sub(r"^\s*stheta:.*$", f"    stheta: {stheta}", txt, flags=re.MULTILINE)
with open(yaml_path, "w") as f:
    f.write(txt)

if os.path.exists(rviz_path):
    with open(rviz_path, "r") as f:
        rtxt = f.read()
    if map_name == "Sakhir_map":
        fx, fy, dist = "30.25", "12.16", "145"
    else:
        fx, fy, dist = "0", "0", "110"
    rtxt = re.sub(
        r"(Views:\s+Current:\s+Class: rviz_default_plugins/Orbit\s+Distance:\s*)[0-9.]+",
        rf"\g<1>{dist}",
        rtxt,
    )
    rtxt = re.sub(
        r"(Focal Point:\s+X:\s*)[-0-9.]+(\s+Y:\s*)[-0-9.]+",
        rf"\g<1>{fx}\g<2>{fy}",
        rtxt,
    )
    with open(rviz_path, "w") as f:
        f.write(rtxt)
EOF

echo "✅ Updated sim.yaml -> Map: $REPO_DIR/src/f1tenth_gym_ros/maps/$MAP_NAME | Pose: (sx=$SX, sy=$SY, stheta=$STHETA)"

# Rebuild f1tenth_gym_ros so installed config/sim.yaml is updated
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  (cd "$REPO_DIR" && colcon build --symlink-install --packages-select f1tenth_gym_ros >/dev/null)
  echo "✅ Rebuilt f1tenth_gym_ros. Ready to race on $MAP_NAME!"
  echo "👉 Launch with: ./run_race.sh race (or Terminal 1: ./run_race.sh sim & Terminal 2: ./run_race.sh lidar)"
fi

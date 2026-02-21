#!/usr/bin/env bash
set -euo pipefail

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Autonav Setup ===${NC}"

# Check ROS 2
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Error: ROS 2 is not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo -e "${YELLOW}Warning: Expected ROS 2 Humble, got $ROS_DISTRO${NC}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

prune_nonexistent_paths() {
    local var_name="$1"
    local current_value="${!var_name:-}"
    local cleaned_value=""
    local path_entry

    if [ -z "$current_value" ]; then
        return
    fi

    IFS=':' read -ra path_entries <<< "$current_value"
    for path_entry in "${path_entries[@]}"; do
        if [ -n "$path_entry" ] && [ -d "$path_entry" ]; then
            cleaned_value="${cleaned_value:+$cleaned_value:}$path_entry"
        fi
    done

    export "$var_name=$cleaned_value"
}

# Install apt dependencies
echo -e "${GREEN}[1/5] Installing apt dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-foxglove-bridge \
    ros-humble-image-transport-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-compressed-image-transport \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy

# Install Python dependencies for motor driver
echo -e "${GREEN}[1.5/5] Installing Python dependencies...${NC}"
python3 -m pip install --user adafruit-circuitpython-pca9685 adafruit-extended-bus

# Clone ZED ROS 2 wrapper
echo -e "${GREEN}[2/5] Setting up ZED ROS 2 wrapper...${NC}"
if [ ! -d "src/zed-ros2-wrapper" ]; then
    cd src
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    cd ..
else
    echo "zed-ros2-wrapper already exists, skipping clone"
fi

# Optional pin for deterministic SDK/wrapper compatibility
if [ -n "${ZED_ROS2_WRAPPER_REF:-}" ]; then
    echo "Pinning zed-ros2-wrapper to ${ZED_ROS2_WRAPPER_REF}"
    git -C src/zed-ros2-wrapper fetch --tags
    git -C src/zed-ros2-wrapper checkout "${ZED_ROS2_WRAPPER_REF}"
else
    echo -e "${YELLOW}Warning: ZED_ROS2_WRAPPER_REF is not set; using current zed-ros2-wrapper checkout.${NC}"
fi

# Install rosdep dependencies
echo -e "${GREEN}[3/5] Installing rosdep dependencies...${NC}"
if ! command -v rosdep &> /dev/null; then
    sudo apt install -y python3-rosdep
    sudo rosdep init || true
fi
rosdep update --rosdistro=humble || true
rosdep install --from-paths src --ignore-src -r -y

# Build
echo -e "${GREEN}[4/5] Building workspace...${NC}"
source /opt/ros/humble/setup.bash
prune_nonexistent_paths AMENT_PREFIX_PATH
prune_nonexistent_paths CMAKE_PREFIX_PATH
prune_nonexistent_paths COLCON_PREFIX_PATH
colcon build --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
echo -e "${GREEN}[5/5] Setup complete!${NC}"
echo ""
echo "To use:"
echo "  source install/setup.bash"
echo "  ros2 launch rover_autonav autonav.launch.py"
echo ""
echo "Then connect Foxglove to: ws://<jetson-ip>:8765"

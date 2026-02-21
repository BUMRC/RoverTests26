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

# Install apt dependencies
echo -e "${GREEN}[1/5] Installing apt dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-foxglove-bridge \
    ros-humble-depthimage-to-laserscan \
    ros-humble-image-transport-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-compressed-image-transport \
    python3-colcon-common-extensions \
    python3-pip

# Install Python dependencies for motor driver
echo -e "${GREEN}[1.5/5] Installing Python dependencies...${NC}"
pip3 install adafruit-circuitpython-pca9685 adafruit-extended-bus

# Clone ZED ROS 2 wrapper
echo -e "${GREEN}[2/5] Setting up ZED ROS 2 wrapper...${NC}"
if [ ! -d "src/zed-ros2-wrapper" ]; then
    cd src
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    cd ..
else
    echo "zed-ros2-wrapper already exists, skipping clone"
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
echo -e "${GREEN}[5/5] Setup complete!${NC}"
echo ""
echo "To use:"
echo "  source install/setup.bash"
echo "  ros2 launch rover_autonav autonav.launch.py"
echo ""
echo "Then connect Foxglove to: ws://<jetson-ip>:8765"

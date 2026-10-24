#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# setup_ros2.sh — Install ROS 2 Humble + tools on Ubuntu 22.04 (x86_64 or
# Jetson aarch64). Safe to re-run; skips anything already installed.
#
# Usage:  ./scripts/setup_ros2.sh
# ---------------------------------------------------------------------------
set -euo pipefail

if [[ "$(lsb_release -rs 2>/dev/null || true)" != "22.04" ]]; then
    echo "ERROR: This script targets Ubuntu 22.04 (required for ROS 2 Humble)."
    echo "Your system: $(lsb_release -ds 2>/dev/null || 'unknown')"
    exit 1
fi

echo "==> Adding the ROS 2 apt repository"
sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -qq

if ! dpkg -s ros-humble-desktop >/dev/null 2>&1; then
    echo "==> Installing ROS 2 Humble desktop (large download, please wait)"
    sudo apt-get install -y ros-humble-desktop
else
    echo "==> ROS 2 Humble already installed"
fi

echo "==> Installing extra tools used by this project"
sudo apt-get install -y ros-dev-tools \
    ros-humble-teleop-twist-keyboard \
    python3-pip python3-dev build-essential

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "==> Adding ROS 2 sourcing to ~/.bashrc"
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo
echo "==> Done! Open a new terminal (or run: source ~/.bashrc) and verify with:"
echo "    ros2 topic list"

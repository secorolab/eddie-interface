#!/bin/bash

echo "POST INSTALL - Ensuring capabilities and ldconfig setup"

# Determine the directory where the script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_FILE="${SCRIPT_DIR}/eddie_ros_interface"
LD_CONF_FILE="/etc/ld.so.conf.d/ld_eddie_ros.conf"

# Prompt for sudo at the start
sudo -v || exit 1

# Set capabilities
if ! getcap "$TARGET_FILE" | grep -q "cap_sys_nice"; then
    echo "Setting capabilities on $TARGET_FILE"
    sudo setcap cap_sys_nice,cap_net_raw,cap_net_admin+ep "$TARGET_FILE"
else
    echo "Capabilities already set on $TARGET_FILE, skipping"
fi

# Update ld.so.conf.d
if [ ! -f "$LD_CONF_FILE" ]; then
    echo -e "/opt/ros/$ROS_DISTRO/lib\n/opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu" | sudo tee "$LD_CONF_FILE"
    sudo ldconfig
    echo "ldconfig updated"
else
    echo "$LD_CONF_FILE already exists, skipping"
fi

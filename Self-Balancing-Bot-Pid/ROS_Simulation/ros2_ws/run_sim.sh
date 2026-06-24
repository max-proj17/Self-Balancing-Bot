#!/bin/bash
source /opt/ros/humble/setup.bash
set -e
cd "$(dirname "$0")"

echo "Cleaning previous install..."
rm -rf build/balance_bot install/balance_bot

echo "Building package..."
colcon build --packages-select balance_bot

echo "Sourcing workspace..."
source install/setup.bash

echo "Launching simulation..."
ros2 launch balance_bot balance_bot_gazebo_.launch.py
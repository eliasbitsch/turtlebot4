#!/bin/bash
# SSH Teleop - Runs teleop_twist_keyboard on the robot via SSH
# This bypasses DDS network issues by running the teleop directly on the robot

ROBOT_IP="192.168.100.100"
ROBOT_USER="ubuntu"
ROBOT_PASS="turtlebot4"

echo "Starting teleop on TurtleBot4 via SSH..."
echo "Use WASD keys to control, Q/Z for speed, press Ctrl+C to exit"
echo ""

sshpass -p "$ROBOT_PASS" ssh -t -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" \
    "source /etc/turtlebot4/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

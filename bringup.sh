#!/bin/bash
# This script is used to bring up the ROS 2 environment for the robot.


# Set correct permissions for serial port and input device
bash /home/ubuntu/krytn_ws/open_ports.sh
# Switch to ubuntu user and run ROS 2 launch
su ubuntu -c "
cd /home/ubuntu/krytn_ws; \
source /home/ubuntu/krytn_ws/install/setup.bash; \
ros2 launch krytn bringup.launch.py
"

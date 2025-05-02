#!/bin/bash
# This script is used to bring up the ROS 2 environment for the robot.


# Set correct permissions for serial port and input device
chmod 666 /dev/ttyS0
for i in {0..5}; do
    chmod 666 /dev/input/event$i
done
echo "Permissions set for /dev/ttyS0 and /dev/input/event5"

# Switch to ubuntu user and run ROS 2 launch
su ubuntu -c "
cd /home/ubuntu/krytn_ws; \
source /home/ubuntu/krytn_ws/install/setup.bash; \
ros2 launch krytn bringup.launch.py
"

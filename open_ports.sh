#!/bin/bash

# Set correct permissions for serial port and input device
chmod 666 /dev/ttyS0
chmod 666 /dev/input/event*
chmod 666 /dev/video*

chmod 666 /dev/i2c-0
chmod 666 /dev/i2c-1

echo "Permissions set for /dev/ttyS0 and /dev/input/event5"

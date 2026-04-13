#!/bin/bash
/usr/bin/docker run --rm --network=host \
    -v /home/ubuntu/krytn_ws:/workspace \
    --group-add dialout \
    --group-add video \
    --group-add tty \
    --group-add messagebus \
    --group-add sgx \
    --device /dev/ttyS0:/dev/ttyS0 \
    --device /dev/ttyUSB0:/dev/ttyUSB0 \
    --device /dev/i2c-1:/dev/i2c-1 \
    --device /dev/i2c-0:/dev/i2c-0 \
    --device /dev/input/event0:/dev/input/event0 \
    --device /dev/input/event1:/dev/input/event1 \
    --device /dev/input/event2:/dev/input/event2 \
    --device /dev/input/event3:/dev/input/event3 \
    --device /dev/input/event4:/dev/input/event4 \
    --device /dev/input/event5:/dev/input/event5 \
    --device /dev/media0:/dev/media0 \
    --device /dev/media1:/dev/media1 \
    --device-cgroup-rule="c 4:* rwm" \
    --device-cgroup-rule="c 188:* rwm" \
    --device-cgroup-rule="c 89:* rwm" \
    --device-cgroup-rule="c 13:* rwm" \
    --device-cgroup-rule="c 81:* rwm" \
    --device-cgroup-rule="c 251:* rwm" \
    --privileged \
     ros2 /bin/bash  /bringup.sh

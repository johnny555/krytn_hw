docker run -it --rm --privileged --network=host -v /dev/:/dev/ --env UDEV=1 --device /dev:/dev -v /run/udev:/run/udev --entrypoint /bin/bash ros2

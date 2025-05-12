#!/bin/bash
bash /home/krytn/krytn_ws/open_ports.sh
#/usr/bin/docker run --rm -d --network=host --privileged --tmpfs /dev/shm:exec -v /run/udev:/run/udev:ro -e MTX_PATHS_CAM_SOURCE=rpiCamera bluenviron/mediamtx:latest-rpi
#sleep 10

/usr/bin/docker compose -f /home/krytn/krytn_ws/.devcontainer/docker-compose-bringup.yaml up

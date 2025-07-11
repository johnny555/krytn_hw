#!/bin/bash
sleep 5
bash /home/krytn/krytn_ws/open_ports.sh
su krytn

/usr/bin/docker compose -f /home/krytn/krytn_ws/.devcontainer/docker-compose-bringup.yaml up

#!/bin/bash
bash /home/krytn/krytn_ws/open_ports.sh

/usr/bin/docker compose -f /home/krytn/krytn_ws/.devcontainer/docker-compose-bringup.yaml up

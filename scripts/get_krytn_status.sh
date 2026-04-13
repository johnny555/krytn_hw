#!/bin/bash
SERVICE_NAME="krytn-bringup.service"
STATUS=$(systemctl is-active "$SERVICE_NAME")

echo -n "$STATUS"

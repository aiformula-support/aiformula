#!/bin/bash
set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)

# CAN
bash ${SCRIPT_DIR}/can_bringup.sh

# IMU
sudo chmod 777 /dev/ttyUSB0 

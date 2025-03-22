#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the mounted repository (docker-compose will do the volume mount)
cd /home/${USERNAME}/virtual_teach_vtr_wrapper || cd /home/root/virtual_teach_vtr_wrapper

# Upgrade pip and setuptools
python3 -m pip install --no-cache-dir --upgrade pip setuptools

# If you want to install Nerfstudio in editable mode:
python3 -m pip install --no-cache-dir -e ./src/nerfstudio

echo "ROS 2 container is ready."
exec "$@"


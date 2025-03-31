#!/bin/bash
set -e
xhost +local:root
cd "${VTRROOT}"

# --- Nerfstudio Initialization ---
export CONDA_ALWAYS_YES=true
NERFSTUDIO_DIR="${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio"

# Count the number of items in the directory
item_count1=$(find "${NERFSTUDIO_DIR}" -mindepth 1 | wc -l)
# Count README files (case-insensitive) in the directory
readme_count1=$(find "${NERFSTUDIO_DIR}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) | wc -l)

if [ "$item_count1" -eq 0 ] || [ "$item_count1" -eq "$readme_count1" ]; then
  echo "NERFSTUDIO_DIR is empty or only contains README(s). Removing README(s) and seeding..."
  # Remove README files (case-insensitive)
  find "${NERFSTUDIO_DIR}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) -delete
  
  cp -r /tmp/nerfstudio_default/* "${NERFSTUDIO_DIR}/"
  
  echo "Installing nerfstudio in editable mode..."
  source /opt/miniconda/etc/profile.d/conda.sh
  conda activate nerfstudio
  cd virtual_teach_vtr_wrapper/src/nerfstudio
  pip install -e "${NERFSTUDIO_DIR}"
  
  # Mark the installation as complete so it won't run again
  touch "${NERFSTUDIO_DIR}/.installed"
  
else
  echo "Nerfstudio repository already exists at ${NERFSTUDIO_DIR}."
fi
  
  conda deactivate
  echo "deactivated conda environment for nerfstudio"
  conda init
  
# --- Catkin Workspace Initialization ---
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

cd "${VTRROOT}"

TARGET="${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws"
DEFAULT="/catkin_ws_default"

# Count the number of items in the directory
item_count2=$(find "${TARGET}" -mindepth 1 | wc -l)
# Count README files (case-insensitive) in the directory
readme_count2=$(find "${TARGET}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) | wc -l)

if [ "$item_count2" -eq 0 ] || [ "$item_count2" -eq "$readme_count2" ]; then
  echo "TARGET is empty or only contains README(s). Removing README(s) and seeding..."
  # Remove README files (case-insensitive)
  find "${TARGET}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) -delete
  
  echo "Initializing persistent catkin directory..."
  cp -r "${DEFAULT}/"* "${TARGET}/"
  
  # Mark the workspace as initialized so these steps are skipped on subsequent startups.
  touch "${TARGET}/.initialized"
else
  echo "Catkin workspace already initialized at ${TARGET}."
fi

# Execute the command passed to the container (or default to an interactive bash shell)
exec "$@"

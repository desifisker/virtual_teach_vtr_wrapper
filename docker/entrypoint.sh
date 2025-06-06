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
  pip install --upgrade pip setuptools
  #  pip install -e "${NERFSTUDIO_DIR}/"
  pip install -e .
 #export PATH=$HOME/.local/bin:$PATH
  export PATH=$PATH:$HOME/.local/bin
  
  conda deactivate
  echo "deactivated conda environment for nerfstudio"
  conda init
  
else
  echo "Nerfstudio repository already exists at ${NERFSTUDIO_DIR}."
fi
  
# --- Catkin Workspace Initialization ---
source /opt/ros/noetic/setup.bash
#source /catkin_ws/devel/setup.bash

cd "${VTRROOT}"

TARGET="${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws"

W_TARGET="${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src"
W_DEFAULT="/catkin_ws_default/src"

# Check if a README file (or variant) exists
readme_count2=$(find "${TARGET}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) | wc -l)
# Check if the specific folder exists
if [ "$readme_count2" -gt 0 ] && [ -d "${TARGET}/src" ]; then
  echo "Found README and src folder in TARGET."
  echo "Removing README(s) and seeding persistent catkin directory (preserving warthog_gazebo_path_publisher in src)..."
  
  # Remove README files (case-insensitive)
  find "${TARGET}" -maxdepth 1 -type f \( -iname "readme" -o -iname "readme.*" \) -delete
  
  echo "Initializing persistent catkin directory..."
  cp -r "${W_DEFAULT}/"* "${W_TARGET}/"
   
  echo "Building catkin workspace..."
  cd "${TARGET}"
  source /opt/ros/noetic/setup.bash
  catkin_make
  catkin_make install -DCMAKE_INSTALL_PREFIX=/catkin_ws/install
  cd "${TARGET}/src/warthog_gazebo_path_publisher/scripts"
  chmod +x ${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_gazebo_path_publisher/scripts/save_path.py

else
  echo "Catkin workspace already initialized at ${TARGET}."
fi

# Execute the command passed to the container (or default to an interactive bash shell)
exec "$@"

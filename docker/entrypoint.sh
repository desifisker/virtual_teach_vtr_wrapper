#!/bin/bash
set -e
# --- Nerfstudio Initialization ---
# Define the target directory for nerfstudio within your project
NERFSTUDIO_DIR="${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio"
export CONDA_ALWAYS_YES=true

if [ ! -d "${NERFSTUDIO_DIR}" ]; then
  echo "Cloning nerfstudio repository into ${NERFSTUDIO_DIR}..."
  git clone https://github.com/nerfstudio-project/nerfstudio.git "${NERFSTUDIO_DIR}"
else
  echo "Nerfstudio repository already exists at ${NERFSTUDIO_DIR}."
fi

  # --- Initialize conda ---
  #if [ -f /opt/miniconda/etc/profile.d/conda.sh ]; then
  #  echo "Sourcing Miniconda initialization..."
  #  source /opt/miniconda/etc/profile.d/conda.sh
  #else
    #echo "Miniconda not found at /opt/miniconda. Exiting."
   # exit 1
  #fi
  
 
if [ ! -f "${NERFSTUDIO_DIR}/.installed" ]; then
  echo "Installing nerfstudio for the first time..."
  cd "${NERFSTUDIO_DIR}"  
  #TOOK OUT MAKING CONDA ENV - I DID GET IT WOKRING FULLY IN CONDA ENV BY REDOING PYTORCH AND TCNN STUFF 
  pip install --upgrade pip

  # Install torch and torchvision for CUDA 11.8
  pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
  
  # Install CUDA toolkit via conda (ensure conda is installed in the container)
  conda install -y -c "nvidia/label/cuda-11.8.0" cuda-toolkit
  
  # Install ninja and tiny-cuda-nn (for torch bindings)
  pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
  
  pip install --upgrade pip setuptools
  
  # Install nerfstudio (non-editable mode, as editable mode requires a setup.py)
  pip install -e .
  
  # Mark the installation as complete so it won't run again
  touch .installed
else
  echo "Nerfstudio is already installed at ${NERFSTUDIO_DIR}."
fi

  conda deactivate
  echo "deactivated conda environment for nerfstudio"
  conda init
  conda deactivate
  
# --- Catkin Workspace ---
# Source the ROS environment
#source /opt/ros/noetic/setup.bash

# Source the catkin workspace built in /catkin_ws
#source /catkin_ws/devel/setup.bash
source /catkin_ws/install/setup.bash
cd ${VTRROOT}

#if [ ! -f "${VTRROOT}/catkin_ws/.initialized" ]; then
  #echo "Initializing catkin workspace at ${VTRROOT}/catkin_ws..."

  # Create the workspace directory if it doesn't exist.
  #mkdir -p "${VTRROOT}/catkin_ws/src"
  
  # Change to the source directory and clone the Clearpath packages.
  #cd "${VTRROOT}/catkin_ws/src"
  
  #if [ ! -d "warthog_simulator" ]; then
    #echo "Cloning warthog_simulator repository..."
    #git clone https://github.com/warthog-cpr/warthog_simulator.git
  #else
    #echo "warthog_simulator already exists."
  #fi
  
  #if [ ! -d "warthog" ]; then
    #echo "Cloning warthog repository..."
    #git clone https://github.com/warthog-cpr/warthog.git
  #else
    #echo "warthog already exists."
  #fi

  # Ensure Empy is installed.
  #echo "Installing Empy in the conda environment..."
  #pip3 install empy
  #echo "Installing catkin_pkg..."
  #pip3 install catkin_pkg
  

  # Build the Catkin workspace.
  #cd "${VTRROOT}/catkin_ws"
  #echo "Building the workspace..."
  #/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make && source ${VTRROOT}/catkin_ws/devel/setup.bash"

  #cd "${VTRROOT}"
  # Overwrite configuration files.
  #echo "Copying custom configuration files..."
  #cp ${VTRROOT}/virtual_teach_vtr_wrapper/configs/warthog.urdf.xacro /opt/ros/noetic/share/warthog_description/urdf/warthog.urdf.xacro
  #cp ${VTRROOT}/virtual_teach_vtr_wrapper/configs/teleop_joy_config.yaml /opt/ros/noetic/share/teleop_twist_joy/config/teleop_joy_config.yaml
  #cp ${VTRROOT}/virtual_teach_vtr_wrapper/configs/material_0.png /opt/ros/noetic/share/warthog_description/meshes/
  #cp ${VTRROOT}/virtual_teach_vtr_wrapper/configs/RULER.dae /opt/ros/noetic/share/warthog_description/meshes/

  # Mark the workspace as initialized so these steps are skipped on subsequent startups.
  #touch "${VTRROOT}/catkin_ws/.initialized"
#else
  #echo "Catkin workspace already initialized at ${VTRROOT}/catkin_ws." 
#fi

export ROS_PACKAGE_PATH=${VTRROOT}/virtual_teach_vtr_wrapper/data:$ROS_PACKAGE_PATH

# Execute the command passed to the container (or default to an interactive bash shell)
exec "$@"


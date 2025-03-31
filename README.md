## Installation of VirT&R
VirT&R is a systematic pipeline to make virtual teach maps for VTR3 (https://github.com/utiasASRL/vtr3). Thus it makes use of several programs and requires VTR3 to use.

## Setup VTR3 Directories
Create the following directories in your local filesystem. Later they will be mapped to the docker container. (If you already have vtr3 installed and running on your machine skip this step.)

```Bash
export VTRROOT=~/ASRL  # (INTERNAL default) root directory of VTR3
# you can change the following directories to anywhere appropriate
export VTRSRC=${VTRROOT}/vtr3        # source code of VTR3
export VTRDEPS=${VTRROOT}/deps       # system dependencies of VTR3
export VTRTEMP=${VTRROOT}/temp       # temporary data directory for testing
mkdir -p ${VTRSRC} ${VTRTEMP} ${VTRDEPS}
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download VTR3 Source Code
Clone VTR3 to your local filesystem. (If you already have vtr3 installed and running on your machine skip this step.)

```Bash
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

## Download virtual_teach_vtr_wrapper
This package contains the vtr_virtual_teach C++ package for VTR3 and the custom scripts required to create virtual teach maps. Download it to your local filesystem in ${VTRROOT} as well.

```Bash
cd ${VTRROOT}
git clone git@github.com:desifisker/virtual_teach_vtr_wrapper.git
```

## Download vtr3_posegraph_tools
It is also recommened to download the following code as well (https://github.com/utiasASRL/vtr3_pose_graph), as this project contains useful scripts for ensuring the teach maps are made correctly. Scripts in this project are referenced in the Using VirT&R Documentation. Download it do your local filesystem in the same directory (${VTRROOT}) alongside vtr_virtual_teach_wrapper. 

```Bash
cd ${VTRROOT}
git clone git@github.com:utiasASRL/vtr3_pose_graph.git
```

## Build VTR3 Docker Image
This builds an image that has all dependencies installed for VTR3. (If you already have VTR3 installed and running on your machine skip this step.)

```Bash
xhost +local:root
cd ${VTRSRC}
docker build -t vtr3 \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Start the VTR3 Docker container
Install nvidia docker runtime first (https://nvidia.github.io/nvidia-container-runtime/), then run the docker container for VTR. (If you already have VTR3 installed and running on your machine skip this step - just open a new terminal in the container.)

```Bash
docker run -dit --rm --name vtr3 \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}:${HOME}:rw \
  -v ${HOME}/ASRL:${HOME}/ASRL:rw 
  vtr3
```

FYI: to start a new terminal with the existing container: 

```Bash
docker exec -it vtr3 bash
```

## Build and Install VT&R3
Start a new terminal and enter the container. (Again, if you have already built VTR3 and have it running, skip this step.)

```Bash
docker exec -it vtr3 bash

source /opt/ros/humble/setup.bash 
cd ${VTRSRC}/main
VTR_PIPELINE=LIDAR colcon build --symlink-install 

VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build
```

wait until it finishes.

## Build and Install virtual_teach_vtr_wrapper (this package)
Make sure to do this inside the vtr3 docker container (where you should currently still be working after having completed the above steps.) 

NOTE: IF YOU ALREADY HAVE VTR3 INSTALLED AND WORKING, THIS IS WHERE YOU NEED TO BEGIN FOLLOWING THESE INSTRUCTIONS.

```Bash

source /opt/ros/humble/setup.bash
echo $ROS_DISTRO                             
source ${VTRSRC}/main/install/setup.bash     
cd ~/ASRL/virtual_teach_vtr_wrapper         
colcon build --symlink-install
```

wait until it finishes.

Note that whenever you change any code in the VTR3 repo, you need to re-compile, do this by re-running the `colcon build ....` command for both VTR3 and then vtr_virtual_teach. Always wait until build process on VTR3 finishes before running the build command for vtr_virtual_testing.


## Create a python venv to install the posegraph python tools
Within the running container, create a virtual environment at `${VTRROOT}`. (If you already have these tools installed, skip this step.)

```Bash
cd ${VTRROOT}
virtualenv venv
source venv/bin/activate  
cd vtr3_posegraph_tools
pip3 install -e .
```

## Build and install the rest of the programs required
Now that VTR3 has been set up with the VirT&R extension package, the rest of the VirT&R pipeline must be installed. A separate Dockerfile for the other
programs was created to simplify the VirT&R pipeline set up for users with pre-exsiting VTR3 installations they may not want to edit, rebuild, or abandon. It has been set up to mount the same directories as VTR# for seamless use to create more of a solidified project and enable simpler file storage and transfer. Please build and run the Dockerfile.

```Bash
cd ${VTRROOT}/virtual_teach_vtr_wrapper/docker
docker build -t virtr \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} \
  --build-arg CUDA_ARCH="86" .
  
docker run -it --name virtr \
  --privileged \
  --network=host \
  --ipc=host \
  --gpus=all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${VTRROOT}:${VTRROOT}:rw \
  -v /dev:/dev \
  -v ${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws:${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws:rw \
  -v ${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio:${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio:rw \
  virtr
```

wait until it finishes.

Now you should be inside the virtr docker container where blender, gazebo, and nerfstudio are installed. All programs and dependencies should be built, installed, and ready for use. Please consult the Using VirT&R Documentation for detailed steps for how to proceed. 

## Example Dataset
An example dataset can be downloaded from this google drive link. It contains all elements created from this pipline (not just the input and output data) so you can follow along and ensure a good result from your installation of the pipeline.

DATASET DOWNLOAD:
https://drive.google.com/drive/folders/1TpRJCtvYFxTDJrL1TxS9-hO1WO6BE5z0?usp=sharing

## Helper scripts used to run the programs
Several helper scripts have been created to automate some tedious or menial aspects involved in creating a virtual teach map. They are referenced in the Using VirT&R Documentation and can be found below as a shortcut:

```Bash
./ImageExtractor.sh "/path/to/flight.csv" "/path/to/video1.mp4,/path/to/video2.mp4" "1742421135000,1742421136000" "/path/to/output" "/path/to/unified.txt" DJI

./ImageProcessor.sh "/path/to/input_images" "/path/to/output" "/path/to/scaling.txt" "/path/to/model_aligner.txt" "0.01" "/path/to/colmap/database.db" "/path/to/colmap/images" "/path/to/mapper/output" "/path/to/model_aligner/input" "/path/to/model_aligner/output" "0" "1.0"

./Blender.sh "/path/to/settings.json" "/path/to/pointcloud.ply" "1.5" "/path/to/mesh.obj"

```

## [License](./LICENSE)

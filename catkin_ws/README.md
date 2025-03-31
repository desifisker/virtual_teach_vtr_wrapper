## Installation of VirT&R
VirT&R is a systematic pipeline to make virtual teach maps for vtr3. Thus it makes use of several programs and requires vtr3 to use.

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

Also to your local filesystem, so that you don't have to access them from within the docker container. (If you already have vtr3 installed and running on your machine skip this step.)

```Bash
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download virtual_teach_vtr_wrapper

This package contains the vtr_virtual_teach c++ package for vtr3 and custom scripts required for data formatting and testing code to create virtual teach maps. Download it do your local filesystem.

```Bash
cd ${VTRROOT}
git clone git@github.com:desifisker/virtual_teach_vtr_wrapper.git
```

## Download vtr3_posegraph_tools

It is also recommened to download the following code as well, as this project contains useful scripts for ensuring the teach maps are made correctly. Scripts in this project are referenced in the readme and detailed instructions. Download it do your local filesystem in the same directory (${VTRROOT}) alongside vtr_virtual_teach_wrapper. 

```Bash
cd ${VTRROOT}
git clone git@github.com:utiasASRL/vtr3_pose_graph.git
```

## Build VTR3 Docker Image

This builds a image that has all dependencies installed. (If you already have vtr3 installed and running on your machine skip this step.)

```Bash
xhost +local:root
cd ${VTRSRC}
docker build -t vtr3_<your_name> \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Start the VTR3 Docker container

Install nvidia docker runtime first: https://nvidia.github.io/nvidia-container-runtime/ . (If you already have vtr3 installed and running on your machine skip this step - just open a new terminal in the container.)

```Bash
docker run -dit --rm --name vtr3 \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}:${HOME}:rw \
  -v ${HOME}/ASRL:${HOME}/ASRL:rw vtr3_<your_name>
```

FYI: to start a new terminal with the existing container: `docker exec -it vtr3 bash`

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Build and Install VT&R3

Start a new terminal and enter the container. (Again, if you have already built vtr3 and have it running, skip this step.)

```Bash
source /opt/ros/galactic/setup.bash 
cd ${VTRSRC}/main
VTR_PIPELINE=LIDAR colcon build --symlink-install 

VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build
```

wait until it finishes.

## Build and Install virtual_teach_vtr_wrapper (this package)

Make sure to do this inside the vtr3 docker container (where you should currently still be working after having completed the above steps.)

```Bash

source /opt/ros/galactic/setup.bash
echo $ROS_DISTRO                             # to ensure ros 2 is working
source ${VTRSRC}/main/install/setup.bash     # source the vtr3 environment
cd ~/ASRL/virtual_teach_vtr_wrapper          # go to where this repo is located
colcon build --symlink-install
```

wait until it finishes.

Note that whenever you change any code in the vtr3 repo, you need to re-compile and re-install, do this by re-running the `colcon build ....` command for both vtr3 and then vtr_virtual_teach. Always wait until build process on vtr3 finishes before running the build command for vtr_virtual_testing.


## Create a python venv to install the posegraph python tools

Within the running container, create a virtual environment at `${VTRROOT}`. 

```Bash
cd ${VTRROOT}
virtualenv venv
source venv/bin/activate  
cd vtr3_posegraph_tools
pip3 install -e .
```

## Run Docker Compose to set up the rest of the programs required for VirT&R, and mount a shared volume FOR NOW DOING AS TWO SEPARATE CONTAINERS!!!!!!!!!!

Now that vtr3 has been set up with the VirT&R extension package, the rest of the VirT&R pipeline must be installed. A separate docker container for the other
programs was created to simplify the VirT&R pipeline set up for users with pre-exsiting vtr3 installations they may not want to edit, rebuild, or abandon. Thus Docker Compose
will be used to mount the ${VTRROOT} directory as a shared directory to create more of a solidified project and enable simpler file storage and transfer.

```Bash
exit                           # exit the vtr3 docker container
cd ${VTRROOT}/virtual_teach_vtr_wrapper/docker
docker compose up --build -d 
docker start virtr_programs
ignore^^^^^^^^ for now^^^^^^^^66

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
  -v ${VTRROOT}/catkin_ws/src/warthog_simulator:/catkin_ws/src/warthog_simulator:rw \
  -v ${VTRROOT}/catkin_ws/src/warthog:/catkin_ws/src/warthog:rw \
  -v ${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio:/nerfstudio:rw \
  virtr
  
```

Now you should be inside the virtr_programs docker container where blender, gazebo, and nerfstudio are installed.

DATASET DOWNLOAD:
https://drive.google.com/drive/folders/1TpRJCtvYFxTDJrL1TxS9-hO1WO6BE5z0?usp=sharing

UPDATED USE COMMANDS:
```Bash
./ImageExtractor.sh "/path/to/flight.csv" "/path/to/video1.mp4,/path/to/video2.mp4" "1742421135000,1742421136000" "/path/to/output" "/path/to/unified.txt" DJI

./ImageProcessor.sh "/path/to/input_images" "/path/to/output" "/path/to/scaling.txt" "/path/to/model_aligner.txt" "0.01" "/path/to/colmap/database.db" "/path/to/colmap/images" "/path/to/mapper/output" "/path/to/model_aligner/input" "/path/to/model_aligner/output" "0" "1.0"

./Blender.sh "/path/to/settings.json" "/path/to/pointcloud.ply" "1.5" "/path/to/mesh.obj"


```


## [License](./LICENSE)





docker run -it --name virtr \
  --privileged \
  --network=host \
  --ipc=host \
  --gpus=all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${VTRROOT}:${VTRROOT}:rw \
  -v /dev:/dev \
  -v ${VTRROOT}/warthog_simulator:/catkin_ws/src/warthog_simulator:rw \
  -v ${VTRROOT}/warthog_control:/catkin_ws/src/warthog_control:rw
  -v /home/desiree/nerfstudio:/nerfstudio:rw \
  virtr
  
  

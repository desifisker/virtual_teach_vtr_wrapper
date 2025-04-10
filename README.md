## Installation of VirT&R
VirT&R is a systematic pipeline to make virtual teach maps for VTR3 (https://github.com/utiasASRL/vtr3). Thus, it makes use of several programs and requires VTR3 to use.

IF YOU ALREADY HAVE VTR3 SET UP AND RUNNING CLICK HERE TO GO TO THE NEXT PART AFTER MAKING SURE TO CLONE THIS REPO TO $VTRROOT: [Existing VTR3 Implementation Checkpoint](#virtr)


## Setup VTR3 Directories
Create the following directories in your local filesystem. Later, they will be mapped to the Docker container. (If you already have VTR3 installed and running on your machine, skip this step.)

```Bash
export VTRROOT=~/ASRL/vtr3         # (INTERNAL default) root directory
export VTRSRC=${VTRROOT}/src       # source code (this repo)
export VTRDATA=${VTRROOT}/data     # datasets
export VTRTEMP=${VTRROOT}/temp     # default output directory
export VTRMODELS=${VTRROOT}/models # .pt models for TorchScript
mkdir -p ${VTRSRC} ${VTRTEMP} ${VTRDEPS}
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download VTR3 Source Code
Clone VTR3 to your local filesystem. (If you already have VTR3 installed and running on your machine, skip this step.)

```Bash
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

## Download virtual_teach_vtr_wrapper
This package contains the vtr_virtualteach C++ package for VTR3 and the custom scripts required to create virtual teach maps. Download it to your local filesystem in ${VTRROOT}.

```Bash
cd ${VTRROOT}
git clone git@github.com:desifisker/virtual_teach_vtr_wrapper.git
```

## Download vtr3_posegraph_tools
It is also recommended to download the following project as well (https://github.com/utiasASRL/vtr3_pose_graph), as it contains useful scripts for ensuring the teach maps are made correctly. Scripts in this project are referenced in the 'Using VirT&R' documentation. Download it to your local filesystem in the same directory (${VTRROOT}) alongside vtr_virtual_teach_wrapper if you do not already have it. 

```Bash
cd ${VTRROOT}
git clone git@github.com:utiasASRL/vtr3_pose_graph.git
```

## Build VTR3 Docker Image
This builds an image that has all dependencies installed for VTR3. (If you already have VTR3 installed and running on your machine, skip this step.)

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
Install Nvidia Docker runtime first (https://nvidia.github.io/nvidia-container-runtime/), then run the Docker container for VTR3. (If you already have VTR3 installed and running on your machine, skip this step.)

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

## Create a python venv to install the posegraph python tools
Within the running VTR3 Docker container, create a virtual environment at `${VTRROOT}`. (If you already have these tools installed, skip this step.)

```Bash
cd ${VTRROOT}
virtualenv venv
source venv/bin/activate  
cd vtr3_posegraph_tools
pip3 install -e .
```
<a id="virtr"></a>
## Build and Install virtual_teach_vtr_wrapper (this package)
The vtr_virtualteach package interfaces directly with the VTR3 codebase and must be built used in the vtr3 Docker container, and built with the VTR3 project sourced. 

NOTE: IF YOU ALREADY HAVE VTR3 INSTALLED AND WORKING, THIS IS WHERE YOU NEED TO BEGIN FOLLOWING THESE INSTRUCTIONS.

```Bash
docker exec -it vtr3 bash

source ${VTRSRC}/main/install/setup.bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO

cd ${VTRROOT}/virtual_teach_vtr_wrapper/
colcon build --packages-select vtr_virtualteach

source ~/ASRL/vtr3/virtual_teach_vtr_wrapper/install/setup.bash

exit
```

Note that whenever you change any code in the VTR3 repo, you need to re-compile, do this by re-running the `colcon build ....` command for both VTR3 and then vtr_virtualteach (nd re-source both). Always wait until the build process on VTR3 finishes before running the build command for vtr_virtual_testing.


## Build and install the rest of the programs required
Now that VTR3 has been set up with the VirT&R extension package, the rest of the VirT&R pipeline must be installed. A separate Dockerfile for the other programs was created to simplify the VirT&R pipeline set-up for users with pre-existing VTR3 installations they may not want to edit, rebuild, or abandon. It has been set up to mount the same directories as VTR3 for seamless use to create more of a solidified project and enable more straightforward file storage and transfer. 

Please build and run the VirT&R Dockerfile (outside of the VTR3 Docker container in ${VTRROOT}).

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
  -v ${HOME}/.gazebo:${HOME}/.gazebo:rw \
  -v /dev:/dev \
  -v ${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws:${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws:rw \
  -v ${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio:${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio:rw \
  virtr
```

wait until it finishes (takes around 30 min to build and 20 min to finish the entrypoint.sh upon run time).

Now you should be inside the VirT&R Docker container where Blender, Gazebo, and Nerfstudio are installed. All programs and dependencies should be built, installed, and ready for use. 

## Please consult the 'Using VirT&R' documentation for detailed steps on how to proceed. 
https://docs.google.com/document/d/1vdrQSJKqGCwA7cPzZpHC-xvCbrQ00U4pSuBSXlRZtOs/edit?usp=sharing

## Example Dataset
An example dataset can be downloaded from this Google Drive link. It contains all elements created as a result of the steps in this pipeline (not just the input and output data), so you can follow along and ensure a good result from your installation of the pipeline.

The structure of the data in the example dataset folder 'test_press' should be replicated for all data used with this pipeline for the helper scripts to function correctly (although by running the first launch script, the correct structure is created for subsequent scripts and so on, so it should be handled for you, provided you initially set up the .csv and .mp4 correctly - file paths are left configurable so that you have more control over your file management.)

DATASET DOWNLOAD:
https://drive.google.com/drive/folders/1TpRJCtvYFxTDJrL1TxS9-hO1WO6BE5z0?usp=sharing

## Helper scripts used to run the programs
The following helper scripts have been created to automate some tedious or menial aspects involved in creating a virtual teach map. Below, they are specific to the test_press dataset and can be run sequentially according to the documentation. The helper scripts, and when to use them, are referenced in the 'Using VirT&R' documentation. To use the example dataset test_press, put the folder you downloaded from the link into the /data folder in the vtr_virtual_teach_wrapper. The video can't be compressed, so it's often downloaded separately. Just make sure it's called DJI_0001-001.MP4 as it is in the test_press folder too. These commands should be run outside the docker container.

```Bash
chmod +x ImageExtraction.sh ImageProcessor.sh Gazebo.sh TeachMap.sh        

./ImageExtraction.sh "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/DJIFlightRecord_2025-02-22_[11-59-34].csv" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/DJI_0001-001.MP4" "1740243662000" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/images" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/all_image_poses.txt" DJI

# makes: images folder and all_image_poses.txt file


./ImageProcessor.sh "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/images" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/colmap"  "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/all_image_poses.txt" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/colmap/filtered.txt" "0.01" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/colmap/database.db" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/colmap/Scaled" "press_test"

# makes: output folder of filtered images corresponding to filtered_and_scaled.txt (also created from this), database.db, the colmap folder, and the scaled project folder (which will hold the 3 generated .bin files produced)


./Gazebo.sh "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/meshes/mesh.dae" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/meshes/material_0.png" "test_press_world"

# makes: .world, .launch, .config, and .sdf files specific to the dataset


./TeachMap.sh "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/pointclouds/point_cloud.ply" "${VTRROOT}/virtual_teach_vtr_wrapper/data/test_press/paths/relative_transforms.csv" "test_press"

# makes: virtual teach map (graph folder used to run with Lidar Teach and Repeat)

```

## [License](./LICENSE)

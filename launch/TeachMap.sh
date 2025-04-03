#!/bin/bash
# TeachMap.sh - Setup and generate the teach map.
#
# Usage: ./TeachMap.sh <point_cloud_ply> <path_csv> <project_name>
# Example:
#   ./TeachMap.sh "/home/desiree/ASRL/vtr3/virtual_teach_vtr_wrapper/data/test_press/pointclouds/point_cloud.ply" "/home/desiree/ASRL/vtr3/virtual_teach_vtr_wrapper/data/test_press/paths/relative_transforms.csv" "test_press"

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <point_cloud_ply> <path_csv> <project_name>"
  exit 1
fi

PC_PATH="$1"
CSV_PATH="$2"
PROJECT_NAME="$3"

# Start the Docker container named vtr3.
docker start vtr3

# Build the multi-line command to be executed inside the container.
DOCKER_SCRIPT=$(cat <<'EOF'
source ${VTRSRC}/main/install/setup.bash; 
source /opt/ros/humble/setup.bash; 
echo "ROS_DISTRO: $ROS_DISTRO"; 
cd /home/desiree/ASRL/vtr3/src/main; 
mkdir -p "${VTRROOT}/data/<PROJECT_NAME>/"; 
cp "<PC_PATH>" "${VTRROOT}/data/<PROJECT_NAME>/point_cloud.ply"; 
cp "<CSV_PATH>" "${VTRROOT}/data/<PROJECT_NAME>/nerf_gazebo_relative_transforms.csv"; 
python3 - <<PYTHON_EOF
import open3d as o3d
import os
vtrroot = os.environ.get("VTRROOT")
ply_path = os.path.join(vtrroot, "data", "<PROJECT_NAME>", "point_cloud.ply")
pcd = o3d.io.read_point_cloud(ply_path)
pcd_path = os.path.join(vtrroot, "data", "<PROJECT_NAME>", "point_cloud.pcd")
o3d.io.write_point_cloud(pcd_path, pcd)
print("Converted point cloud from .ply to .pcd.")
PYTHON_EOF
sed -i 's|/home/desiree/ASRL/vtr3/data/test_press/point_cloud.pcd|/home/desiree/ASRL/vtr3/data/<PROJECT_NAME>/point_cloud.pcd|g' ${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtualteach/src/generate_global_map.cpp; 
sed -i 's|/home/desiree/ASRL/vtr3/data/test_press/nerf_gazebo_relative_transforms.csv|/home/desiree/ASRL/vtr3/data/<PROJECT_NAME>/nerf_gazebo_relative_transforms.csv|g' ${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtualteach/src/generate_global_map.cpp; 
sed -i 's|/home/desiree/ASRL/vtr3/data/test_press/graph|/home/desiree/ASRL/vtr3/data/<PROJECT_NAME>/graph|g' ${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtualteach/src/generate_global_map.cpp; 

source ~/ASRL/vtr3/virtual_teach_vtr_wrapper/install/setup.bash 
ros2 run vtr_virtualteach generate_global_map
EOF
)

# Replace placeholders with actual paths and project name.
DOCKER_SCRIPT="${DOCKER_SCRIPT//<PC_PATH>/$PC_PATH}"
DOCKER_SCRIPT="${DOCKER_SCRIPT//<CSV_PATH>/$CSV_PATH}"
DOCKER_SCRIPT="${DOCKER_SCRIPT//<PROJECT_NAME>/$PROJECT_NAME}"

# Execute the multi-line command inside the container.
docker exec -it vtr3 bash -c "$DOCKER_SCRIPT"
EXIT_CODE=$?
exit $EXIT_CODE

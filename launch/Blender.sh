#!/bin/bash
# launch3.sh - Launch file to run Blender processing inside the 'virtr_programs' Docker container

# Expected arguments:
#   1. JSON file path (contains a key "scale" with a precise floating point value)
#   2. Point cloud file path
#   3. Scale factor (additional multiplier for both objects)
#   4. Mesh file path

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <json_file> <point_cloud> <scale_factor> <mesh_file>"
    exit 1
fi

JSON_FILE="$1"
POINT_CLOUD="$2"
SCALE_FACTOR="$3"
MESH_FILE="$4"

# Start the Docker container named virtr_programs.
docker start virtr_programs

# Build the command string to be executed inside the container.
# Adjust the path to blender_process.py as needed.
DOCKER_CMD="
  source /opt/ros/noetic/setup.bash &&
  blender --background --python \${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtual_teach/scripts/blender_process.py -- \"$JSON_FILE\" \"$POINT_CLOUD\" \"$SCALE_FACTOR\" \"$MESH_FILE\"
"

# Run the command inside the 'virtr_programs' container.
docker exec -it virtr_programs bash -c "$DOCKER_CMD"
EXIT_CODE=$?
exit $EXIT_CODE


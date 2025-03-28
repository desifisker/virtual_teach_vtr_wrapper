#!/bin/bash
# launch.sh - Launch file to start Docker container and run VTR registration scripts

if [ "$#" -ne 6 ]; then
  echo "Usage: $0 <csv_input_file> <video_file_paths> <epoch_timestamps> <output_dir> <txt_file_path> <DJI|GOPRO>"
  exit 1
fi

CSV_INPUT="$1"
VIDEO_FILES="$2"
EPOCH_TS="$3"
OUTPUT_DIR="$4"
TXT_FILE="$5"
REG_TYPE="$6"

# Start the Docker container named vtr3
docker start vtr3

# Build the command string to be executed inside the container.
DOCKER_CMD="
  source \${VTRSRC}/main/install/setup.bash &&
  source /opt/ros/humble/setup.bash &&
  cd /home/desiree/ASRL/vtr3/src/main &&
  echo \$ROS_DISTRO &&
  if [ \"$REG_TYPE\" = \"DJI\" ]; then
      python3 \${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtual_teach/scripts/DJI_GPSRegistration4Colmap.py \"$CSV_INPUT\" \"$VIDEO_FILES\" \"$EPOCH_TS\" \"$OUTPUT_DIR\" \"$TXT_FILE\";
  elif [ \"$REG_TYPE\" = \"GOPRO\" ]; then
      python3 \${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtual_teach/scripts/GOPRO_GPSRegistration4Colmap.py \"$CSV_INPUT\" \"$VIDEO_FILES\" \"$EPOCH_TS\" \"$OUTPUT_DIR\" \"$TXT_FILE\";
  else
      echo \"Unknown registration type. Please specify DJI or GOPRO.\"; 
      exit 1;
  fi
"

# Run the command inside the container
docker exec -it vtr3 bash -c "$DOCKER_CMD"
EXIT_CODE=$?
exit $EXIT_CODE


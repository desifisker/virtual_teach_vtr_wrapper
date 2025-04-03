#!/bin/bash
# launch2.sh - Launch file for image scaling/filtering and COLMAP processing

if [ "$#" -ne 8 ]; then
  echo "Usage: $0 <image_input_folder> <output_folder> <txt_file_prescaling> <txt_file_postscaling> <scale> <colmap_db_path> <model_aligner_output> <destination_subfolder>"
  exit 1
fi

IMGS="$1"
OUTPUT="$2"
ALL_IMG_POSES="$3"
FILTERED="$4"
SCALE="${5}"
COLMAP_DB="$6"
MODEL_ALIGNER_OUTPUT="$7"
DEST_SUBFOLDER="${8}"

# Start the Docker container named virtr.
docker start virtr

# Build the command string to be executed inside the container.
DOCKER_CMD="
  mkdir -p \"$OUTPUT\" &&
  mkdir -p \"$(dirname "$COLMAP_DB")\" &&
  mkdir -p \"$MODEL_ALIGNER_OUTPUT\" &&
  # Run the image filtering and coordinate scaling script.
  python3 \${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtualteach/scripts/imageFilterandCoordScaler.py \"$IMGS\" \"$OUTPUT\" \"$ALL_IMG_POSES\" \"$SCALE\" &&
  # COLMAP processing commands:
  colmap database_creator --database_path \"$COLMAP_DB\" &&
  colmap feature_extractor --image_path \"$IMGS\" --database_path \"$COLMAP_DB\" --ImageReader.camera_model SIMPLE_RADIAL --SiftExtraction.use_gpu=1 --SiftExtraction.num_threads=4 --SiftExtraction.max_num_features=8192 --log_level 2 &&
  colmap exhaustive_matcher --database_path \"$COLMAP_DB\" &&
  colmap mapper --database_path \"$COLMAP_DB\" --image_path \"$IMGS\" --output_path \"$OUTPUT\" &&
  colmap model_aligner --input_path "$OUTPUT/0" --output_path "$MODEL_ALIGNER_OUTPUT" --ref_images_path "$FILTERED" --ref_is_gps 0 --alignment_max_error 1.0 &&
  # Create the destination folder within nerfstudio using the provided subfolder name.
  mkdir -p \"/home/desiree/ASRL/vtr3/virtual_teach_vtr_wrapper/src/nerfstudio/nerfstudio/data/nerfstudio/${DEST_SUBFOLDER}\" &&
  # Move the images folder and the scaled folder into the destination subfolder.
  cp -r \"$IMGS\" \"/home/desiree/ASRL/vtr3/virtual_teach_vtr_wrapper/src/nerfstudio/nerfstudio/data/nerfstudio/${DEST_SUBFOLDER}/images\" &&
  cp -r \"$MODEL_ALIGNER_OUTPUT\" \"/home/desiree/ASRL/vtr3/virtual_teach_vtr_wrapper/src/nerfstudio/nerfstudio/data/nerfstudio/${DEST_SUBFOLDER}/Scaled_100\" &&
  cd "${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio" && 
  source /opt/miniconda/etc/profile.d/conda.sh &&
  conda activate nerfstudio && 
  pip install nerfstudio -e . && 
  export PATH=$PATH:$HOME/.local/bin &&
  exec bash
"

# Run the command string inside the container.
docker exec -it virtr bash -c "$DOCKER_CMD"
EXIT_CODE=$?
exit $EXIT_CODE

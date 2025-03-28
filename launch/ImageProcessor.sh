#!/bin/bash
# launch2.sh - Launch file for image scaling/filtering and COLMAP processing

# Expected arguments:
#  1. IMAGE_INPUT_FOLDER      : Folder containing input images.
#  2. OUTPUT_FOLDER           : Folder where processed images (and other outputs) will be saved.
#  3. TXT_FILE_SCALING        : Path to the txt file for image scaling.
#  4. TXT_FILE_MODEL_ALIGNER  : Path to the txt file for COLMAP model_aligner.
#  5. SCALE                   : Scaling factor (e.g., 0.01 or 0.166).
#  6. COLMAP_DB_PATH          : Database path for COLMAP (used for database_creator, feature_extractor, exhaustive_matcher, and mapper).
#  7. FEATURE_IMG_PATH        : Image path for COLMAP feature_extractor (and also used as mapper image path).
#  8. MAPPER_OUT_PATH         : Output path for COLMAP mapper.
#  9. MODEL_ALIGNER_INPUT     : Input path for COLMAP model_aligner.
# 10. MODEL_ALIGNER_OUTPUT    : Output path for COLMAP model_aligner.
# 11. REF_IS_GPS              : Reference flag for COLMAP model_aligner (e.g., 0).
# 12. ALIGNMENT_MAX_ERROR     : Alignment maximum error for COLMAP model_aligner (e.g., 1.0).

if [ "$#" -ne 12 ]; then
  echo "Usage: $0 <image_input_folder> <output_folder> <txt_file_scaling> <txt_file_model_aligner> <scale> <colmap_db_path> <feature_img_path> <mapper_out_path> <model_aligner_input> <model_aligner_output> <ref_is_gps> <alignment_max_error>"
  exit 1
fi

IMG_INPUT="$1"
OUT_FOLDER="$2"
TXT_SCALING="$3"
TXT_MODEL_ALIGNER="$4"
SCALE="$5"
COLMAP_DB_PATH="$6"
FEATURE_IMG_PATH="$7"
MAPPER_OUT_PATH="$8"
MODEL_ALIGNER_INPUT="$9"
MODEL_ALIGNER_OUTPUT="${10}"
REF_IS_GPS="${11}"
ALIGNMENT_MAX_ERROR="${12}"

# Start the Docker container named vtr3.
docker start vtr3

# Build the command string to be executed inside the container.
DOCKER_CMD="
  source \${VTRSRC}/main/install/setup.bash &&
  source /opt/ros/humble/setup.bash &&
  cd /home/desiree/ASRL/vtr3/src/main &&
  echo \$ROS_DISTRO &&
  # Run the image scaling/filtering script with provided arguments.
  python3 \${VTRROOT}/virtual_teach_vtr_wrapper/src/vtr_virtual_teach/scripts/imageFilterandCoordScaler.py \"$IMG_INPUT\" \"$OUT_FOLDER\" \"$TXT_SCALING\" \"$SCALE\" &&
  # COLMAP processing commands:
  colmap database_creator --database_path \"$COLMAP_DB_PATH\" &&
  colmap feature_extractor --image_path \"$FEATURE_IMG_PATH\" --database_path \"$COLMAP_DB_PATH\" --ImageReader.camera_model SIMPLE_RADIAL --SiftExtraction.use_gpu=1 --SiftExtraction.num_threads=4 --SiftExtraction.max_num_features=8192 --log_level 2 &&
  colmap exhaustive_matcher --database_path \"$COLMAP_DB_PATH\" &&
  colmap mapper --database_path \"$COLMAP_DB_PATH\" --image_path \"$FEATURE_IMG_PATH\" --output_path \"$MAPPER_OUT_PATH\" &&
  colmap model_aligner --input_path \"$MODEL_ALIGNER_INPUT\" --output_path \"$MODEL_ALIGNER_OUTPUT\" --ref_images_path \"$TXT_MODEL_ALIGNER\" --ref_is_gps \"$REF_IS_GPS\" --alignment_max_error \"$ALIGNMENT_MAX_ERROR\"
"

# Run the command string inside the container.
docker exec -it vtr3 bash -c "$DOCKER_CMD"
EXIT_CODE=$?
exit $EXIT_CODE


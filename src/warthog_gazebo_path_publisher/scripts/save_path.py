#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import os
import time
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

def pose_to_matrix(position, orientation):
    """Convert position and orientation (quaternion) to a 4x4 transformation matrix."""
    rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [position.x, position.y, position.z]
    return transform

def compute_relative_transform(current_matrix, previous_matrix):
    """Compute the relative transformation matrix."""
    return np.linalg.inv(current_matrix) @ previous_matrix

# Parse the world name so we know where to save the CSV file.
parser = argparse.ArgumentParser(
    description="Save relative transform data for a Warthog simulation."
)
parser.add_argument("--world_name", type=str, required=True,
                    help="The world name; used to determine the output directory.")
args, unknown = parser.parse_known_args()

# Determine output directory based on VTRROOT and the given world name.
vtrroot = os.environ.get("VTRROOT", "/home/desiree/ASRL/Thesis/BuddySystemDatasets")
output_dir = os.path.join(vtrroot, "virtual_teach_vtr_wrapper", "data", "warthog_gazebo", "models", args.world_name)
output_file = os.path.join(output_dir, "relative_transforms.csv")

# Ensure the directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Global variables to store the initial and previous pose matrices and timestamp
initial_pose = None
previous_pose = None
last_timestamp = None

# Desired time interval between messages (in seconds)
sampling_interval = 0.2  # Adjust as needed

def callback(msg):
    global initial_pose, previous_pose, last_timestamp

    try:
        # Get the current time
        timestamp = time.time()

        # Check if enough time has passed since the last message
        if last_timestamp is not None and (timestamp - last_timestamp) < sampling_interval:
            return

        # Update the last timestamp
        last_timestamp = timestamp

        # Find the index of the Warthog
        warthog_index = msg.name.index("warthog")

        # Extract the Warthog's pose
        position = msg.pose[warthog_index].position
        orientation = msg.pose[warthog_index].orientation

        # Convert the pose to a 4x4 transformation matrix
        current_matrix = pose_to_matrix(position, orientation)

        # If this is the first message, record the initial pose and write header
        if initial_pose is None:
            initial_pose = current_matrix
            with open(output_file, "w", newline="") as csvfile:
                csvwriter = csv.writer(csvfile)
                header = ["timestamp"] + [f"m{i}{j}" for i in range(4) for j in range(4)]
                csvwriter.writerow(header)
                csvwriter.writerow([timestamp] + initial_pose.flatten().tolist())

        # If we have a previous pose, compute and log the relative transform
        if previous_pose is not None:
            relative_matrix = compute_relative_transform(current_matrix, previous_pose)
            relative_flat = relative_matrix.flatten()
            with open(output_file, "a", newline="") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow([timestamp] + relative_flat.tolist())

        # Update the previous pose
        previous_pose = current_matrix

    except ValueError:
        rospy.logwarn("Warthog not found in ModelStates message.")

# Initialize the ROS node and subscribe to /gazebo/model_states
rospy.init_node("warthog_relative_transform_recorder")
rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

rospy.loginfo(f"Logging Warthog relative transforms to {output_file}")
rospy.spin()

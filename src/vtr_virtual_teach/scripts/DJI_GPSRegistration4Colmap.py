#!/usr/bin/env python3
import pandas as pd
from datetime import datetime
import numpy as np
import open3d as o3d
import math
import cv2
import os
import csv
import sys

def extract_dji_columns(input_csv):
    """
    Extracts relevant columns from the DJI CSV file.
    """
    columns_to_extract = [
        "CUSTOM.date [local]",
        "CUSTOM.updateTime [local]",
        "OSD.latitude",
        "OSD.longitude",
        "OSD.altitude [ft]",
        "OSD.height [ft]",
        "OSD.roll",
        "OSD.pitch",
        "OSD.yaw",
        "OSD.directionOfTravel",
        "GIMBAL.mode",
        "GIMBAL.pitch",
        "GIMBAL.roll",
        "GIMBAL.yaw",
        "CAMERA.isVideo"
    ]

    df = pd.read_csv(input_csv, skiprows=1, delimiter=',', header=0, dtype=str, low_memory=False)

    missing_columns = [col for col in columns_to_extract if col not in df.columns]
    if missing_columns:
        print(f"Warning: The following columns are missing in the CSV: {missing_columns}")

    data = {col: df[col].tolist() for col in columns_to_extract if col in df.columns}
    return data

def convert_update_time_to_epoch(data, output_dir):
    """
    Converts CUSTOM.updateTime [local] to epoch time in milliseconds.
    """
    if "CUSTOM.updateTime [local]" not in data or "CUSTOM.date [local]" not in data:
        print("Error: Required columns 'CUSTOM.updateTime [local]' or 'CUSTOM.date [local]' are missing.")
        return data

    update_time_local = data["CUSTOM.updateTime [local]"]
    date_local = data["CUSTOM.date [local]"]

    epoch_times_ms = []
    for time_str, date_str in zip(update_time_local, date_local):
        try:
            dt = datetime.strptime(f"{date_str} {time_str}", '%m/%d/%Y %I:%M:%S.%f %p')
            epoch_ms = int(dt.timestamp() * 1e3)  # Convert to milliseconds
            epoch_times_ms.append(epoch_ms)
        except ValueError as e:
            print(f"Error converting time for {date_str} {time_str}: {e}")
            epoch_times_ms.append(None)

    data["CUSTOM.updateTime [epoch_ms]"] = epoch_times_ms
    return data

def compute_drone_path(data, output_dir):
    """
    Computes Cartesian x, y, z coordinates from drone data and saves them as a CSV.
    """
    EARTH_RADIUS = 6378137.0  # in meters
    ECCENTRICITY_SQUARED = 0.00669437999014

    latitudes = list(map(float, data['OSD.latitude']))
    longitudes = list(map(float, data['OSD.longitude']))
    altitudes = list(map(float, data['OSD.altitude [ft]']))  # in feet
    epoch_times = list(map(int, data['CUSTOM.updateTime [epoch_ms]']))
    is_video = data['CAMERA.isVideo']

    # Convert altitudes from feet to meters
    altitudes_m = [alt * 0.3048 for alt in altitudes]

    # Use the first point as the reference for Cartesian conversion
    lat_ref = math.radians(latitudes[0])
    lon_ref = math.radians(longitudes[0])
    alt_ref = altitudes_m[0]

    N_ref = EARTH_RADIUS / math.sqrt(1 - ECCENTRICITY_SQUARED * math.sin(lat_ref)**2)
    x_ref = (N_ref + alt_ref) * math.cos(lat_ref) * math.cos(lon_ref)
    y_ref = (N_ref + alt_ref) * math.cos(lat_ref) * math.sin(lon_ref)
    z_ref = (N_ref * (1 - ECCENTRICITY_SQUARED) + alt_ref) * math.sin(lat_ref)

    results = {}
    for i, (lat, lon, alt) in enumerate(zip(latitudes, longitudes, altitudes_m)):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = EARTH_RADIUS / math.sqrt(1 - ECCENTRICITY_SQUARED * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad) - x_ref
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad) - y_ref
        z = (N * (1 - ECCENTRICITY_SQUARED) + alt) * math.sin(lat_rad) - z_ref

        results[epoch_times[i]] = (x, y, z, is_video[i])

    first_epoch = epoch_times[0] if epoch_times else "unknown"
    output_file = os.path.join(output_dir, f"drone_path_coordinates_{first_epoch}.csv")
    with open(output_file, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["epoch_ms", "x_meters", "y_meters", "z_meters", "is_video"])
        for epoch, (x, y, z, video) in results.items():
            csv_writer.writerow([epoch, x, y, z, video])

    return results

def split_video_sessions(xyz_coords_with_timestamps, output_dir):
    """
    Separates video sessions based on the CAMERA.isVideo flag and saves them as CSV files.
    """
    video_sessions = []
    current_session = []
    is_recording = False

    for epoch, (x, y, z, video) in xyz_coords_with_timestamps.items():
        if video == 'True':
            if not is_recording:
                if current_session:
                    video_sessions.append(current_session)
                current_session = []
                is_recording = True
            current_session.append((epoch, x, y, z))
        else:
            if is_recording:
                is_recording = False

    if current_session:
        video_sessions.append(current_session)

    for i, session in enumerate(video_sessions):
        session_file = os.path.join(output_dir, f"video_session_{session[0][0]}.csv")
        with open(session_file, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["epoch_ms", "x_meters", "y_meters", "z_meters"])
            for row in session:
                csv_writer.writerow(row)

    return tuple(video_sessions)

def process_video_sessions(video_sessions, start_epoch_times, video_paths, output_dir, unified_txt_file=None):
    """
    Processes video sessions by adjusting epoch times, extracting frames, and writing a unified txt file.
    """
    if unified_txt_file is None:
        unified_txt_file = os.path.join(output_dir, f"all_video_sessions_{start_epoch_times[0]}.txt")
    with open(unified_txt_file, mode='w') as unified_file:
        for i, session in enumerate(video_sessions):
            start_epoch = start_epoch_times[i]
            video_path = video_paths[i]
            print(f"Processing video {video_path} with start epoch time {start_epoch}")
            cap = cv2.VideoCapture(video_path)

            if not cap.isOpened():
                print(f"Error: Could not open video {video_path}")
                continue

            # Adjust session timestamps with the offset between CSV and known start epoch
            timestamp_offset = start_epoch - session[0][0]
            session = [(epoch + timestamp_offset, x, y, z) for epoch, x, y, z in session]

            for j, (adjusted_epoch, x, y, z) in enumerate(session):
                if j % 10 == 0:
                    video_time_ms = adjusted_epoch - start_epoch
                    cap.set(cv2.CAP_PROP_POS_MSEC, video_time_ms)
                    ret, frame = cap.read()
                    if ret:
                        frame_resized = cv2.resize(frame, (960, 540))
                        image_name = f"{adjusted_epoch}.jpg"
                        image_path = os.path.join(output_dir, image_name)
                        cv2.imwrite(image_path, frame_resized)
                        unified_file.write(f"{image_name} {x} {y} {z}\n")
                        print(f"Frame saved: {image_path}")
                    else:
                        print(f"Warning: Could not read frame at {adjusted_epoch} ms in video {video_path}")
            cap.release()
    print(f"All sessions processed and saved in {unified_txt_file}.")

def main():
    if len(sys.argv) != 6:
        print("Usage: python3 DJI_GPSRegistration4Colmap.py <csv_input_file> <video_file_paths> <epoch_timestamps> <output_dir> <txt_file_path>")
        sys.exit(1)
    csv_input_file = sys.argv[1]
    video_file_paths_str = sys.argv[2]
    epoch_timestamps_str = sys.argv[3]
    output_dir = sys.argv[4]
    txt_file_path = sys.argv[5]

    # Expect comma-separated lists for video file paths and epoch timestamps.
    video_paths = [v.strip() for v in video_file_paths_str.split(',')]
    start_epoch_times = [int(t.strip()) for t in epoch_timestamps_str.split(',')]

    data = extract_dji_columns(csv_input_file)
    data_epoch = convert_update_time_to_epoch(data, output_dir)

    print("Measurements (first and last values):")
    for key, values in data.items():
        print(f"  {key}: {values[:1]} ... {values[-1:]}")
    
    print("Measurements after epoch conversion:")
    for key, values in data_epoch.items():
        print(f"  {key}: {values[:1]} ... {values[-1:]}")

    xyz_coords_with_timestamps = compute_drone_path(data_epoch, output_dir)

    # Visualize the drone path using Open3D
    points = []
    for _, (x, y, z, _) in xyz_coords_with_timestamps.items():
        points.append([x, y, z])

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    drone_path_pcd = o3d.geometry.PointCloud()
    drone_path_pcd.points = o3d.utility.Vector3dVector(np.array(points))
    drone_path_pcd.paint_uniform_color([1, 0, 0])
    vis.add_geometry(drone_path_pcd)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(axis)

    video_sessions = split_video_sessions(xyz_coords_with_timestamps, output_dir)
    print(f"Processed {len(video_sessions)} video sessions. Session data saved to {output_dir}.")

    # For visualization, also add each video session as its own point cloud.
    for i, session in enumerate(video_sessions):
        session_points = []
        for epoch, x, y, z in session:
            session_points.append([x, y, z])
        session_pcd = o3d.geometry.PointCloud()
        session_pcd.points = o3d.utility.Vector3dVector(np.array(session_points))
        color = [0, 0, 1] if i == 0 else [0, 1, 0] if i == 1 else [1, 0, 1]
        session_pcd.paint_uniform_color(color)
        vis.add_geometry(session_pcd)

    vis.run()
    vis.destroy_window()

    process_video_sessions(video_sessions, start_epoch_times, video_paths, output_dir, unified_txt_file=txt_file_path)

if __name__ == '__main__':
    main()


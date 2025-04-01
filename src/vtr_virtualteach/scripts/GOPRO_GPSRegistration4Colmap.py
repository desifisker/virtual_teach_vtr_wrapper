#!/usr/bin/env python3
import pandas as pd
from datetime import datetime, timezone
import numpy as np
import open3d as o3d
import math
import cv2
import os
import csv
import sys

def extract_gopro_columns(input_csv):
    """
    Extracts relevant columns from the GoPro CSV file.
    Expected columns:
        - date
        - GPS (Lat.) [deg]
        - GPS (Long.) [deg]
        - GPS (Alt.) [m]
    """
    df = pd.read_csv(input_csv, delimiter=',', header=0, dtype=str, low_memory=False)
    df.columns = df.columns.str.strip()

    columns_to_extract = [
        "date",
        "GPS (Lat.) [deg]",
        "GPS (Long.) [deg]",
        "GPS (Alt.) [m]"
    ]
    
    missing_columns = [col for col in columns_to_extract if col not in df.columns]
    if missing_columns:
        print(f"Warning: The following columns are missing in the CSV: {missing_columns}")

    data = {col: df[col].tolist() for col in columns_to_extract if col in df.columns}
    return data

def convert_date_to_epoch(data):
    """
    Converts the 'date' column from ISO 8601 format to epoch time in milliseconds.
    """
    if "date" not in data:
        print("Error: Required column 'date' is missing.")
        return data
    
    date_strs = data["date"]
    epoch_times_ms = []
    for date_str in date_strs:
        try:
            dt = datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S.%fZ")
            dt = dt.replace(tzinfo=timezone.utc)
            epoch_ms = int(dt.timestamp() * 1e3)
            epoch_times_ms.append(epoch_ms)
        except ValueError as e:
            print(f"Error converting date for {date_str}: {e}")
            epoch_times_ms.append(None)
    
    data["date_epoch_ms"] = epoch_times_ms
    return data

def compute_gopro_path(data, output_dir):
    """
    Computes Cartesian x, y, z coordinates from GoPro data and saves them as a CSV.
    """
    EARTH_RADIUS = 6378137.0
    ECCENTRICITY_SQUARED = 0.00669437999014

    latitudes = list(map(float, data['GPS (Lat.) [deg]']))
    longitudes = list(map(float, data['GPS (Long.) [deg]']))
    altitudes = list(map(float, data['GPS (Alt.) [m]']))
    epoch_times = list(map(int, data['date_epoch_ms']))
    
    lat_ref = math.radians(latitudes[0])
    lon_ref = math.radians(longitudes[0])
    alt_ref = altitudes[0]

    N_ref = EARTH_RADIUS / math.sqrt(1 - ECCENTRICITY_SQUARED * math.sin(lat_ref)**2)
    x_ref = (N_ref + alt_ref) * math.cos(lat_ref) * math.cos(lon_ref)
    y_ref = (N_ref + alt_ref) * math.cos(lat_ref) * math.sin(lon_ref)
    z_ref = (N_ref * (1 - ECCENTRICITY_SQUARED) + alt_ref) * math.sin(lat_ref)

    results = {}
    for i, (lat, lon, alt) in enumerate(zip(latitudes, longitudes, altitudes)):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = EARTH_RADIUS / math.sqrt(1 - ECCENTRICITY_SQUARED * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad) - x_ref
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad) - y_ref
        z = (N * (1 - ECCENTRICITY_SQUARED) + alt) * math.sin(lat_rad) - z_ref
        results[epoch_times[i]] = (x, y, z)
    
    first_epoch = epoch_times[0] if epoch_times else "unknown"
    output_file = os.path.join(output_dir, f"gopro_path_coordinates_{first_epoch}.csv")
    with open(output_file, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["epoch_ms", "x_meters", "y_meters", "z_meters"])
        for epoch, (x, y, z) in results.items():
            csv_writer.writerow([epoch, x, y, z])
    
    return results

def process_gopro_video(session_data, video_path, output_dir, unified_txt_file=None):
    """
    Processes the GoPro video session by extracting frames and writing coordinates to a txt file.
    """
    start_epoch = session_data[0][0]
    if unified_txt_file is None:
        unified_txt_file = os.path.join(output_dir, f"gopro_video_session_{start_epoch}.txt")
    
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video {video_path}")
        return
    
    with open(unified_txt_file, mode='w') as unified_file:
        for j, (adjusted_epoch, x, y, z) in enumerate(session_data):
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
    print(f"Video session processed. Data saved in {unified_txt_file}.")

def main():
    if len(sys.argv) != 6:
        print("Usage: python3 GOPRO_GPSRegistration4Colmap.py <csv_input_file> <video_file_paths> <epoch_timestamps> <output_dir> <txt_file_path>")
        sys.exit(1)
    csv_input_file = sys.argv[1]
    video_file_paths_str = sys.argv[2]
    epoch_timestamps_str = sys.argv[3]
    output_dir = sys.argv[4]
    txt_file_path = sys.argv[5]

    video_paths = [v.strip() for v in video_file_paths_str.split(',')]
    start_epoch_times = [int(t.strip()) for t in epoch_timestamps_str.split(',')]

    data = extract_gopro_columns(csv_input_file)
    print("Measurements from CSV (raw columns):")
    for key, values in data.items():
        print(f"  {key}: {values[:1]} ... {values[-1:]}")
    
    data = convert_date_to_epoch(data)
    print("\nMeasurements after date to epoch conversion:")
    for key, values in data.items():
        print(f"  {key}: {values[:1]} ... {values[-1:]}")

    gopro_path = compute_gopro_path(data, output_dir)
    
    session_data = []
    for epoch, (x, y, z) in sorted(gopro_path.items()):
        session_data.append((epoch, x, y, z))
    
    # Visualize the GPS path using Open3D
    points = [[x, y, z] for _, x, y, z in session_data]
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    path_pcd = o3d.geometry.PointCloud()
    path_pcd.points = o3d.utility.Vector3dVector(np.array(points))
    path_pcd.paint_uniform_color([1, 0, 0])
    vis.add_geometry(path_pcd)
    
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(axis)
    
    vis.run()
    vis.destroy_window()
    
    # For GoPro, we assume a single video session
    process_gopro_video(session_data, video_paths[0], output_dir, unified_txt_file=txt_file_path)

if __name__ == '__main__':
    main()


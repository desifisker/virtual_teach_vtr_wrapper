import open3d as o3d
import numpy as np
import pandas as pd
import collada
import os
import xml.etree.ElementTree as ET


def transform_point_cloud(pcd, scale_factor=1.0, rotation_angle_deg=0, axis='z'):
    pcd.scale(scale_factor, center=pcd.get_center())
    rotation_angle_rad = np.radians(rotation_angle_deg)

    if axis == 'x':
        R = pcd.get_rotation_matrix_from_xyz((rotation_angle_rad, 0, 0))
    elif axis == 'y':
        R = pcd.get_rotation_matrix_from_xyz((0, rotation_angle_rad, 0))
    elif axis == 'z':
        R = pcd.get_rotation_matrix_from_xyz((0, 0, rotation_angle_rad))
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")

    pcd.rotate(R,  center=(0, 0, 0))
    return pcd

def translate_point_cloud(pcd, translation=(0, 0, 0)):
    pcd.translate(translation)
    return pcd

def load_pcd(pcd_file):
    """Loads a .pcd file as an Open3D point cloud."""
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd.paint_uniform_color([1, 0, 0])  # Color the point cloud red
    return pcd

def load_or_convert_point_cloud(file_path, number_of_points=None):
    """
    Loads a point cloud from a file. If the file is a PCD, it is loaded directly.
    If the file is an OBJ, it attempts to load it as a mesh first. If the OBJ
    contains only vertices, it loads them directly. The resulting point cloud is then
    colored green and normals are estimated.
    
    Parameters:
        file_path (str): Path to the input file (.pcd or .obj).
        number_of_points (int or None): For mesh OBJ files with faces, the target number
            of points to sample. If None, a default value of 10,000 is used.
            
    Returns:
        open3d.geometry.PointCloud: The loaded or converted point cloud, colored green with normals.
        
    Raises:
        FileNotFoundError: If the file does not exist.
        ValueError: If the file extension is unsupported.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
        
    ext = os.path.splitext(file_path)[1].lower()
    
    if ext == ".pcd":
        pcd = o3d.io.read_point_cloud(file_path)
    elif ext == ".obj":
        # Try to load as a mesh.
        mesh = o3d.io.read_triangle_mesh(file_path)
        if len(mesh.vertices) > 0:
            if len(mesh.triangles) > 0:
                # Mesh with faces: sample a point cloud.
                if number_of_points is None:
                    number_of_points = 10000  # Default sample count.
                pcd = mesh.sample_points_uniformly(number_of_points=number_of_points)
            else:
                # Vertex-only OBJ: use vertices directly.
                pcd = o3d.geometry.PointCloud()
                pcd.points = mesh.vertices
            # If the point cloud is still empty, fall back to manual parsing.
            if len(pcd.points) == 0:
                pcd = _load_obj_as_point_cloud(file_path)
        else:
            # Fallback: manually parse the OBJ file.
            pcd = _load_obj_as_point_cloud(file_path)
    else:
        raise ValueError(f"Unsupported file extension: {ext}")
    
    # Color the point cloud green.
    green = [0, 1, 0]  # RGB for green.
    num_points = len(pcd.points)
    if num_points > 0:
        pcd.colors = o3d.utility.Vector3dVector([green] * num_points)
    
    # Estimate normals. Adjust the radius and max_nn as needed for your data scale.
    if num_points > 0:
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    return pcd

def _load_obj_as_point_cloud(file_path):
    """
    Manually parses an OBJ file to extract vertex data (lines starting with "v ")
    and creates a point cloud.
    
    Parameters:
        file_path (str): Path to the OBJ file.
        
    Returns:
        open3d.geometry.PointCloud: The created point cloud.
    """
    points = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith("v "):  # Look for vertex definitions.
                parts = line.strip().split()
                if len(parts) >= 4:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        points.append([x, y, z])
                    except ValueError:
                        continue
    pcd = o3d.geometry.PointCloud()
    if points:
        pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def compute_actual_path(matrices):
    """
    Computes the actual path from relative transformations.
    
    Parameters:
        matrices (np.ndarray): Array of relative transformation matrices.
        
    Returns:
        line_set (o3d.geometry.LineSet): Line set representing the path.
        coordinate_frames (list): List of coordinate frames at each pose.
    """
    transforms = []
    current_transform = np.eye(4)
    total_transform = []
    
    for idx, transform in enumerate(matrices):
        if idx == 0:
            current_transform = np.eye(4)
        else:
            current_transform = current_transform @ np.linalg.inv(transform)
        total_transform.append(current_transform)

    actual_positions = np.array([t[:3, 3] for t in total_transform])
    print(actual_positions)

    lines = [[i, i+1] for i in range(len(actual_positions)-1)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(actual_positions)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1] for _ in lines])  # blue color for path

    # Create coordinate frames at each pose
    coordinate_frames = []
    for transform in total_transform:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=transform[:3, 3])
        frame.rotate(transform[:3, :3], center=transform[:3, 3])
        coordinate_frames.append(frame)
    
    return line_set, coordinate_frames

def compute_absolute_poses(matrices, interval=0.5):
    """
    Computes absolute poses from a CSV of transforms.
    Assumes the first matrix in `matrices` is the base (absolute) pose,
    and each subsequent matrix is a relative transform to be composed.
    
    Parameters:
        matrices (np.ndarray): Array of 4x4 matrices (first row absolute,
                               the rest relative).
        interval (float): Minimum distance interval between selected poses.
        
    Returns:
        List[np.ndarray]: List of compounded absolute 4x4 transformation matrices.
    """
    # Use the first matrix as the base absolute pose.
    cumulative_transform = matrices[0].copy()
    absolute_transforms = [cumulative_transform.copy()]
    positions = [cumulative_transform[:3, 3]]
    
    # Compound each subsequent relative transform.
    for i in range(1, len(matrices)):
        # Assuming matrices[i] is the relative transform from the previous pose.
        cumulative_transform = cumulative_transform @ np.linalg.inv(matrices[i])
        absolute_transforms.append(cumulative_transform.copy())
        positions.append(cumulative_transform[:3, 3])
    
    positions = np.array(positions)
    
    # Select poses based on the specified interval.
    selected_indices = [0]
    last_distance = 2.0
    # Compute cumulative distances along the path.
    cumulative_dists = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    cumulative_dists = np.insert(np.cumsum(cumulative_dists), 0, 0)
    for i in range(1, len(cumulative_dists)):
        if cumulative_dists[i] - last_distance >= interval:
            selected_indices.append(i)
            last_distance = cumulative_dists[i]
            
    # Return only the selected absolute transforms.
    selected_transforms = [absolute_transforms[i] for i in selected_indices]
    return selected_transforms

def indent(elem, level=0):
    """
    Recursively adds indentation to an XML element.
    """
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for child in elem:
            indent(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def create_launch_file(idx, pose, output_file):
    """
    Creates a launch file with specified poses.
    
    Parameters:
        poses (list): List of poses (4x4 transformation matrices).
        output_file (str): Path to the output XML file.
    """
    launch = ET.Element("launch")

    # Compute pose values
    x, y, z = pose[:3, 3]
    yaw = np.arctan2(pose[1, 0], pose[0, 0])
    
    # Static arguments with computed values
    ET.SubElement(launch, "arg", name="config", default="base")
    ET.SubElement(launch, "arg", name="x", default=str(x))
    ET.SubElement(launch, "arg", name="y", default=str(y))
    ET.SubElement(launch, "arg", name="z", default=str(z))
    ET.SubElement(launch, "arg", name="yaw", default=f"{yaw:.8f}")

    
    # Include description and associated argument
    include_description = ET.SubElement(launch, "include", file="$(find warthog_description)/launch/ghost_description.launch")
    ET.SubElement(include_description, "arg", name="config", value="$(arg config)")
    
    # Additional nodes and includes
    #ET.SubElement(launch, "node", name="$(anon joint_state_publisher)", pkg="joint_state_publisher", type="joint_state_publisher")
    #ET.SubElement(launch, "include", file="$(find warthog_control)/launch/ghost_control.launch")
    
    ET.SubElement(
        launch,
        "node",
        name=f"urdf_spawner_{idx}",
        pkg="gazebo_ros",
        type="spawn_model",
        args=f"-urdf -model ghost_warthog_{idx} -param robot_description -x $(arg x) -y $(arg y) -z $(arg z) -R 0 -P 0 -Y $(arg yaw)"
    )
    
    # Pretty-print the XML tree
    indent(launch)
    tree = ET.ElementTree(launch)
    with open(output_file, 'wb') as f:
        tree.write(f, xml_declaration=True, encoding='utf-8', method="xml")

def create_combined_launch_file(num_files, output_file):
    """
    Creates a combined launch file with include tags for each generated file.
    
    Parameters:
        num_files (int): Number of individual launch files to include.
        output_file (str): Path to the output combined launch file.
    """
    launch = ET.Element("launch")
    
    # Global simulation arguments
    ET.SubElement(launch, "arg", name="use_sim_time", default="true")
    ET.SubElement(launch, "arg", name="gui", default="true")
    ET.SubElement(launch, "arg", name="headless", default="false")
    ET.SubElement(launch, "arg", name="world_name", default="worlds/empty.world")
    
    # Include Gazebo world launch with nested arguments
    include_gazebo = ET.SubElement(launch, "include", file="$(find gazebo_ros)/launch/empty_world.launch")
    ET.SubElement(include_gazebo, "arg", name="debug", value="0")
    ET.SubElement(include_gazebo, "arg", name="gui", value="$(arg gui)")
    ET.SubElement(include_gazebo, "arg", name="use_sim_time", value="$(arg use_sim_time)")
    ET.SubElement(include_gazebo, "arg", name="headless", value="$(arg headless)")
    ET.SubElement(include_gazebo, "arg", name="world_name", value="$(arg world_name)")
    
    # Comment for clarity
    launch.append(ET.Comment("Add Warthog robots"))
    
    # Include each individual launch file with explicit closing tags
    for idx in range(num_files):
        include = ET.Element("include", file=f"$(find warthog_gazebo)/launch/dome_ghost_spawners/spawn_ghost_warthog_{idx}.launch")
        # Add an empty text node to force a separate closing tag.
        include.text = ""
        launch.append(include)
    
    # Pretty-print the XML tree
    indent(launch)
    tree = ET.ElementTree(launch)
    with open(output_file, 'wb') as f:
        tree.write(f, xml_declaration=True, encoding='utf-8', method="xml", short_empty_elements=False)

#############################################################################################################################################################################################################################################################
# Load the Path
odometry_csv_path2 = "/home/desiree/ASRL/vtr3/data/feb19Dome5th/nerf_gazebo_relative_transforms.csv"
relative_transforms2 = np.loadtxt(odometry_csv_path2, delimiter=",", skiprows=1)
matrices2 = relative_transforms2[:, 1:].reshape(-1, 4, 4)

line_set2, coordinate_frames = compute_actual_path(matrices2)

#############################################################################################################################################################################################################################################################
# Load OBJ or PLY point cloud
pcd1 = o3d.io.read_point_cloud("/home/desiree/ASRL/vtr3/data/feb19Dome5th/point_cloud.pcd")
#/home/desiree/ASRL/vtr3/data/parking2/point_cloud.pcd
#/home/desiree/ASRL/vtr3/data/feb19Dome5th/point_cloud.pcd

#mesh = o3d.io.read_triangle_mesh("/home/desiree/ASRL/Thesis/BuddySystemDatasets/feb19Dome/meshes/nerfacto5th/cropped/FINALOBJ.obj")
'''
#############################  DOME  #######################################################
# pcd1.scale(14.4901, center=(0, 0, 0))
# pcd1 = transform_point_cloud(pcd1, 1, 189.245, 'x')
# pcd1 = transform_point_cloud(pcd1, 1, 1.56437, 'y')
# pcd1 = transform_point_cloud(pcd1, 1, -179.333, 'z')
# # Input specific x, y, z translations for the point cloud
# x_translation = 63.2813  # Replace with your desired x translation
# y_translation = -31.1737   # Replace with your desired y translation
# z_translation = 8.08844   # Replace with your desired z translation
# pcd1 = translate_point_cloud(pcd1, (x_translation, y_translation, z_translation))

###############################  FEB19DOME NERFCTO HUGE #######################################################
# pcd1.scale(60, center=(0, 0, 0))
# pcd1 = transform_point_cloud(pcd1, 1, -51.712, 'x')
# pcd1 = transform_point_cloud(pcd1, 1, -8.16658, 'y')
# pcd1 = transform_point_cloud(pcd1, 1, -12.1107, 'z')
# # Input specific x, y, z translations for the point cloud
# x_translation = 0  # Replace with your desired x translation
# y_translation = -0.092411    # Replace with your desired y translation
# z_translation = -0.066359   # Replace with your desired z translation
# pcd1 = translate_point_cloud(pcd1, (x_translation, y_translation, z_translation))

###############################  FEB19DOME NERFCTO 5th #######################################################
# pcd1.scale(60, center=(0, 0, 0))
# pcd1 = transform_point_cloud(pcd1, 1, -46.5641, 'x')
# pcd1 = transform_point_cloud(pcd1, 1, -9.30854, 'y')
# pcd1 = transform_point_cloud(pcd1, 1, -17.148, 'z')
# # Input specific x, y, z translations for the point cloud
# x_translation = 0  # Replace with your desired x translation
# y_translation = -2.59181     # Replace with your desired y translation
# z_translation = -5.01977    # Replace with your desired z translation
# pcd1 = translate_point_cloud(pcd1, (x_translation, y_translation, z_translation))

###############################  WOODY  #######################################################
# pcd1.scale(60, center=(0, 0, 0))
# pcd1 = transform_point_cloud(pcd1, 1, -45.5032, 'x')
# pcd1 = transform_point_cloud(pcd1, 1, -8.31418, 'y')
# pcd1 = transform_point_cloud(pcd1, 1, -19.9334, 'z')
# # Input specific x, y, z translations for the point cloud
# x_translation = 0  # Replace with your desired x translation
# y_translation = 0   # Replace with your desired y translation
# z_translation = 0  # Replace with your desired z translation

###############################  PARKING #######################################################
# pcd1.scale(100, center=(0, 0, 0))
# pcd1 = transform_point_cloud(pcd1, 1, -47.3285, 'x')
# pcd1 = transform_point_cloud(pcd1, 1, -7.32503, 'y')
# pcd1 = transform_point_cloud(pcd1, 1, -19.8174, 'z')
# # Input specific x, y, z translations for the point cloud
# x_translation = 0  # Replace with your desired x translation
# y_translation = 2.90695     # Replace with your desired y translation
# z_translation = -3.68714    # Replace with your desired z translation
# pcd1 = translate_point_cloud(pcd1, (x_translation, y_translation, z_translation))

###############################  PRINTING PRESS  #######################################################
#pcd1.scale(100, center=(0, 0, 0))
#pcd1 = transform_point_cloud(pcd1, 1, -47.7598, 'x')
#pcd1 = transform_point_cloud(pcd1, 1, -10.3315, 'y')
#pcd1 = transform_point_cloud(pcd1, 1, -4.67965, 'z')
# Input specific x, y, z translations for the point cloud
#x_translation = -31.4726  # Replace with your desired x translation
#y_translation = 36.3687   # Replace with your desired y translation
#z_translation = -9.40156  # Replace with your desired z translation
#pcd1 = translate_point_cloud(pcd1, (x_translation, y_translation, z_translation))
'''
#############################################################################################################################################################################################################################################################
# Extract the first line of the CSV file to get the absolute pose
absolute_pose = relative_transforms2[0, 1:].reshape(4, 4)

# Apply the absolute pose transformation to the point cloud
pcd1_rebased = pcd1.transform(np.linalg.inv(absolute_pose))
#mesh_rebased = mesh.transform(np.linalg.inv(absolute_pose))

#############################################################################################################################################################################################################################################################
#o3d.visualization.draw_geometries([pcd1, line_set2] + coordinate_frames, window_name="Comparison")

#############################################################################################################################################################################################################################################################
# Save the point cloud pcd1 as a PCD file
#output_pcd_path = "/home/desiree/ASRL/Thesis/BuddySystemDatasets/virtualTeachInputs/PointClouds/grassy/NEW/point_cloud.pcd"
#o3d.io.write_point_cloud(output_pcd_path, pcd1)

#############################################################################################################################################################################################################################################################
# Visualize products going into VTR and Gazebo
#pcd_path = "/home/desiree/ASRL/Thesis/BuddySystemDatasets/virtualTeachInputs/PointClouds/grassy/NEW/point_cloud.pcd"

# Load and optionally normalize point cloud
#o3d_pcd = load_pcd(pcd_path)

#o3d.visualization.draw_geometries([o3d_pcd, line_set2] + coordinate_frames, window_name="Comparison")

#o3d.io.write_line_set("dome_lines.ply", line_set2)

# Compute and print absolute poses at 0.5 m intervals
absolute_poses = compute_absolute_poses(matrices2, interval=0.5)

# Visualize the absolute poses over the point cloud
coordinate_frames_absolute = []
for pose in absolute_poses:
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=pose[:3, 3])
    frame.rotate(pose[:3, :3], center=pose[:3, 3])
    coordinate_frames_absolute.append(frame)
    print(pose)

# Visualize the point cloud with the absolute poses
o3d.visualization.draw_geometries([pcd1] + coordinate_frames_absolute, window_name="Absolute Poses over Point Cloud")

# Create launch files with absolute poses for each index
for idx, pose in enumerate(absolute_poses):
    print(idx)
    output_launch_file = f"/home/desiree/catkin_ws/src/warthog_simulator/warthog_gazebo/launch/dome_ghost_spawners/spawn_ghost_warthog_{idx}.launch"
    create_launch_file(idx, pose, output_launch_file)

# Create the combined launch file
num_files = len(absolute_poses)
combined_launch_file = "/home/desiree/catkin_ws/src/warthog_simulator/warthog_gazebo/launch/combined_dome_spawn_ghost_warthogs.launch"
create_combined_launch_file(num_files, combined_launch_file)
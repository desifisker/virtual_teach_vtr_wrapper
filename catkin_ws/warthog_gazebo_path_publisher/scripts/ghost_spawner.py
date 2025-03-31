'''

#!/usr/bin/env python3
import rospy
import csv
import numpy as np
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
import open3d as o3d

def read_trajectory(csv_file):
    """
    Reads the CSV file and returns a list of cumulative 4x4 transformation matrices.
    
    Assumes:
      - The CSV header is the first row.
      - The first data row is the initial absolute pose, where the first column is a timestamp,
        and the remaining 16 columns represent the flattened 4x4 matrix.
      - Each subsequent row is a relative transform in the same format.
      
    This function computes the cumulative (absolute) pose by multiplying the relative transforms.
    """
    poses = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header row
        try:
            first_row = next(reader)
        except StopIteration:
            rospy.logerr("CSV file is empty after header.")
            return poses
        # Skip the timestamp (first element) and convert the remaining 16 values to a 4x4 matrix
        initial_matrix = np.array(first_row[1:], dtype=float).reshape((4, 4))
        poses.append(initial_matrix)
        cumulative = initial_matrix
        for row in reader:
            # Expecting 17 columns: 1 for timestamp and 16 for the matrix
            if len(row) != 17:
                rospy.logwarn("Skipping row with %d entries (expected 17)", len(row))
                continue
            # Convert the row (excluding the timestamp) to a 4x4 matrix
            rel = np.array(row[1:], dtype=float).reshape((4, 4))
            cumulative = np.dot(cumulative, rel)
            poses.append(cumulative.copy())
    return poses

def plot_trajectory_with_point_cloud(csv_file, point_cloud_file):
    """
    Plots the trajectory read from the CSV file over the provided point cloud.
    """
    # Read the trajectory from the CSV file
    poses = read_trajectory(csv_file)
    if not poses:
        rospy.logerr("No poses found in CSV file!")
        return
    # Compute the actual path
    line_set, coordinate_frames = compute_actual_path(poses)
    # Load the point cloud
    point_cloud = o3d.io.read_point_cloud(point_cloud_file)
    if not point_cloud.has_points():
        rospy.logerr("Point cloud file is empty or invalid: %s", point_cloud_file)
        return
    # Visualize the point cloud and the trajectory
    o3d.visualization.draw_geometries([point_cloud, line_set] + coordinate_frames)

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

def publish_spline_marker(poses):
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spline"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Line width
    marker.color.a = 1.0  # Alpha
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    for matrix in poses:
        point = Point()
        point.x = matrix[0, 3]
        point.y = matrix[1, 3]
        point.z = matrix[2, 3]
        marker.points.append(point)

    marker_pub.publish(marker)

def main():
    rospy.init_node("ghost_spawner")
    # Parameters (set these via a launch file or parameter server as needed)
    csv_file = rospy.get_param("~csv_file", "/home/desiree/ASRL/vtr3/data/feb19Dome5th/nerf_gazebo_relative_transforms.csv")
    point_cloud_file = rospy.get_param("~point_cloud_file", "/home/desiree/ASRL/vtr3/data/feb19Dome5th/point_cloud.pcd")
    plot_trajectory_with_point_cloud(csv_file, point_cloud_file)

    poses = read_trajectory(csv_file)
    if not poses:
        rospy.logerr("No poses found in CSV file!")
        return

    publish_spline_marker(poses)
    rospy.loginfo("Published spline marker for the path.")
    rospy.spin()

if __name__ == '__main__':
    main()




'''
#!/usr/bin/env python3
import rospy
import csv
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from tf.transformations import quaternion_from_matrix

def read_trajectory(csv_file):
    """
    Reads the CSV file and returns a list of cumulative 4x4 transformation matrices.
    
    Assumes:
      - The CSV header is the first row.
      - The first data row is the initial absolute pose, where the first column is a timestamp,
        and the remaining 16 columns represent the flattened 4x4 matrix.
      - Each subsequent row is a relative transform in the same format.
      
    This function computes the cumulative (absolute) pose by multiplying the relative transforms.
    """
    poses = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header row
        try:
            first_row = next(reader)
        except StopIteration:
            rospy.logerr("CSV file is empty after header.")
            return poses
        # Skip the timestamp (first element) and convert the remaining 16 values to a 4x4 matrix
        initial_matrix = np.array(first_row[1:], dtype=float).reshape((4, 4))
        poses.append(initial_matrix)
        cumulative = initial_matrix
        for row in reader:
            # Expecting 17 columns: 1 for timestamp and 16 for the matrix
            if len(row) != 17:
                rospy.logwarn("Skipping row with %d entries (expected 17)", len(row))
                continue
            # Convert the row (excluding the timestamp) to a 4x4 matrix
            rel = np.array(row[1:], dtype=float).reshape((4, 4))
            cumulative = np.dot(cumulative, rel)
            poses.append(cumulative.copy())
    return poses

def matrix_to_pose(matrix):
    """
    Converts a 4x4 transformation matrix to a geometry_msgs/Pose message.
    """
    t = matrix[:3, 3]
    q = quaternion_from_matrix(matrix)
    pose = Pose()
    pose.position = Point(t[0], t[1], t[2])
    pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    return pose

def spawn_ghost_model(model_name, model_xml, pose_msg):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp = spawn_model(model_name, model_xml, "", pose_msg, "world")
        rospy.loginfo("Spawned ghost model: %s", model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: %s", e)

def main():
    rospy.init_node("ghost_spawner")
    # Parameters (set these via a launch file or parameter server as needed)
    csv_file = rospy.get_param("~csv_file", "/home/desiree/ASRL/Thesis/BuddySystemDatasets/WarthogMeshPaths/relative_transforms.csv")
    ghost_model_file = rospy.get_param("~ghost_model_file", "/opt/ros/noetic/share/warthog_description/meshes/ghost_warthog.urdf")
    distance_interval = rospy.get_param("~distance_interval", 0.5)  # in meters

    # Read the ghost URDF XML
    try:
        with open(ghost_model_file, 'r') as f:
            ghost_model_xml = f.read()
    except IOError:
        rospy.logerr("Could not read ghost model file at: %s", ghost_model_file)
        return

    poses = read_trajectory(csv_file)
    if not poses:
        rospy.logerr("No poses found in CSV file!")
        return

    last_spawn_position = poses[0][:3, 3]
    ghost_count = 0

    for matrix in poses:
        current_position = matrix[:3, 3]
        dist = np.linalg.norm(current_position - last_spawn_position)
        if dist >= distance_interval:
            pose_msg = matrix_to_pose(matrix)
            ghost_name = "ghost_warthog_%d" % ghost_count
            spawn_ghost_model(ghost_name, ghost_model_xml, pose_msg)
            ghost_count += 1
            last_spawn_position = current_position

    rospy.loginfo("Spawned %d ghost models along the path.", ghost_count)
    rospy.spin()

if __name__ == '__main__':
    main()



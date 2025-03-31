import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from pylgmath import se3op 

# Load the point cloud from the .ply file
point_cloud = o3d.io.read_point_cloud("/home/desiree/ASRL/Thesis/BuddySystemDatasets/Point Clouds/MarsDomeLidarMap/outputMarsLoop.ply")
#point_cloud_nerf = o3d.io.read_point_cloud("/home/desiree/ASRL/Thesis/BuddySystemDatasets/Point Clouds/warthog_lidar_scaled_Pcloud.ply")
# Compute normals for proper visualization
point_cloud.estimate_normals()
#point_cloud_nerf.estimate_normals()

# Load the relative transforms (6DOF) from the CSV file
odometry_csv_path = "/home/desiree/ASRL/vtr3/data/ExistingGraph/relative_transforms_mat_test.csv"
relative_transforms = np.loadtxt(odometry_csv_path, delimiter=",", skiprows=1)  # Adjust as per CSV format
matrices = relative_transforms[:, 1:].reshape(-1, 4, 4)  # First column is timestamp, rest is 16 matrix elements
print(matrices[0], matrices[1], matrices[2])

# Compute the actual path from relative transformations
transforms = []
current_transform = np.eye(4)
total_transform = []

for transform in matrices:
    #print(current_transform, current_transform.shape)
    current_transform = current_transform @ np.linalg.inv(transform)
    total_transform.append(current_transform)

print('total_transform', total_transform[0], total_transform[1], total_transform[2])

# Extract positions from transforms
actual_positions = np.array([t[:3, 3] for t in total_transform])
print('positions', actual_positions[0], actual_positions[1], actual_positions[2])

# Plot the path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(actual_positions[:, 0], actual_positions[:, 1], actual_positions[:, 2], label='Odometry Path')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('Odometry Path')
plt.show()

# Initialize visualization
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(point_cloud)
#vis.add_geometry(point_cloud_nerf)

# Create a line set from actual positions
lines = [[i, i+1] for i in range(len(actual_positions)-1)]
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(actual_positions)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])  # Red color for path
vis.add_geometry(line_set)

# Set some reasonable default parameters
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])  # Black background
opt.point_size = 1.0  # Adjust the point size if needed

# Run the visualization
vis.run()
vis.destroy_window()

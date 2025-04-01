#!/bin/bash
# launch_warthog.sh - Setup and launch Gazebo world with teleop and path saving

# Usage: ./launch_warthog.sh <mesh_file> <world_name>
if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <mesh_file> <png_file> <world_name>"
  exit 1
fi

MESH_FILE="$1"
PNG_FILE="$2"
WORLD="$3"

# Start the Docker container named virtr_programs.
docker start virtr

# Build the multi-line command to be executed inside the container.
# Note: This script assumes that environment variables VTRSRC and VTRROOT are set in the container.
DOCKER_SCRIPT=$(cat <<EOF
source /opt/ros/noetic/setup.bash &&
echo \$ROS_DISTRO &&
# Create necessary directories
mkdir -p "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/models/${WORLD}/meshes" &&
# Copy the mesh file into the models folder (renamed as mesh.dae)
cp -r "${MESH_FILE}" "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/models/${WORLD}/meshes/mesh.dae" &&
cp -r "${PNG_FILE}" "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/models/${WORLD}/meshes/material_0.png" &&
# Create the .world file with the world name substituted
cat <<WORLD_EOF > "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/worlds/${WORLD}.world"
<sdf version='1.6'>
  <world name='${WORLD}'>
    <!-- Omni Light for Full Uniform Lighting -->
    <light name="full_light" type="point">
      <pose frame="">0 0 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>1.0 1.0 1.0 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>1.0</constant>
        <linear>0.0</linear>
        <quadratic>0.0</quadratic>
      </attenuation>
      <cast_shadows>0</cast_shadows>
    </light>

    <model name='${WORLD}'>
      <static>1</static>
      <link name='press'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://${WORLD}/meshes/mesh.dae</uri>
              <size>1 1</size>
            </mesh>
          </geometry>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <mesh>
              <uri>model://${WORLD}/meshes/mesh.dae</uri>
              <size>1 1</size>
            </mesh>
          </geometry>
        </visual>
        <velocity_decay>
          <linear>0</linear>
          <angular>0</angular>
        </velocity_decay>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <scene>
      <ambient>1.0 1.0 1.0 1</ambient>
      <background>1.0 1.0 1.0 1</background>
      <shadows>0</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>17.8083 2.7942 26.7053 0 0.923643 -3.08299</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
  </world>
</sdf>

WORLD_EOF

# Create the .launch file with the world name substituted
cat <<LAUNCH_EOF > "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/launch/${WORLD}.launch"
<?xml version="1.0"?>
<launch>
  <arg name="use_sim_time" default="true" />
  <arg name="gui" default="true" />
  <arg name="headless" default="false" />
  <arg name="world_name" default="\\\$(find warthog_gazebo)/worlds/${WORLD}.world" />
  <!-- Launch Gazebo with the specified world -->
  <include file="\\\$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="0" />
    <arg name="gui" value="\\\$(arg gui)" />
    <arg name="use_sim_time" value="\\\$(arg use_sim_time)" />
    <arg name="headless" value="\\\$(arg headless)" />
    <arg name="world_name" value="\\\$(arg world_name)" />
  </include>
  <!-- Add a single Warthog robot -->
  <include file="\\\$(find warthog_gazebo)/launch/spawn_warthog.launch" />
</launch>

LAUNCH_EOF

# Create the .launch file with the world name substituted
cat <<CONFIG_EOF > "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/models/${WORLD}/model.config"
<?xml version="1.0" ?>
<model>
  <name>${WORLD}_mesh</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>
    This is the NeRF mesh for Gazebo simulation.
  </description>
</model>

CONFIG_EOF

# Create the .launch file with the world name substituted
cat <<SDF_EOF > "\${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws/src/warthog_simulator/warthog_gazebo/models/${WORLD}/model.sdf"
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="${WORLD}_mesh">
    <static>true</static>
    <link name="mesh_link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://${WORLD}_mesh/meshes/mesh.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://${WORLD}_mesh/meshes/mesh.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
SDF_EOF
EOF
)

# Execute the multi-line command inside the container.
docker exec -it virtr bash -c "$DOCKER_SCRIPT"
EXIT_CODE=$?
exit $EXIT_CODE


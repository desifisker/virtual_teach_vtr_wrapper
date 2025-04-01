#!/usr/bin/env python3
"""
This Blender Python script does the following:
  1. Reads a JSON file to obtain a precise "scale" value.
  2. Imports a mesh file and a point cloud.
  3. Sets the mesh rotation to 0° on all axes.
  4. Sets the mesh scale to the inverse of the JSON "scale" value.
  5. Then scales both the mesh and the point cloud by an additional user‐supplied scale factor.
  
Usage (when run from Blender):
  blender --background --python blender_process.py -- <json_file> <point_cloud> <scale_factor> <mesh_file>
"""

import bpy
import sys
import os

# Install the io_scene_obj addon from a zip file (if needed)
bpy.ops.preferences.addon_install(filepath="/opt/blender-4.3.2-linux-x64/4.3/scripts/addons/io_scene_obj.zip")
# Then enable it.
bpy.ops.preferences.addon_enable(module="io_scene_obj")

# (Continue with your other imports and code below)
import json
import argparse

# Define the path to the built-in addons directory and the io_scene_obj addon file.
blender_addon_dir = "/opt/blender-4.3.2-linux-x64/4.3/scripts/modules/_bpy_internal/addons"

io_obj_init = os.path.join(blender_addon_dir, "io_scene_obj", "__init__.py")

# Append the addons directory to sys.path so Python can find built-in addons.
if blender_addon_dir not in sys.path:
    sys.path.append(blender_addon_dir)

# Attempt to manually load the io_scene_obj module if it exists.
if os.path.exists(io_obj_init):
    spec = importlib.util.spec_from_file_location("io_scene_obj", io_obj_init)
    io_scene_obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(io_scene_obj)
    sys.modules["io_scene_obj"] = io_scene_obj
    print("io_scene_obj module loaded from file.")
else:
    print("io_scene_obj addon not found at:", io_obj_init)

# Now try to enable the OBJ importer addon using the new operator for Blender 2.8+.
try:
    bpy.ops.preferences.addon_enable(module="io_scene_obj")
except Exception as e:
    print("Failed to enable io_scene_obj addon via preferences operator:", e)

# Check if the addon is enabled.
if "io_scene_obj" in bpy.context.preferences.addons:
    print("io_scene_obj addon is enabled.")
else:
    print("io_scene_obj addon is not enabled.")

import json
import argparse

def import_object(filepath):
    """Helper function that imports a file based on its extension and returns the imported object(s)."""
    # For OBJ files, attempt to enable the addon (again) in case it's needed.
    if os.path.splitext(filepath)[1].lower() == ".obj":
        try:
            bpy.ops.preferences.addon_enable(module="io_scene_obj")
        except Exception as e:
            print("Failed to enable io_scene_obj addon in import_object:", e)
    
    ext = os.path.splitext(filepath)[1].lower()
    before = set(bpy.context.scene.objects)
    if ext == ".obj":
        bpy.ops.import_scene.obj(filepath=filepath)
    elif ext == ".ply":
        bpy.ops.import_mesh.ply(filepath=filepath)
    else:
        print("Unsupported file format:", ext)
        return None
    after = set(bpy.context.scene.objects)
    new_objs = list(after - before)
    if not new_objs:
        print("No objects were imported from", filepath)
        return None
    return new_objs

def main():
    # Blender passes extra args after '--'
    argv = sys.argv
    if "--" not in argv:
        argv = []  # no args
    else:
        argv = argv[argv.index("--") + 1:]
    
    parser = argparse.ArgumentParser(
        description="Process mesh and point cloud in Blender using JSON scale."
    )
    parser.add_argument("json_file", type=str, help="Path to the JSON file (contains key 'scale').")
    parser.add_argument("point_cloud", type=str, help="Path to the point cloud file.")
    parser.add_argument("scale_factor", type=float, help="Additional scale factor to apply to both objects.")
    parser.add_argument("mesh_file", type=str, help="Path to the mesh file.")
    
    args = parser.parse_args(argv)
    
    # Read the JSON file and extract the precise "scale" value.
    with open(args.json_file, 'r') as f:
        data = json.load(f)
    if "scale" not in data:
        print("Error: JSON does not contain a 'scale' key.")
        return
    json_scale = data["scale"]
    
    # Import the mesh.
    mesh_objs = import_object(args.mesh_file)
    if not mesh_objs:
        print("Failed to import mesh.")
        return
    mesh_obj = mesh_objs[0]
    
    # Reset mesh rotation.
    mesh_obj.rotation_euler = (0.0, 0.0, 0.0)
    
    # Set mesh scale: (1 / json_scale) * scale_factor.
    base_scale = 1.0 / json_scale
    final_scale = base_scale * args.scale_factor
    mesh_obj.scale = (final_scale, final_scale, final_scale)
    
    # Import the point cloud.
    pc_objs = import_object(args.point_cloud)
    if not pc_objs:
        print("Failed to import point cloud.")
        return
    pc_obj = pc_objs[0]
    # Scale the point cloud directly by scale_factor.
    pc_obj.scale = (args.scale_factor, args.scale_factor, args.scale_factor)
    
    bpy.context.view_layer.update()
    print("Blender processing complete.")
    
if __name__ == "__main__":
    main()


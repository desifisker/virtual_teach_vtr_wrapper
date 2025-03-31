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
import json
import sys
import argparse
import os

def import_object(filepath):
    """Helper function that imports a file based on its extension and returns the imported object(s)."""
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
    # No rounding is applied—json_scale retains its full precision.
    
    # Import the mesh.
    mesh_objs = import_object(args.mesh_file)
    if not mesh_objs:
        print("Failed to import mesh.")
        return
    # For this example, assume the first imported mesh is the one to process.
    mesh_obj = mesh_objs[0]
    
    # Set mesh rotation to (0,0,0) (in radians).
    mesh_obj.rotation_euler = (0.0, 0.0, 0.0)
    
    # Set mesh scale to the inverse of the JSON scale value.
    # Then multiply that scale by the additional scale factor.
    base_scale = 1.0 / json_scale
    final_scale = base_scale * args.scale_factor
    mesh_obj.scale = (final_scale, final_scale, final_scale)
    
    # Import the point cloud.
    pc_objs = import_object(args.point_cloud)
    if not pc_objs:
        print("Failed to import point cloud.")
        return
    # Assume the first imported object is the point cloud.
    pc_obj = pc_objs[0]
    # Scale the point cloud by the additional scale factor.
    pc_obj.scale = (args.scale_factor, args.scale_factor, args.scale_factor)
    
    # Update the scene.
    bpy.context.view_layer.update()
    print("Blender processing complete.")
    
if __name__ == "__main__":
    main()


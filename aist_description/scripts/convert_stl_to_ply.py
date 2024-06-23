#!/usr/bin/env python3
# Reads every STL file in the meshes folder and saves it as a PLY file.

import open3d as o3d
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

directory = os.path.join(get_package_share_directory('aist_description'),
                         'parts', 'meshes')

nothing_to_be_done = True
# First check if we need to do anything
for filename in os.listdir(directory):
    if filename.endswith("STL") or filename.endswith("stl"):
        stl_filepath = os.path.join(directory, filename)
        ply_filepath = Path(stl_filepath).with_suffix(".ply")
        if not os.path.exists(str(ply_filepath)):
            nothing_to_be_done = False  # There are files to convert
            break

if nothing_to_be_done:
    sys.exit()

print("Convert STL to PLY files in " + directory)
for filename in os.listdir(directory):
    if filename.endswith("STL") or filename.endswith("stl"):
        stl_filepath = os.path.join(directory, filename)
        mesh = o3d.io.read_triangle_mesh(stl_filepath)
        ply_filepath = Path(stl_filepath).with_suffix(".ply")
        o3d.io.write_triangle_mesh(str(ply_filepath), mesh)
        print("Wrote " + str(ply_filepath))
sys.exit()

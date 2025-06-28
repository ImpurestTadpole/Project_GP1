#!/usr/bin/env python3
"""Convert a 3D mesh file into a basic Gazebo world (.world) file.

Usage:
    python3 convert_mesh_to_world.py --mesh path/to/mesh.obj --output path/to/world.world --scale 1 1 1

The generated world will contain the mesh as a static model at the origin.
"""
import argparse
from pathlib import Path
import sys

template = """<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='default'>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name='scanned_environment'>
      <static>true</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://{mesh_path}</uri>
              <scale>{scale}</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://{mesh_path}</uri>
              <scale>{scale}</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""

def parse_args():
    parser = argparse.ArgumentParser(description="Wrap a 3D mesh into a Gazebo world file.")
    parser.add_argument("--mesh", required=True, help="Path to the mesh file (OBJ, DAE, PLY, etc.)")
    parser.add_argument("--output", required=True, help="Output .world file path")
    parser.add_argument("--scale", nargs=3, default=["1", "1", "1"], metavar=("X", "Y", "Z"), help="Scale factors for the mesh")
    return parser.parse_args()

def main():
    args = parse_args()
    mesh_path = Path(args.mesh).expanduser().resolve()
    if not mesh_path.exists():
        print(f"Mesh file {mesh_path} not found.")
        sys.exit(1)

    scale = " ".join(args.scale)
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    world_content = template.format(mesh_path=mesh_path.as_posix(), scale=scale)
    output_path.write_text(world_content)
    print(f"Gazebo world saved to {output_path}")

if __name__ == "__main__":
    main() 
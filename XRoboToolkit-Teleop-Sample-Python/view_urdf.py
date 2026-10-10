#!/usr/bin/env python3
"""Quick URDF viewer for the new QJ2M model under assets/qj2m."""
import os

import yourdfpy

ROOT = os.path.dirname(os.path.abspath(__file__))
model_dir = os.path.join(ROOT, "assets/qj2m/urdf")
mesh_dir = os.path.join(ROOT, "assets/qj2m/meshes")
urdf_path = os.path.join(model_dir, "qj2m_teleop.urdf")

robot = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_dir)
robot.show()

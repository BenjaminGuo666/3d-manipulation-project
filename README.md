# 3d-manipulation-project


# 3D Manipulation Project

This repository contains the code for my individual project on **3D reconstruction
and robotic manipulation**.

The goal is to reconstruct a 3D model of an orange from images, import the
reconstructed mesh into a ROS 2 + MoveIt 2 simulation, and program a Panda
robot arm to perform a simple **pick-and-place** task on the reconstructed
object. The project also visualises the trajectory of the orange during the
manipulation using RViz markers.

---

## 1. Environment and Dependencies

The project was developed and tested with the following setup:

- **OS:** Ubuntu 24.04 (WSL2)
- **ROS 2:** Jazzy Jalisco
- **MoveIt 2** (including the Panda demo configuration)
  - `ros-jazzy-moveit`  
  - `ros-jazzy-moveit-resources-panda-moveit-config`
- **Build tools:** `colcon`, `cmake`, `gcc`/`g++`
- **3D reconstruction tools (optional, for re-running SfM):**
  - Any standard SfM/MVS pipeline (e.g. COLMAP / OpenMVG + OpenMVS)
  - A mesh-editing tool for converting to STL (e.g. MeshLab / Blender)

If you only want to run the robotic manipulation, you can use the provided
mesh file in `meshes/orange.stl` and **do not** need to re-run the SfM
pipeline.

---

## 2. Repository Structure

```text
3d-manipulation-project/
├── meshes/
│   └── orange.stl           # Reconstructed and simplified mesh of the orange
├── src/
│   └── sfm_pick_place/
│       ├── src/
│       │   └── add_orange_scene.cpp   # Main C++ node (scene + pick-and-place)
│       ├── include/        # (empty for now, reserved for headers)
│       ├── CMakeLists.txt
│       └── package.xml
├── report/
│   └── technical_report.pdf # LaTeX report (not required to run the code)
└── README.md
























































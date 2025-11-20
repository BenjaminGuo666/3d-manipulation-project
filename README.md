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

## 3. 
# 0) To the home directory (just to be safe)
cd ~

# 1) Create project root directory （In the WSL Linux file system）
mkdir -p ~/projects/3d_manipulation_project
cd ~/projects/3d_manipulation_project

# 2) Create subdirectory structure
mkdir -p reconstruction/{calib,sfm,mvs,export} \
         ros2_ws/src \
         scripts \
         data/{obj1,obj2} \
         docs

# 3) Initialize git repository
git init
echo "# 3D Reconstruction + Manipulation (Coursework)" > README.md
echo ".venv/" >> .gitignore
echo "build/
install/
log/" >> .gitignore

# 4) To reconstruct part of the Python virtual environment 
python3 -m venv .venv
source .venv/bin/activate

# 5) Create 3 package skeletons in a ROS2 workspace
cd ros2_ws/src
ros2 pkg create --build-type ament_python object_loader --dependencies rclpy geometry_msgs moveit_msgs
ros2 pkg create --build-type ament_python grasp_planner   --dependencies rclpy geometry_msgs sensor_msgs visualization_msgs
ros2 pkg create --build-type ament_python manipulation_demo --dependencies rclpy moveit_msgs moveit_commander geometry_msgs

# 6) Return to the workspace root, first build
cd ..
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# 7)  Go back to the project root and place a minimal script as a placeholder
cd ~/projects/3d_manipulation_project
printf '#!/usr/bin/env bash\nset -e\n# TODO: obj_name from args\n' > scripts/run_reconstruction.sh
chmod +x scripts/run_reconstruction.sh

printf '#!/usr/bin/env bash\nset -e\nsource /opt/ros/jazzy/setup.bash\ncd ros2_ws\nsource install/setup.bash\n# TODO: launch RViz/MoveIt demo\n' > scripts/run_demo.sh
chmod +x scripts/run_demo.sh

## 4. # Return to the project root
cd ~/projects/3d_manipulation_project

# Calibration and Photo Directory
mkdir -p reconstruction/calib/images

## 5. reconstruction/calib/calib_chessboard.py






























































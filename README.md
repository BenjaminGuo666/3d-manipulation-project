
---

# 3D Manipulation Project

This repository contains the code, data and instructions for my individual project on **3D reconstruction for robotic manipulation**.

The goals are:

1. Reconstruct 3D models of an orange from images using **camera calibration + COLMAP (SfM + MVS)**.
2. Clean the reconstructed mesh and convert it to **STL**.
3. Import the mesh into a **ROS 2 + MoveIt 2** simulation with a **Panda** robot arm.
4. Program a **pick-and-place** manipulation and visualise the motion in RViz.

---

## 1. Environment and Dependencies

The project was developed and tested with the following setup:

- **OS**
  - Ubuntu 24.04 under WSL2 (Windows Subsystem for Linux)

- **ROS 2**
  - Jazzy Jalisco
```bash
# 1) Set locale (only needed once on a fresh system)
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2) Add ROS 2 apt repository
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3) Install ROS 2 Jazzy (desktop)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# 4) Add ROS 2 setup to your shell (optional but convenient)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

- **MoveIt 2 (Panda demo)**
  - `ros-jazzy-moveit`
  - `ros-jazzy-moveit-resources-panda-moveit-config`
  ```bash
  sudo apt update
  sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-resources-panda-moveit-config
  ```


- **Build tools (for the ROS 2 workspace)**
  - `colcon`, `cmake`, `gcc`, `g++`
  ```bash
  sudo apt update
  sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions
  ```
  Then build the workspace:
  ```bash
  cd ~/projects/3d_manipulation_project/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build
  source install/setup.bash
  ```

- **3D reconstruction tools**
  - **COLMAP** (used both on Windows and Ubuntu)
    ```bash
    sudo apt update
    sudo apt install -y colmap
    colmap -h | head -n 3
    ```
  - **MeshLab** (or Blender) for mesh cleaning and conversion to STL  
    ```bash
    sudo snap install meshlab
    ```
    or install via GUI / other methods.

- **Python**
  - System Python 3 on WSL (`/usr/bin/python3`).
  - (Optional) virtual environment:
    ```bash
    cd ~/projects/3d_manipulation_project
    python3 -m venv .venv
    source .venv/bin/activate
    ```

> If you only want to run the **robotic manipulation demo**, you can directly use
> the provided mesh file `ros2_ws/src/sfm_pick_place/meshes/orange.stl` and
> **do not** need to re-run the SfM/MVS pipeline.

---

## 2. Repository Structure

From inside the WSL Linux home directory:

```text
~/projects/3d_manipulation_project/
├── .git/                      # Git metadata
├── .venv/                     # Optional Python virtual environment
├── data/                      # Raw image datasets (downsampled image sequences)
│   ├── obj3_small/
│   ├── obj4_small/
│   └── obj5_small/
│
├── reconstruction/            # Complete 3D reconstruction pipeline
│   ├── calib/                 # Camera calibration
│   │   ├── images/            # Original chessboard images
│   │   ├── images_small/      # Downsampled images for calibration
│   │   ├── calib_chessboard.py
│   │   ├── show_chessboard.py
│   │   └── camera.yaml        # Calibration result (intrinsics + distortion)
│   │
│   ├── sfm/                   # Structure-from-Motion projects (COLMAP)
│   │   ├── obj3_small/
│   │   ├── obj4_small/
│   │   └── obj5_small/
│   │       ├── 0/
│   │       ├── 1/
│   │       └── 2/              # One of these contains the best model:
│   │           ├── cameras.bin
│   │           ├── images.bin  # Number of registered images
│   │           ├── points3D.bin
│   │           └── project.ini
│   │
│   ├── mvs/                   # Multi-View Stereo workspaces
│   │   ├── obj3_small/
│   │   ├── obj4_small/
│   │   └── obj5_small/
│   │       ├── images/        # Input images (undistorted)
│   │       ├── sparse/        # Copied sparse model
│   │       ├── stereo/        # MVS intermediate data
│   │       ├── fused.ply      # Dense fused point cloud
│   │       ├── object5.ply    # Cleaned densed fused point mesh
│   │       ├── fused.ply.vis
│   │       ├── run-colmap-geometric.sh
│   │       └── run-colmap-photometric.sh
│   │
│   └── export/                # Cleaned / exported meshes
│       ├── obj1.obj
│       └── obj2.obj
│
├──  ros2_ws/                   # ROS 2 workspace for the manipulation demo（another independent package）
│   ├── src/
│   │   └── sfm_pick_place/
│   │       ├── meshes/
│   │       │   └── orange.stl       # Final mesh used in simulation
│   │       ├── src/
│   │       │   └── add_orange_scene.cpp  # Main C++ node (scene + pick-and-place)
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   └── ...                     # build/, install/, log/ etc. (created by colcon)
│
├── docs/                      # Report, slides and other documentation
│   └── technical_report.pdf
│
├── scripts/                   # Helper scripts (optional)
│
└── README.md                  # This file
````

---

## 3. Project Initialisation (One-time)

Run these commands in the **Ubuntu terminal** under WSL:

```bash
# 0) Go to the home directory (just to be safe)
cd ~

# 1) Create project root directory (in the WSL Linux file system)
mkdir -p ~/projects/3d_manipulation_project
cd ~/projects/3d_manipulation_project

# 2) Create subdirectory structure
mkdir -p reconstruction/{calib,sfm,mvs,export} \
         ros2_ws/src \
         scripts \
         data/{obj1,obj2} \
         docs

# 3) Initialise git repository
git init
echo "# 3D Reconstruction + Manipulation (Coursework)" > README.md
{
  echo ".venv/"
  echo "build/"
  echo "install/"
  echo "log/"
} >> .gitignore

# 4) (Optional) Create Python virtual environment
python3 -m venv .venv
source .venv/bin/activate
```

You can also create additional ROS 2 Python packages if needed (my final submission mainly uses the C++ node `sfm_pick_place/add_orange_scene.cpp`, so the following is optional):

```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python object_loader      --dependencies rclpy geometry_msgs moveit_msgs
ros2 pkg create --build-type ament_python grasp_planner      --dependencies rclpy geometry_msgs sensor_msgs visualization_msgs
ros2 pkg create --build-type ament_python manipulation_demo  --dependencies rclpy moveit_msgs moveit_commander geometry_msgs

cd ..
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Minimal helper scripts (optional):

```bash
cd ~/projects/3d_manipulation_project

# Reconstruction script placeholder
cat << 'EOF' > scripts/run_reconstruction.sh
#!/usr/bin/env bash
set -e
# TODO: obj_name from args
EOF
chmod +x scripts/run_reconstruction.sh

# Demo script placeholder
cat << 'EOF' > scripts/run_demo.sh
#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
# TODO: launch RViz/MoveIt demo
EOF
chmod +x scripts/run_demo.sh
```

---
## 4. How to Use This Repository

### 4.1 Only run the robot demo

If you only care about the robotic manipulation demo:

1. Make sure `ros2_ws/src/sfm_pick_place/meshes/orange.stl` exists (provided).
2. Follow **Section 8** to build and run the ROS 2 demo.
3. You **do not** need to run any of the COLMAP or calibration steps.

### 4.2 Reproduce the full pipeline

To reproduce the complete pipeline:

1. Place calibration images into `reconstruction/calib/images/` and run the calibration script to obtain `camera.yaml`.
2. Place your object images into `data/objX_small/`.
3. Run SfM with COLMAP (Section 5) and select the best sparse model.
4. Create the MVS workspace and run PatchMatch Stereo + Fusion (Section 6).
5. Clean and export the mesh to STL (Section 7).
6. Copy the STL file to `ros2_ws/src/sfm_pick_place/meshes/orange.stl`.
7. Rebuild the ROS 2 workspace and run the pick-and-place demo (Section 8).

## 5. Camera Calibration

Return to the project root:

```bash
cd ~/projects/3d_manipulation_project
mkdir -p reconstruction/calib/images
```

1. Use `reconstruction/calib/show_chessboard.py` to visualise the chessboard pattern (ensure the script exists).

2. Take photos of the calibration chessboard and save them into:

   ```text
   reconstruction/calib/images/
   ```

3. Run the calibration script:

   ```bash
   python3 reconstruction/calib/calib_chessboard.py \
     --images reconstruction/calib/images \
     --pattern_cols 9 --pattern_rows 6 \
     --square 0.024 \
     --out reconstruction/calib/camera.yaml \
     --viz
   ```

This generates `camera.yaml` and an example image `undistort_sample.jpg`.

---

## 6. COLMAP SfM, Undistortion and MVS Workspace (Linux)

1. Place object images into:

   ```text
   data/obj1_small/
   data/obj2_small/
   # (or obj3_small / obj4_small / obj5_small depending on which object)
   ```

2. run COLMAP **feature extraction**:

   ```bash
   colmap feature_extractor \
     --database_path reconstruction/sfm/obj1_small/db.db \
     --image_path    data/obj1_small \
     --SiftExtraction.use_gpu=0 \
     --SiftExtraction.max_image_size=2000 \
     --SiftExtraction.max_num_features=8000
   ```

3. **Sequential matching**:

   ```bash
   colmap sequential_matcher \
     --database_path reconstruction/sfm/obj1_small/db.db \
     --SequentialMatching.overlap=6
   ```

4. **Mapping (SfM)**:

   ```bash
   colmap mapper \
     --database_path reconstruction/sfm/obj1_small/db.db \
     --image_path    data/obj1_small \
     --output_path   reconstruction/sfm/obj1_small
   ```

5. In `reconstruction/sfm/obj1_small/`, find the subdirectory (`0`, `1`, `2`, …) that contains the **largest** `images.bin` (most registered images). Call this directory `X`.

6. **Undistort** and create an MVS workspace:

   ```bash
   colmap image_undistorter \
     --image_path  data/obj1_small \
     --input_path  reconstruction/sfm/obj1_small/X \
     --output_path reconstruction/mvs/obj1_small \
     --max_image_size=2000
   ```

---

## 7. PatchMatch Stereo and Fusion (Windows PowerShell)

The dense reconstruction is run via COLMAP on Windows, accessing the WSL path.

1. Set the workspace path (example):

   ```powershell
   $ws = "\\wsl$\Ubuntu-24.04\root\projects\3d_manipulation_project\reconstruction\mvs\obj1_small"
   ```

2. Provide COLMAP executable:

   ```powershell
   $colmap = "C:\Path\To\colmap.bat"   # change to your actual path
   ```

3. **PatchMatch Stereo** and **Fusion**:

   ```powershell
   & "$colmap" patch_match_stereo --workspace_path "$ws" --PatchMatchStereo.gpu_index=0 --PatchMatchStereo.max_image_size=2000
   & "$colmap" stereo_fusion      --workspace_path "$ws" --output_path "$ws\fused.ply"
   ```

4. Open the fused point cloud/mesh in MeshLab:

   ```bash
   meshlab /root/projects/3d_manipulation_project/reconstruction/mvs/obj1_small/fused.ply
   ```

---

## 8. Mesh Cleaning and Export

In MeshLab (or Blender):

1. Open `reconstruction/mvs/objX_small/fused.ply`.

2. Remove floating noise and background.

3. Fill holes and simplify the mesh.

4. Make sure the scale is reasonable (orange roughly real size).

5. Export the cleaned mesh as:

   ```text
   reconstruction/export/obj1.obj
   reconstruction/export/obj2.obj
   ```

6. Convert the chosen OBJ file to STL and copy it into the ROS 2 package:

   ```text
   ros2_ws/src/sfm_pick_place/meshes/orange.stl
   ```

---

## 9. ROS 2 Manipulation Demo

### 9.1 Build the workspace

```bash
cd ~/projects/3d_manipulation_project/ros2_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build only the sfm_pick_place package
colcon build --packages-select sfm_pick_place

# Source the workspace
source install/setup.bash
```

### 9.2 Start Panda + MoveIt + RViz

In a **new terminal**:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

### 9.3 Run the pick-and-place node

In another terminal:

```bash
cd /ros2_ws
source install/setup.bash
ros2 run sfm_pick_place add_orange_scene
```

The C++ node `add_orange_scene.cpp` will:

1. Load the orange mesh `meshes/orange.stl`.
2. Add the orange as a collision object in the planning scene.
3. Move the Panda arm to a pre-grasp pose.
4. Move down to the grasp pose and **attach** the orange to the gripper.
5. Lift the orange, translate it to a new position, and lower it.
6. **Detach** the orange and leave it on the table.
7. Return the arm to a named configuration (`ready`).

---





# Classwork 2 — Armando: ArUco Markers and KDL IK

ROS 2 simulation and control pipeline for the **Armando** robotic arm. The system detects ArUco fiducial markers from a simulated RGB camera in Gazebo, computes a safe approach target pose in front of the selected marker, and drives the arm to reach it using KDL inverse kinematics.

---

## 1. System Architecture

The integrated pipeline connects perception and inverse kinematics through three main nodes:

1. **ArUco Detector (`aruco_ros` — `marker_publisher`)**
   - Subscribes to `/camera/image_raw` and `/camera/camera_info`.
   - Estimates the 6-DoF pose of visible markers and publishes them as `aruco_msgs/msg/MarkerArray` on `/aruco_marker_publisher/markers` expressed in `base_link`.
2. **Marker-to-Target Adapter (`armando_controller` — `armando_marker_adapter`)**
   - Subscribes to `/aruco_marker_publisher/markers` and filters detections by `target_id`.
   - Applies a rigid offset transformation $T_{\text{marker}}^{\text{target}}$ with respect to the marker frame: a translation of $d$ along the marker's outward normal ($+z$) and a rotation of $\pi\text{ rad}$ around the $x$-axis so that the end-effector's $z$-axis points toward the marker.
   - Publishes the resulting `geometry_msgs/msg/PoseStamped` on `/target_pose`.
3. **KDL Inverse Kinematics Solver (`armando_controller` — `armando_ik_kdl_node`)**
   - Subscribes to `/target_pose`, solves position IK for the chain `base_link` $\rightarrow$ `crawer_base`, checks joint limits, and sends trajectories to the arm controller.

---

## 2. ArUco Markers & Design Choices

### Marker Specifications
The custom Gazebo marker models are packaged inside `armando_description/models/` and included in `armando_workbench.sdf`:

| Parameter | Value | Notes |
| :--- | :--- | :--- |
| **OpenCV Dictionary** | `<YOUR_DICTIONARY, e.g., DICT_4X4_50>` | Used to generate the PNG textures |
| **Placed Marker ID(s)** | `4` *(add other IDs if present)* | Documented in `armando_workbench.sdf` |
| **Black Square Side (`marker_size`)** | `0.10 m` | Exact size of the inner black border passed to `aruco_ros` |
| **Total Plane Side ($L$)** | `<YOUR_L, e.g., 0.12 m>` | Full texture size (including white margin) in `model.sdf` |

### Target Offset ($d$)
- **Chosen value:** `offset_d = 0.10 m` *(update if you chose a different value)*
- **Rationale:** Placing the target $10\text{ cm}$ along the marker's normal axis ensures the end-effector stops cleanly in front of the marker without colliding with the workbench surface, while keeping the target well inside the reachable workspace of the Armando arm.

---

## 3. Repository Structure

```text
.
├── docker_scripts/
│   ├── docker_build_image.sh
│   ├── docker_run_container.sh
│   └── docker_connect.sh
└── src/
    ├── armando_description/
    │   ├── models/aruco_marker_<ID>/     # Custom Gazebo marker model & PNG texture
    │   └── worlds/armando_workbench.sdf  # World with placed ArUco marker(s)
    └── armando_controller/
        ├── src/armando_marker_adapter.cpp
        └── launch/armando_pipeline.launch.py
```

---

## 4. Setup & Running from a Clean Machine

### Prerequisites
- Linux machine with **Docker** and **Git** installed (X11 forwarding enabled for Gazebo GUI).

### Step 1: Clone the Repository
```bash
git clone https://github.com/<YOUR_USERNAME>/<YOUR_REPO_NAME>.git
cd <YOUR_REPO_NAME>
```

### Step 2: Build and Start the Docker Container
```bash
cd docker_scripts
./docker_build_image.sh
./docker_run_container.sh
```
> **Note:** `docker_run_container.sh` mounts `src/` into `/root/ros2_ws/src` inside the container.

### Step 3: Build the ROS 2 Workspace
Inside the container terminal, compile and source the workspace:
```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

### Step 4: Launch the Gazebo Simulation (Terminal 1)
Start Gazebo with the robot and workbench world:
```bash
ros2 launch armando_description armando_gazebo.launch.py
```
Make sure the simulation is **playing** (not paused) in the Gazebo GUI.

### Step 5: Launch the Detection & IK Pipeline (Terminal 2)
Open a second terminal on the host, connect to the running container, and launch the full pipeline specifying the desired `target_id`:
```bash
cd <YOUR_REPO_NAME>/docker_scripts
./docker_connect.sh
source /root/ros2_ws/install/setup.bash
ros2 launch armando_controller armando_pipeline.launch.py target_id:=4
```

### Configurable Launch Arguments
You can override the default parameters directly from the command line:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `target_id` | `0` | ID of the ArUco marker to track and reach |
| `offset_d` | `0.1` | Distance $d$ [m] along the marker's normal ($+z$) axis |
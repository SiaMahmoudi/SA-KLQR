# SA-KLQR ROS1 Workspace  
Force–Tilt–Centroid Adaptive Control for UR5e Swabbing
<img width="3544" height="1768" alt="Swab_exp" src="https://github.com/user-attachments/assets/9be07c8f-25f9-4917-8e1a-2dee452d357b" />


This repository contains a complete ROS Noetic implementation of the **Surface-Aware Koopman LQR (SA-KLQR)** controller developed for robotic swabbing tasks.  
The system integrates:

- Force sensing via an Arduino-based FSR pad  
- Surface plane detection using Intel RealSense and Open3D  
- 6-state Koopman region models with LQR gains  
- UR5e control through `/scaled_pos_joint_trajectory_controller`  
- Full data logging for training new Koopman operators  

---

# 1. System Overview

FSR Sensor  →  Force + Centroid  
RealSense   →  Plane Normal (tilt_x, tilt_y)  
SA-KLQR     →  6-state vector (force, centroid, tilt, derivative)  
UR5e        →  Joint command via scaled_pos_joint_trajectory_controller  

All components run as ROS nodes.
<img width="2368" height="1792" alt="framework" src="https://github.com/user-attachments/assets/02174463-95e0-446a-99b9-6b7030c9c298" />

---

# 2. Installation

## 2.1 Install dependencies

sudo apt update  
sudo apt install ros-noetic-desktop-full ros-noetic-ur-robot-driver ros-noetic-realsense2-camera python3-serial python3-scipy python3-numpy  
pip3 install open3d  

## 2.2 Build

cd ~/catkin_ws  
catkin_make  
source devel/setup.bash  

---

# 3. FSR Sensor (Arduino Nano)

Arduino prints 64 comma-separated force values (8×8 grid).  
ROS publishes:

- /fsr/force_map  
- /fsr/force_total  

Launch:

roslaunch fsr_sensor_pkg fsr.launch  

---

# 4. RealSense Plane Detection

Uses Open3D RANSAC to compute:

- Plane normal
- Tilt components (tilt_x, tilt_y)

Published topics:

- /surface_tilt  
- /surface_normal  

Launch:

roslaunch realsense_plane_pkg plane_detection.launch  

---

# 5. SA-KLQR Controller

Builds 6-state vector:

[force_error, cx_err, cy_err, tilt_x, tilt_y, force_derivative]

Launch:

roslaunch sa_klqr_controller sa_klqr_full.launch  

Publishes:

- /sa_klqr/state_vector  
- /sa_klqr/control_output  
- /sa_klqr/enable  

---

# 6. Swab Orchestrator

Enables KLQR when:

1. Force OK  
2. Plane detected  

Launch:

roslaunch swab_orchestrator_pkg swab_full_system.launch  

---

# 7. Data Logging

Creates dataset for Koopman training:

roslaunch data_collection_pkg data_collection.launch  

Data saved to:

~/sa_klqr_logs/YYYYMMDD_HHMMSS/swab_data.csv  

---

# 8. Koopman Operators

Stored in:

sa_klqr_controller/config/koopman_ops/

Each file contains A (6×6) and B (6×1).  

Use placeholder generator:

python3 generate_koopman_placeholders.py  

---

# 9. Run Full System

roslaunch swab_orchestrator_pkg swab_full_system.launch  

---

# 10. Directory Structure

sa_klqr_controller/  
fsr_sensor_pkg/  
realsense_plane_pkg/  
swab_orchestrator_pkg/  
data_collection_pkg/  
utils_pkg/  

---

# 11. Troubleshooting

If no FSR:

ls /dev/ttyUSB*  

If no plane detection:

rostopic echo /camera/depth/color/points  

If robot not moving:

rostopic list | grep scaled_pos  

If KLQR not active:

/sa_klqr/enable must be TRUE  

---

# 12. License

MIT License

---

# 13. Contact

Add your name & email here.

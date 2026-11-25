# RAMABOT V3 — Hardware Implementation & Autonomous Mapping

Repositori implementasi perangkat keras **RAMABOT V3** berbasis **NVIDIA Jetson Xavier NX** (ROS 2 Humble) dan **M5Stack Core Grey** (Closed-Loop Encoder + IMU Odometry & PID Controller).

## Struktur Repositori
- `ros2_packages/mapping_bot/` : Paket ROS 2 Humble (Serial Odometry Bridge, RPLidar A2M8, SLAM Toolbox, Nav2, Foxglove Bridge).
- `firmware_m5stack/` : Kode sumber Arduino C++ untuk M5Stack Core Grey (2X Quadrature Encoder, MPU6886 IMU Fusion, ESP-NOW Teleop, Web OTA).

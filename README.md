# RAMABOT V3 — Autonomous SLAM & Nav2 Mapping Implementation

Implementasi sistem pemetaan otonom (**SLAM Toolbox**) dan navigasi (**ROS 2 Nav2**) pada robot *differential drive* **RAMABOT V3** menggunakan arsitektur komputasi dua tingkat antara **NVIDIA Jetson Xavier NX** dan **M5Stack Core Grey (ESP32)**.

## Arsitektur Sistem

* **High-Level Compute (NVIDIA Jetson Xavier NX — ROS 2 Humble):**
  * `mapping_odom_bridge.py`: Jembatan serial dua arah (`/dev/ttyUSB1`, `115200 bps`) untuk konversi `/cmd_vel` dan publikasi `/odom`, `/joint_states`, serta TF `odom -> base_footprint`.
  * `sllidar_ros2`: Akuisisi data LiDAR **Slamtec RPLidar A2M8** (`/dev/ttyUSB0`, frame `laser_frame`).
  * `slam_toolbox` & `nav2_bringup`: Pemetaan *real-time* dan perencanaan lintasan otonom berbasis DWB *local planner*.
  * `foxglove_bridge`: Visualisasi telemetri, URDF, peta, dan pengiriman *2D Goal Pose* via WebSocket port `8765`.
* **Low-Level Controller (M5Stack Core Grey — FreeRTOS Dual-Core):**
  * **Core 1 (200 Hz IMU / 50 Hz Control):** Akuisisi MPU6886, interupsi *2X quadrature encoder* (`CPR = 1189.64`), fusi odometri 2D *Runge-Kutta Midpoint*, dan kendali PWM BTS7960 (`R = 0.0675 m`, `W = 0.2350 m`).
  * **Core 0 (UI & Comms):** Antarmuka serial ROS 2 REP-103, penerima ESP-NOW (**M5StickC Plus + Hat Mini JoyC**), serta web dashboard & OTA (`[http://ramabot-map.local](http://ramabot-map.local)`).

## Konvensi Koordinat (ROS REP-103)

Antarmuka komunikasi serial antara M5Stack Core Grey dan ROS 2 telah diselaraskan dengan standar tangan kanan **REP-103** (`+X` depan, `+Y` kiri, `+Z` atas, `+yaw` CCW):
* Kinematika invers `/cmd_vel` menghitung kecepatan roda kiri dan kanan sebagai `vL = vx - 0.5 * vth * W` dan `vR = vx + 0.5 * vth * W`.
* *Heading Lock PID* lokal dinonaktifkan otomatis saat perintah `/cmd_vel` ROS 2 aktif agar tidak berkonflik dengan Nav2.

## Prosedur Operasional

1. **Build Workspace:** `cd ~/ros2_ws && colcon build --packages-select mapping_bot --symlink-install`
2. **Jalankan Robot Bringup (Terminal 1):** `source ~/ros2_ws/install/setup.bash && ros2 launch mapping_bot robot_bringup_launch.py`
3. **Reset Pose Odometri (Terminal 2):** `source ~/ros2_ws/install/setup.bash && ros2 service call /reset_odom std_srvs/srv/Trigger "{}"`
4. **Jalankan SLAM & Nav2 (Terminal 2):** `source ~/ros2_ws/install/setup.bash && ros2 launch mapping_bot nav2_launch.py`
5. **Visualisasi Foxglove Studio:** Hubungkan ke `ws://192.168.137.8:8765`, atur **Fixed frame** dan **Display frame** ke `map` (atau `odom`), lalu kirim target melalui **2D Goal Pose**.

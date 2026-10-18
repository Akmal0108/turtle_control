# Turtle Control ROS 2

Proyek ini dibuat menggunakan ROS 2 dan bahasa pemrograman C++. Program ini menggunakan Turtlesim untuk menggerakkan turtle secara otomatis mengikuti pola tertentu, salah satunya pola spirograph.

## Persiapan

Pastikan Ubuntu sudah terpasang ROS 2, Turtlesim, serta alat yang diperlukan untuk melakukan kompilasi menggunakan colcon.

## Cara Menjalankan Program

1. Masuk ke workspace ROS 2.

   ```bash
   cd ~/ros2_ws
   ```

2. Kompilasi paket.

   ```bash
   colcon build --packages-select turtle_control
   ```

3. Aktifkan environment ROS 2 dan workspace.

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

   Sesuaikan `humble` dengan distribusi ROS 2 yang digunakan.

4. Buka terminal baru dan jalankan Turtlesim.

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run turtlesim turtlesim_node
   ```

5. Di terminal sebelumnya, jalankan program spirograph.

   ```bash
   ros2 run turtle_control spirograph
   ```

   Pastikan nama executable sesuai dengan yang tercantum di `CMakeLists.txt`.

## File Program

- `spirograph.cpp`: program untuk menggambar pola spirograph.
- `circle_driver.cpp`: program untuk menggerakkan turtle membentuk lingkaran.
- `square_driver.cpp`: program untuk menggerakkan turtle membentuk persegi.
- `go_to_goal.cpp`: program untuk menggerakkan turtle menuju titik tujuan.
- `pose_listener.cpp`: program untuk menerima informasi posisi dan orientasi turtle.

## Tujuan Proyek

Proyek ini merupakan bagian dari penugasan modul 2. Melalui simulasi Turtlesim, proyek ini membantu memahami komunikasi antar-node, penggunaan topic dan service, serta pengendalian gerakan robot.

## Link Video Penjelasan

Link: https://youtu.be/9zccgU-BSCg?si=WSbDNBkQEKqHOyTc
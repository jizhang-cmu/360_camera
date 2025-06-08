Clone the repository and follow the readme in 'src/receive_theta' to set up the 360 camera driver. Install the dependencies. Then, go to the ROS workspace ('360_camera' folder) and compile.

  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Copy the 'start_360_camera.desktop' file in the 'desktop_button' folder to desktop. Edit the 2 links to the '360_camera_sensorpod.sh' script and 'start.png' icon in the desktop file. Note that if running on the AI computer with the 360 camera plugged into the AI computer, point the link to the '360_camera_ai_computer.sh' script instead. Right-click the desktop file and 'Allow Launching'. Double-click to launch the 360 camera driver.

To calibrate the lidar-to-camera extrinsics and the camera latency, follow the readme in 'src/extrinsic_latency_calib'.

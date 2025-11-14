Install dependencies with the command lines below.
```
  sudo apt update
  sudo apt install libgstreamer1.0-0 libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

In a terminal, go to the 'receive_theta/dependency/libuvc-theta' folder and install 'libuvc-theta'.
```
  mkdir build && cd build
  cmake ..
  make && sudo make install
  sudo /sbin/ldconfig -v
```

In a terminal, go to the 'receive_theta/dependency/gstthetauvc/thetauvc' folder and install 'gstthetauvc'. After installation, use `gst-inspect-1.0 thetauvcsrc` to check if the gstreamer plugin is valid.
```
  make && sudo make install
```

Plug the camera into the computer with a USB cable, turn on the camera, and set it to live video mode (click the 'Mode' button until you see 'LIVE' under the camera icon on the screen). Now, use the command line below to view images.
```
  gst-launch-1.0 -v thetauvcsrc ! h264parse ! decodebin ! autovideosink
```

In a terminal, go to the '360_camera' ROS workspace folder and compile.
```
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the ROS workspace and launch the driver (make sure the camera is on/active and in live video mode).
```
  source install/setup.sh
  ros2 launch receive_theta receive_theta_sensorpod.launch
```

The image messages are published on the '/camera/image' topic. To show images in a window, set 'showImage = true' in 'receive_theta_sensorpod.launch'. The top and bottom margin is set to 'TopBottonMargin = 160', cropping from 960 pixels/180 degs to 640 pixels/120 degs vertically. Set 'imageLatency' to adjust image message timestamps if needed.

## Notes

The image framerate is set to 10Hz (skipFrameNum = 2) in 'gstthetauvcsrc.c'. It can be set up to 30Hz. The image resolution is set to FHD (1920 * 960) by default in 'thetauvc.h'. It can be set to UHD (3840 * 1920). Make sure to remake and reinstall 'gstthetauvc'.

## Credits

[libuvc-theta](https://github.com/ricohapi/libuvc-theta) and [gstthetauvc](https://github.com/nickel110/gstthetauvc) packages are from open-source releases.

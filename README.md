# Red Object Detector (ROS2 Humble)

This ROS2 Humble package detects red objects in real-time using the Intel RealSense D405 camera and publishes their 3D coordinates.

---

## Features
- Real-time red object detection.
- Computes 3D positions (X, Y, Z) relative to the camera frame.
- Publishes detected positions to a ROS2 topic `/detected_red_object`.
- Clean, lightweight, easy to install.

---

## Requirements

| Software | Required Version |
|:---|:---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| RealSense SDK | 2.53.1 |
| RealSense ROS2 Wrapper | Version/tag compatible with SDK 2.53.1 (example: 4.51.1) |

✅ Ensure RealSense D405 camera is connected via USB.

---

## Installation

1. **Install ROS2 Humble** following the official guide:  
   [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. **Install Intel RealSense SDK 2.53.1**:

```bash
sudo apt update
sudo apt install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

mkdir -p ~/realsense_ws
cd ~/realsense_ws
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.53.1

mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install


3. Install RealSense ROS2 Wrapper (realsense2_camera) version compatible with SDK 2.53.1:

cd ~/your_ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout 4.51.1

cd ~/your_ros2_ws
colcon build --symlink-install
source install/setup.bash


4. Clone this package:

cd ~/your_ros2_ws/src
git clone https://github.com/ARMLaboratory/red_object_detector.git
cd ~/your_ros2_ws
colcon build --symlink-install
source install/setup.bash



Usage

1. Launch the RealSense camera driver:

ros2 launch realsense2_camera rs_launch.py


 2.   Run the red object detector node:

ros2 run red_object_detector red_detector_node

 3.   (Optional) View detected positions:

ros2 topic echo /detected_red_object

Node Information
Topic	Type	Purpose
/camera/color/image_rect_raw	sensor_msgs/Image	RGB camera stream
/camera/depth/image_rect_raw	sensor_msgs/Image	Depth camera stream
/camera/color/camera_info	sensor_msgs/CameraInfo	Camera intrinsic parameters
/detected_red_object	geometry_msgs/Point	Published detected 3D point




License

This project is licensed under the MIT License.
Acknowledgments

    Intel RealSense ROS2 Wrapper

    OpenCV

    ROS2 Humble Community


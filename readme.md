# This is packge for visual seroving for object without prior model

---

## Package Summary
This package combines object recognition, object tracking, point cloud processing and visual servoing to achieve visual servoing for objects without prior model.

object recognition --> obejct tracking --> point cloud extraction -> object pose estimation

---

## Package Dependencies
* ROS Packages
    * [realsense2_camera](https://github.com/intel-ros/realsense)
    * [ros_opencl_caffe](https://github.com/intel/ros_opencl_caffe)
    * [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
* PCL
* OpenCV 3.4.3
* Boost
* VISP

---



---

## How to launch package

1. roslaunch arm_vs track.launch
2. rosservice call /mico_interaction/switch_controllers "controllers_to_be_turned_on: 'velocity'

---

# Visual Seroving for Object without Prior Model

---

## Package Summary

This package combines object recognition, object tracking, point cloud processing and visual servoing to achieve visual servoing for objects without prior model. This pacakge can be used on robotic arm with RGBD sensor attached to the end effector.

Object Recognition --> Obejct Tracking --> Point Cloud Extraction -> Object Pose Estimation

Object recognition and object tracking work hand in hand to enable reliable and uninterrupted tracking of the object of interested. With the bounding box of the tracked object, program can extract the point cloud in the boundng box region from the structured point cloud from RGBD sensor. 

At the beginning of the visual servoing sessiong, a template of the object can be captured and cached for position based visual servoing.

---

## Package Dependencies
* ROS Packages
    * [realsense2_camera](https://github.com/intel-ros/realsense)
    * [ros_opencl_caffe](https://github.com/intel/ros_opencl_caffe)
    * [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
    * [arUco Tag]()
* PCL
* OpenCV 3.4.3 with tracking module 
* Boost
* VISP
* and a package for driving robot

## Hardware Used in Project
1. Realsense D435 RGBD camera
2. Kinova Mico arm

---

## Topics and Nodes

---

## How to launch package

#### Calibrate
1. roslaunch arm_vs calibrate_mico_eye_in_hand.launch

#### Visual Servoing on arUco tag
1. roslaunch arm_vs pbvs_aruco_tag.launch

#### Visual Servoing on object
1. roslaunch arm_vs track.launch
2. rosservice call /mico_interaction/switch_controllers "controllers_to_be_turned_on: 'velocity'

---
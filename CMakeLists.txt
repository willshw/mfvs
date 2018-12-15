cmake_minimum_required(VERSION 2.8.3)
project(mfvs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED
  roscpp

  std_msgs
  geometry_msgs
  sensor_msgs

  std_srvs

  tf2
  tf2_ros
  tf2_eigen

  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  pcl_msgs

  message_filters
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(VISP REQUIRED)

# point OpenCV to the custom version instead of the ros version
find_package(OpenCV 3.4.3 REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH )
set(OpenCV_INCLUDE_DIRS /usr/local/include)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_message_files(
  FILES
  CartVelCmd.msg
  BBox.msg
)

add_service_files(
  FILES
  SetObjPtCldTemplate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
  LIBRARIES arm-vs ${OpenCV_LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs pcl_ros message_filters pcl_msgs pcl_conversions
  DEPENDS OpenCV Eigen3 PCL VISP Boost
)

# PBVS
add_executable(${PROJECT_NAME}_pbvs
  src/pbvs.cpp
)

target_link_libraries(${PROJECT_NAME}_pbvs
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_pbvs
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  arm_vs_generate_messages_cpp
)

# node for rgb tracking
add_executable(${PROJECT_NAME}_obj_image_tracking
  src/obj_image_tracking.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_image_tracking
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_image_tracking
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  # arm_vs_generate_messages_cpp
)

# node for rgb tracking with object recognition
add_executable(${PROJECT_NAME}_obj_image_tracking_w_recognition
  src/obj_image_tracking_w_recognition.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_image_tracking_w_recognition
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_image_tracking_w_recognition
  ${catkin_EXPORTED_TARGETS}
  arm_vs_generate_messages_cpp
)

# node for extracing pointcloud according to rgb obj tracking
add_executable(${PROJECT_NAME}_obj_pointcloud_extraction
  src/obj_pointcloud_extraction.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_pointcloud_extraction
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_pointcloud_extraction
  ${catkin_EXPORTED_TARGETS}
  arm_vs_generate_messages_cpp
)

# tracking pointcloud using template
add_executable(${PROJECT_NAME}_obj_pointcloud_tracking
  src/obj_pointcloud_tracking.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_pointcloud_tracking
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_pointcloud_tracking
  ${catkin_EXPORTED_TARGETS}
)

# align pointcloud using template
add_executable(${PROJECT_NAME}_obj_pointcloud_alignment
  src/obj_pointcloud_alignment.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_pointcloud_alignment
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_pointcloud_alignment
  ${catkin_EXPORTED_TARGETS}
)

# save color cloud to pcd
add_executable(${PROJECT_NAME}_obj_pointcloud_template_save_to_pcd
  src/obj_pointcloud_template_save_to_pcd.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_pointcloud_template_save_to_pcd
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_pointcloud_template_save_to_pcd
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_obj_pointcloud_segmentation
  src/obj_pointcloud_segmentation.cpp
)

target_link_libraries(${PROJECT_NAME}_obj_pointcloud_segmentation
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obj_pointcloud_segmentation
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
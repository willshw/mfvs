cmake_minimum_required(VERSION 2.8.3)
project(arm_vs)

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
find_package(OpenCV 3.4.3 REQUIRED)
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
  Rect.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
  LIBRARIES arm-vs ${OpenCV_LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs pcl_ros message_filters pcl_msgs pcl_conversions
  DEPENDS OpenCV Eigen3 PCL VISP Boost
)

# PBVS
add_executable(${PROJECT_NAME}_pbvs
  src/pbvs_ar_tag.cpp
)

target_link_libraries(${PROJECT_NAME}_pbvs
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_pbvs
  ${catkin_EXPORTED_TARGETS}
  arm_vs_generate_messages_cpp
)

# node for extracing pointcloud according to 2d tracking obj
add_executable(${PROJECT_NAME}_extract_obj_pointcloud
  src/extract_obj_pointcloud.cpp
)

target_link_libraries(${PROJECT_NAME}_extract_obj_pointcloud
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_extract_obj_pointcloud
  ${catkin_EXPORTED_TARGETS}
)

# tracking pointcloud using template
add_executable(${PROJECT_NAME}_track_obj_pointcloud
  src/track_obj_pointcloud.cpp
)

target_link_libraries(${PROJECT_NAME}_track_obj_pointcloud
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_track_obj_pointcloud
  ${catkin_EXPORTED_TARGETS}
)

# align pointcloud using template
add_executable(${PROJECT_NAME}_align_obj_pointcloud
  src/align_obj_pointcloud.cpp
)

target_link_libraries(${PROJECT_NAME}_align_obj_pointcloud
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_align_obj_pointcloud
  ${catkin_EXPORTED_TARGETS}
)

# save color cloud to pcd
add_executable(${PROJECT_NAME}_save_color_cloud
  src/save_to_pcd.cpp
)

target_link_libraries(${PROJECT_NAME}_save_color_cloud
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_save_color_cloud
  ${catkin_EXPORTED_TARGETS}
)

#point cloud tracking
# add_executable(${PROJECT_NAME}_pointcloud_tracking
#   src/pointcloud_tracking.cpp
# )

# target_link_libraries(${PROJECT_NAME}_pointcloud_tracking
#   ${catkin_LIBRARIES}
#   ${roscpp_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME}_pointcloud_tracking
#   ${catkin_EXPORTED_TARGETS}
# )

#visual tracking
# add_executable(${PROJECT_NAME}_cv_tracking
#   src/cv_tracking.cpp
# )

# target_link_libraries(${PROJECT_NAME}_cv_tracking
#   ${catkin_LIBRARIES}
#   ${roscpp_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME}_cv_tracking
#   ${catkin_EXPORTED_TARGETS}
# )


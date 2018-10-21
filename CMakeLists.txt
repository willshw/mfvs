cmake_minimum_required(VERSION 2.8.3)
project(arm_vs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  roscpp

  std_msgs
  geometry_msgs
  sensor_msgs

  tf2
  tf2_ros
  tf2_eigen

  cv_bridge
  image_transport
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(VISP REQUIRED)
find_package(OpenCV 3.4.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES arm-vs
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs pcl_ros
#  DEPENDS system_lib
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

#visual tracking
add_executable(${PROJECT_NAME}_visual_tracking
  src/visual_tracking.cpp
)

target_link_libraries(${PROJECT_NAME}_visual_tracking
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_visual_tracking
  ${catkin_EXPORTED_TARGETS}
)

#point cloud tracking
add_executable(${PROJECT_NAME}_pointcloud_tracking
  src/pointcloud_tracking_node.cpp
)

target_link_libraries(${PROJECT_NAME}_pointcloud_tracking
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_pointcloud_tracking
  ${catkin_EXPORTED_TARGETS}
)

#point cloud tracking
add_executable(${PROJECT_NAME}_2d_tracking
  src/2d_tracking.cpp
)

target_link_libraries(${PROJECT_NAME}_2d_tracking
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_2d_tracking
  ${catkin_EXPORTED_TARGETS}
)
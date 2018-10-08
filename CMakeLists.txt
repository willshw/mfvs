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
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
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

# apriltag_detector
# add_executable(${PROJECT_NAME}_apriltag_detector
#   src/april_proc.cpp
# )

# target_link_libraries(${PROJECT_NAME}_apriltag_detector
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${roscpp_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME}_apriltag_detector
#   ${catkin_EXPORTED_TARGETS}
# )
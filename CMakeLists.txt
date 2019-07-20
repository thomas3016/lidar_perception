cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package (PCL  REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_registration
#   CATKIN_DEPENDS 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  include/lidar_perception
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  
)

add_library(${PROJECT_NAME}
    src/lidar_perception.cpp
    )

target_link_libraries(${PROJECT_NAME}
    #pthread
    stdc++
    #${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
  )


add_executable( ${PROJECT_NAME}_node 
   node/lidar_perception_node.cpp
   )
target_link_libraries(${PROJECT_NAME}_node 
   ${PROJECT_NAME}
   #${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   )




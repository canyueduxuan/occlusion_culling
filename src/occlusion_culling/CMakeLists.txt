cmake_minimum_required(VERSION 3.0.2)
project(occlusion_culling)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_filters
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(occlusion_culling_node src/main.cpp)

target_link_libraries(occlusion_culling_node 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(kinect)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  pcl_msgs
)

include_directories(
  /src
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)


catkin_package()

add_executable(kinect_node src/kinect.cpp)
target_link_libraries(kinect_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


cmake_minimum_required(VERSION 2.8.3)
project(demonstration_application)

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

add_executable(demonstration_application_node src/demonstration_application.cpp)
target_link_libraries(demonstration_application_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


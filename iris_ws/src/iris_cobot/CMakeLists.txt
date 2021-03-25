cmake_minimum_required(VERSION 2.8.3)
project(iris_cobot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
)

catkin_python_setup()

catkin_package()

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer src/viewer.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})



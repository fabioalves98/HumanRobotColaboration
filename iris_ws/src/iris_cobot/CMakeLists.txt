cmake_minimum_required(VERSION 3.2.0)
project(iris_cobot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  moveit_core
  moveit_ros_planning_interface
)

catkin_python_setup()

catkin_package()

find_package(PCL REQUIRED)
find_package(ur_rtde REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer src/viewer.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(fk_ik_jacobian src/fk_ik_jacobian.cpp)
target_link_libraries(fk_ik_jacobian ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(rtde src/rtde.cpp)
target_link_libraries(rtde PRIVATE ur_rtde::rtde ${catkin_LIBRARIES})

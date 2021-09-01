cmake_minimum_required(VERSION 3.2.0)
project(iris_cobot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  moveit_core
  moveit_ros_planning_interface
)

catkin_python_setup()

add_message_files(
    FILES
    JointSpeed.msg
    Obstacles.msg
    PFVector.msg
    TapAction.msg
)

add_service_files(
    FILES
    WeightUpdate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/FT_to_Vel.cfg
  cfg/Jacobian.cfg
  cfg/Clustering.cfg
)

catkin_package(CATKIN_DEPENDS message_runtime)

find_package(PCL REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(Curses REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer src/visualization/viewer.cpp)
target_link_libraries(viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(environment src/environment/environment.cpp)
target_link_libraries(environment ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(obstacle src/environment/obstacle.cpp)
target_link_libraries(obstacle ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(obstacle ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(ft_to_vel src/compliance/ft_to_vel.cpp)
target_link_libraries(ft_to_vel ${catkin_LIBRARIES})
add_dependencies(ft_to_vel ${PROJECT_NAME}_gencfg)

add_executable(attraction src/environment/attraction.cpp)
target_link_libraries(attraction ${catkin_LIBRARIES})

add_executable(jacobian src/compliance/jacobian.cpp)
target_link_libraries(jacobian ${catkin_LIBRARIES})
add_dependencies(jacobian ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(moveit_controller src/compliance/moveit_controller.cpp)
target_link_libraries(moveit_controller ${catkin_LIBRARIES})
add_dependencies(moveit_controller ${PROJECT_NAME}_generate_messages_cpp)

add_executable(joint_vel_cont src/compliance/joint_vel_cont.cpp)
target_link_libraries(joint_vel_cont ${catkin_LIBRARIES})
add_dependencies(joint_vel_cont ${PROJECT_NAME}_generate_messages_cpp)

# Helpers
add_executable(fk_ik_jacobian src/test/fk_ik_jacobian.cpp)
target_link_libraries(fk_ik_jacobian ${catkin_LIBRARIES})

# RTDE Test - Currently not working due to debian release not being up to date
# To make sure it works, build from source
# add_executable(rtde src/test/rtde.cpp)
# target_link_libraries(rtde PRIVATE ur_rtde::rtde ${catkin_LIBRARIES} ${CURSES_LIBRARY})
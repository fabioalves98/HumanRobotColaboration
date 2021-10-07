cmake_minimum_required(VERSION 3.2.0)
project(iris_cobot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  eigen_conversions
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
    WrenchControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/FTtoVel.cfg
  cfg/JointSpeedCont.cfg
  cfg/ObstacleDetection.cfg
  cfg/WrenchFilter.cfg
)

catkin_package(CATKIN_DEPENDS message_runtime)

find_package(PCL REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(Curses REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Hand-Guiding
add_executable(wrench_filter src/cobot/hand_guiding/wrench_filter.cpp)
target_link_libraries(wrench_filter ${catkin_LIBRARIES})
add_dependencies(wrench_filter ${PROJECT_NAME}_gencfg)

add_executable(wrench_to_vel src/cobot/hand_guiding/wrench_to_vel.cpp)
target_link_libraries(wrench_to_vel ${catkin_LIBRARIES})
add_dependencies(wrench_to_vel ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

# Obstacle Detection
add_executable(obstacle_detection src/obstacle_avoidance/obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(obstacle_detection ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(attraction src/obstacle_avoidance/attraction.cpp)
target_link_libraries(attraction ${catkin_LIBRARIES})

# Control
add_executable(jacobian src/control/jacobian.cpp)
target_link_libraries(jacobian ${catkin_LIBRARIES})
add_dependencies(jacobian ${PROJECT_NAME}_generate_messages_cpp)

add_executable(joint_speed_controller src/control/joint_speed_controller.cpp)
target_link_libraries(joint_speed_controller ${catkin_LIBRARIES})
add_dependencies(joint_speed_controller ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(moveit_controller src/control/moveit_controller.cpp)
target_link_libraries(moveit_controller ${catkin_LIBRARIES})
add_dependencies(moveit_controller ${PROJECT_NAME}_generate_messages_cpp)

# rqt_cobot
install(PROGRAMS scripts/rqt_cobot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
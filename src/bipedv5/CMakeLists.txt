cmake_minimum_required(VERSION 2.8.3)
project(bipedv5)
SET(CMAKE_BUILD_TYPE release) #debug
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  roslaunch
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  rviz
  xacro
  tf
  sensor_msgs
  std_msgs
  message_filters
)
find_package(pinocchio REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(sensor src/sensor.cpp)
target_link_libraries(sensor ${catkin_LIBRARIES} pinocchio::pinocchio)

add_executable(drive_gz src/drive_gz.cpp)
target_link_libraries(drive_gz ${catkin_LIBRARIES} )

add_executable(ini_cam_pose src/ini_cam_pose.cpp)
target_link_libraries(ini_cam_pose ${catkin_LIBRARIES} )

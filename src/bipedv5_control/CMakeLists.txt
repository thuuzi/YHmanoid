cmake_minimum_required(VERSION 2.8.3)

project(bipedv5_control)
SET(CMAKE_BUILD_TYPE release) #debug
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  actuatorcontroller_ros
  tf
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bipedv5_control
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    actuatorcontroller_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
   src/inv_kine.cpp
   src/cubic_spline.cpp
   src/swingfoot_trajectory.cpp
   src/leg.cpp
   src/drive.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)





add_executable(drive_gazebo src/drive_gazebo.cpp)
add_dependencies(drive_gazebo ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_gazebo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(drive src/drive.cpp)
add_dependencies(drive ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(motor_read src/motor_read.cpp)
add_dependencies(motor_read ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_read ${PROJECT_NAME} ${catkin_LIBRARIES})


# add_executable(save_data src/save_data.cpp)
# add_dependencies(save_data ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(save_data ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(swingfoot_test src/swingfoot_test.cpp)
add_dependencies(swingfoot_test ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swingfoot_test ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(odom src/odom.cpp)
add_dependencies(odom ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom ${PROJECT_NAME} ${catkin_LIBRARIES})

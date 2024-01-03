cmake_minimum_required(VERSION 2.8.3)
project(state_estimate)
SET(CMAKE_BUILD_TYPE release) #debug
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

generate_dynamic_reconfigure_options(
   cfg/VarianceControl.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimate
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs dynamic_reconfigure
  DEPENDS EIGEN3 PINOCCHIO 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

link_directories(  
${PINOCCHIO_LIBRARY_DIRS})

add_executable(estimate src/serow_driver.cpp src/humanoid_ekf.cpp   src/IMUEKF.cpp  src/MovingAverageFilter.cpp src/CoMEKF.cpp src/differentiator.cpp src/butterworthLPF.cpp src/JointDF.cpp src/butterworthHPF.cpp src/IMUinEKFQuad.cpp src/IMUinEKF.cpp src/ContactEKF.cpp)
target_link_libraries(estimate ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
add_dependencies(estimate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(multisensor_fusion src/multisensor_fusion_driver.cpp src/multisensor_fusion.cpp   src/IMUEKF.cpp  src/MovingAverageFilter.cpp src/CoMEKF.cpp src/CoMLKF.cpp src/differentiator.cpp src/butterworthLPF.cpp src/JointDF.cpp src/butterworthHPF.cpp)
target_link_libraries(multisensor_fusion ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
add_dependencies(multisensor_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

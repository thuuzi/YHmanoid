# Install script for directory: /home/jack/bipedv5/src/lipm_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/msg" TYPE FILE FILES
    "/home/jack/bipedv5/src/lipm_msgs/msg/TrajectoryPoints.msg"
    "/home/jack/bipedv5/src/lipm_msgs/msg/StepTarget.msg"
    "/home/jack/bipedv5/src/lipm_msgs/msg/FootStepPlan.msg"
    "/home/jack/bipedv5/src/lipm_msgs/msg/MPCTraj.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/action" TYPE FILE FILES
    "/home/jack/bipedv5/src/lipm_msgs/action/MotionPlan.action"
    "/home/jack/bipedv5/src/lipm_msgs/action/MotionControl.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/msg" TYPE FILE FILES
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanAction.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanActionGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanActionResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanActionFeedback.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionPlanFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/msg" TYPE FILE FILES
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlAction.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlActionGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlActionResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlActionFeedback.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/lipm_msgs/msg/MotionControlFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/cmake" TYPE FILE FILES "/home/jack/bipedv5/src/build/lipm_msgs/catkin_generated/installspace/lipm_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/include/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/roseus/ros/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/common-lisp/ros/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/gennodejs/ros/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/jack/anaconda3/envs/py38/bin/python3" -m compileall "/home/jack/bipedv5/src/build/devel/lib/python3/dist-packages/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/lib/python3/dist-packages/lipm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jack/bipedv5/src/build/lipm_msgs/catkin_generated/installspace/lipm_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/cmake" TYPE FILE FILES "/home/jack/bipedv5/src/build/lipm_msgs/catkin_generated/installspace/lipm_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs/cmake" TYPE FILE FILES
    "/home/jack/bipedv5/src/build/lipm_msgs/catkin_generated/installspace/lipm_msgsConfig.cmake"
    "/home/jack/bipedv5/src/build/lipm_msgs/catkin_generated/installspace/lipm_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lipm_msgs" TYPE FILE FILES "/home/jack/bipedv5/src/lipm_msgs/package.xml")
endif()


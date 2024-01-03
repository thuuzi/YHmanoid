# Install script for directory: /home/jack/bipedv5/src/whole_body_ik_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/msg" TYPE FILE FILES
    "/home/jack/bipedv5/src/whole_body_ik_msgs/msg/LinearTask.msg"
    "/home/jack/bipedv5/src/whole_body_ik_msgs/msg/AngularTask.msg"
    "/home/jack/bipedv5/src/whole_body_ik_msgs/msg/DOFTask.msg"
    "/home/jack/bipedv5/src/whole_body_ik_msgs/msg/Task.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/action" TYPE FILE FILES "/home/jack/bipedv5/src/whole_body_ik_msgs/action/Humanoid.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/msg" TYPE FILE FILES
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidAction.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidActionGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidActionResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidActionFeedback.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidGoal.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidResult.msg"
    "/home/jack/bipedv5/src/build/devel/share/whole_body_ik_msgs/msg/HumanoidFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/cmake" TYPE FILE FILES "/home/jack/bipedv5/src/build/whole_body_ik_msgs/catkin_generated/installspace/whole_body_ik_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/include/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/roseus/ros/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/common-lisp/ros/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/share/gennodejs/ros/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/jack/anaconda3/envs/py38/bin/python3" -m compileall "/home/jack/bipedv5/src/build/devel/lib/python3/dist-packages/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jack/bipedv5/src/build/devel/lib/python3/dist-packages/whole_body_ik_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jack/bipedv5/src/build/whole_body_ik_msgs/catkin_generated/installspace/whole_body_ik_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/cmake" TYPE FILE FILES "/home/jack/bipedv5/src/build/whole_body_ik_msgs/catkin_generated/installspace/whole_body_ik_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs/cmake" TYPE FILE FILES
    "/home/jack/bipedv5/src/build/whole_body_ik_msgs/catkin_generated/installspace/whole_body_ik_msgsConfig.cmake"
    "/home/jack/bipedv5/src/build/whole_body_ik_msgs/catkin_generated/installspace/whole_body_ik_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/whole_body_ik_msgs" TYPE FILE FILES "/home/jack/bipedv5/src/whole_body_ik_msgs/package.xml")
endif()


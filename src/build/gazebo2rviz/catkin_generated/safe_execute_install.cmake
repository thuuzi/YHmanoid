execute_process(COMMAND "/home/jack/bipedv5/src/build/gazebo2rviz/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jack/bipedv5/src/build/gazebo2rviz/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

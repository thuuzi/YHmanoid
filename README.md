# Bipedv5
# 编译流程
1.安装Pinocchio: sudo apt install ros-$ROS_DISTRO-pinocchio
2.安装qpmad: git clone ...; mkdir build && cd build && cmake .. && make -j4 && sudo make install
3.编译: catkin_make -DCMAKE_BUILD_TYPE=Release

# 传感器频率
IMU：100Hz
力传感器：100Hz
joint_states：1000 Hz 在joint_params.yaml中修改
sensor.cpp pub：1000 Hz

# ORB_SLAM3
rosrun ORB_SLAM3 RGBD ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples/RGB-D/RealSense_D435i.yaml
roslaunch orb_slam3_ros_wrapper bipedv5_slam.launch

# 深度相机
添加相机后gazebo连杆对应编号变了，因此sensor中需要对应更改
频率不够，减小相机数量可以提升频率

# 实物样机
电机ip设置：sudo ifconfig enp4s0 static 192.168.1.111
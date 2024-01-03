#include <bipedv5/robotDyn.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h" 
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include<nav_msgs/Odometry.h>
#include<gazebo_msgs/LinkStates.h>
#include<std_msgs/String.h>
#include<geometry_msgs/WrenchStamped.h>
#include<geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <cmath>
#include "Eigen/Core"
#include "bipedv5/mediator.h"
#include "std_msgs/Bool.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#define Pi 3.14159265359

using namespace std;
using namespace message_filters;
using namespace Eigen;

class Sensor{
    public:
        Sensor();

        void bipedv5IniCb(std_msgs::Bool msg);
        
        void joint_stateCb(const sensor_msgs::JointState msg);

        void model_stateCb(const gazebo_msgs::ModelStates msg);

        void link_stateCb(const gazebo_msgs::LinkStates msg);

        void ftCb(const geometry_msgs::WrenchStampedConstPtr&  msg_l,const geometry_msgs::WrenchStampedConstPtr&  msg_r);

        void pubSensorInfo();

        bool gazebo_launch;

    private:    
        int buffer_num,r_id;
        std::string modelname;
        robotDyn* rd;
        ros::NodeHandle n;
        ros::Publisher js_pub,com_pub,odom_pub,odom_pub_LLeg,odom_pub_RLeg,gait_phase_pub,zmp_pub;
        ros::Publisher pub_lleg1,pub_lleg2,pub_lleg3,pub_lleg4,pub_lleg5,com_position_pub,base_pose_pub,lleg_pose_pub,rleg_pose_pub,
        pub_rleg1,pub_rleg2,pub_rleg3,pub_rleg4,pub_rleg5,
        pub_larm1,pub_larm2,pub_larm3,pub_larm4,
        pub_rarm1,pub_rarm2,pub_rarm3,pub_rarm4;
        ros::Subscriber joint_state_sub,model_state_sub,link_state_sub, bipedv5_sub;
        Eigen::Vector3d vwb_, omegawb_, pwb_,CoM,CoM_last,vCoM,aCoM,vCoM_last;
        Eigen::Quaterniond qwb_;
        sensor_msgs::JointState joint_state_msg;
        gazebo_msgs::LinkStates link_state_msg;
        bool rviz_view,pub_tf,bipedv5_ini;
        double fh_,HX,HY; //足底高度，脚掌中心到la的偏移
        double dt,robot_ini_y,robot_ini_z;
};

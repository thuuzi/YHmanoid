

#include "leg.h"
#include "key.h"
#include <unistd.h>
#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include "Eigen/Dense"
// standard message type
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
// custom message type
#include "actuatorcontroller_ros/ActuatorAttribute.h"
#include "actuatorcontroller_ros/ActuatorCommand.h"
#include "actuatorcontroller_ros/ActuatorModes.h"
#include "actuatorcontroller_ros/ActuatorArray.h"

// custom service type
#include "actuatorcontroller_ros/AttributeLookup.h"
#include "actuatorcontroller_ros/AttributeQuery.h"
#include "actuatorcontroller_ros/GeneralQuery.h"
#include "actuatorcontroller_ros/AttributeDictionary.h"
#include "actuatorcontroller_ros/DebugQuery.h"
#include "actuatorcontroller_ros/TriviaQuery.h"
#include "actuatorcontroller_ros/IDModify.h"
#include "actuatorcontroller_ros/ParametersSave.h"
#include "actuatorcontroller_ros/ZeroReset.h"
#include <geometry_msgs/WrenchStamped.h>
class Drive{
public:
    Drive();
    void setPosition( Eigen::VectorXd pos);
    void state_read( Eigen::VectorXd pos);
    void setRate(double r0_,double r1_,double r2_){
        r0=r0_;
        r1=r1_;
        r2=r2_;
    }
    void setLegParam(double L1_,double L2_){
        L1=L1_;
        L2=L2_;
    }
    void states_cb(const sensor_msgs::JointState& msg);
    
    void iniPosition(Eigen::VectorXd pos,Eigen::VectorXd start_cmd,double Td);

    bool JS_receive;

    sensor_msgs::JointState js_now;

private:
    ros::NodeHandle n;
    ros::Publisher pub_enable, pub_mode, pub_targets,LF_pub,RF_pub,js_pub;
    ros::Subscriber sub_states;
    Eigen::MatrixXd Jl,Jr;  //左右腿雅克比
    Eigen::MatrixXd Fl,Fr,Tr,Tl;  //左右腿雅克比
    double L1,L2;
    double r0,r1,r2; 
};
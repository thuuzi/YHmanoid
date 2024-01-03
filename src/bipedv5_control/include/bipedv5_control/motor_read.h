

#include "leg.h"
#include <unistd.h>
#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

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

class Drive{
public:
    Drive();
    void setPosition( Eigen::VectorXd pos);
    void setRate(double r0_,double r1_,double r2_){
        r0=r0_;
        r1=r1_;
        r2=r2_;
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub_enable, pub_mode, pub_targets;
    double r0,r1,r2;
};
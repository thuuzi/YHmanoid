
#include <unistd.h>
#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "Eigen/Core"
// standard message type
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class Drive{
public:
    Drive();
    void setPosition( Eigen::VectorXd& pos);
    void setPosition(std::vector<double>& pos);
    void setZero();
    void commandJointStatesCb(const sensor_msgs::JointState msg);
    bool get_command;
private:
    ros::NodeHandle n;
    ros::Subscriber command_js_sub;
    ros::Publisher pub_ly,pub_lh,pub_lkp,pub_lk,pub_lap,pub_la,pub_rs,pub_ry,pub_rh,pub_rkp,pub_rk,pub_rap,pub_ra;
};
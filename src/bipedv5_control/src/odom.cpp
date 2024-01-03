#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

void odomCb(const geometry_msgs::PoseStamped msg){
	 //发布tf变换
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = msg.pose.position.x;
	transformStamped.transform.translation.y = msg.pose.position.y;
	transformStamped.transform.translation.z = msg.pose.position.z;
	transformStamped.transform.rotation.x = msg.pose.orientation.x;
	transformStamped.transform.rotation.y = msg.pose.orientation.y;
	transformStamped.transform.rotation.z = msg.pose.orientation.z;
	transformStamped.transform.rotation.w = msg.pose.orientation.w;
	br.sendTransform(transformStamped);
}

//原地踏步测试
int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom");
	ros::NodeHandle n;      
	ros::Subscriber odom_sub = n.subscribe("/orb_slam3/camera_pose",50,odomCb);
	ros::Rate rate(1000);
	ros::spin();
	return 0;
}


#include <bipedv5/sensor.h>

int main(int argc, char  *argv[])
{   
    ros::init(argc,argv,"ini_cam_pose");
    ros::NodeHandle nh;
    ros::Rate rate(50);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher cam_pub = nh.advertise<geometry_msgs::PoseStamped>("/bipedv5/cam_pose", 50);
    while(ros::ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("world", "left_foot_dummy_Link", ros::Time(0));        //代表第二个参数在第一个参数坐标系下的位置
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "camera_rgb";
        odom_msg.pose.pose.position.x = transformStamped.transform.translation.x;
        odom_msg.pose.pose.position.y = transformStamped.transform.translation.y;
        odom_msg.pose.pose.position.z = transformStamped.transform.translation.z;
        odom_msg.pose.pose.orientation.x = transformStamped.transform.rotation.x;
        odom_msg.pose.pose.orientation.y = transformStamped.transform.rotation.y;
        odom_msg.pose.pose.orientation.z = transformStamped.transform.rotation.z;
        odom_msg.pose.pose.orientation.w = transformStamped.transform.rotation.w;
        odom_msg.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = odom_msg.header;
        temp_pose.pose = odom_msg.pose.pose;
        cam_pub.publish(temp_pose);
        
        cout<<"get transform, x:"<<transformStamped.transform.translation.x<<",  y:"<<transformStamped.transform.translation.y<<",  z:"<<transformStamped.transform.translation.z<<endl;
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

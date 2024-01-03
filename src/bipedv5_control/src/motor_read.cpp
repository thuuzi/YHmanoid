#include "bipedv5_control/drive.h"

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_read");
	Drive drive;
	drive.setLegParam(0.29,0.29);
	double r0 = 0.00610865;
	double r1 = 10.0;
	double r2 = 3.5643564356435;
	drive.setRate(r0,r1,r2);
	ros::Rate rate(1000);
	cout<<"loop"<<endl;
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}	
	ros::spin();
	return 0;
}


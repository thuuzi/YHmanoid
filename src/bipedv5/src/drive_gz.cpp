#include "bipedv5/drive_gz.h"

using namespace std;



Drive::Drive(){
	pub_ly = n.advertise<std_msgs::Float64>("/bipedv5/ly/command", 1);
	pub_lh = n.advertise<std_msgs::Float64>("/bipedv5/lh/command", 1);
	pub_lkp = n.advertise<std_msgs::Float64>("/bipedv5/lkp/command", 1);
	pub_lk = n.advertise<std_msgs::Float64>("/bipedv5/lk/command", 1);
	pub_lap = n.advertise<std_msgs::Float64>("/bipedv5/lap/command", 1);
	pub_la = n.advertise<std_msgs::Float64>("/bipedv5/la/command", 1);
	pub_rs = n.advertise<std_msgs::Float64>("/bipedv5/rs/command", 1);
	pub_ry = n.advertise<std_msgs::Float64>("/bipedv5/ry/command", 1);
	pub_rh = n.advertise<std_msgs::Float64>("/bipedv5/rh/command", 1);
	pub_rkp = n.advertise<std_msgs::Float64>("/bipedv5/rkp/command", 1);
	pub_rk = n.advertise<std_msgs::Float64>("/bipedv5/rk/command", 1);
	pub_rap = n.advertise<std_msgs::Float64>("/bipedv5/rap/command", 1);
	pub_ra = n.advertise<std_msgs::Float64>("/bipedv5/ra/command", 1);
	command_js_sub = n.subscribe("/bipedv5/command_joint_states", 1, &Drive::commandJointStatesCb,this);
	get_command=false;
	
}

void Drive::setZero(){
	std_msgs::Float64 command;
	command.data = 0;
	pub_ly.publish(command);
	pub_lh.publish(command);
	pub_lkp.publish(command);
	pub_lk.publish(command);
	pub_lap.publish(command);
	pub_la.publish(command);
	pub_rs.publish(command);
	pub_ry.publish(command);
	pub_rh.publish(command);
	pub_rkp.publish(command);
	pub_rk.publish(command);
	pub_rap.publish(command);
	pub_ra.publish(command);
}

void Drive::commandJointStatesCb(const sensor_msgs::JointState msg)
{
    if(!get_command)
        get_command=true;
    std::vector<double> pos_command = msg.position;
    setPosition(pos_command);
}

void Drive::setPosition(std::vector<double>& pos){
		std_msgs::Float64 command;
		command.data =  pos[7];
		pub_ly.publish(command);
		command.data =  pos[8];
		pub_lh.publish(command);
		command.data = (pos[9]+pos[11])/2;
		pub_lkp.publish(command);
		command.data =  pos[10];
		pub_lk.publish(command);
		command.data = (pos[9]+pos[11])/2;
		pub_lap.publish(command);
		command.data = pos[12];
		pub_la.publish(command);
		command.data =  pos[13];
		pub_ry.publish(command);
		command.data = pos[14];
		pub_rh.publish(command);
		command.data = (pos[15]+pos[17])/2;
		pub_rkp.publish(command);
		command.data = pos[16];
		pub_rk.publish(command);
		command.data = (pos[15]+pos[17])/2;
		pub_rap.publish(command);
		command.data = pos[18];
		pub_ra.publish(command);
}	

void Drive::setPosition(Eigen::VectorXd& pos){
		std_msgs::Float64 command;
		command.data =  pos[0];
		pub_ly.publish(command);
		command.data =  pos[1];
		pub_lh.publish(command);
		command.data = pos[2];
		pub_lkp.publish(command);
		command.data =  pos[3];
		pub_lk.publish(command);
		command.data = pos[4];
		pub_lap.publish(command);
		command.data = pos[5];
		pub_la.publish(command);
		command.data =  pos[6];
		pub_ry.publish(command);
		command.data = pos[7];
		pub_rh.publish(command);
		command.data = pos[8];
		pub_rkp.publish(command);
		command.data = pos[9];
		pub_rk.publish(command);
		command.data = pos[10];
		pub_rap.publish(command);
		command.data = pos[11];
		pub_ra.publish(command);
	}	

int main(int argc, char **argv)
{

	ros::init(argc, argv, "drive_gz");
	bool sz,echo,ini;
	ros::NodeHandle n_p("~");      
	n_p.param<bool>("echo", echo, true);
	n_p.param<bool>("setzero", sz, true);
	n_p.param<bool>("ini", ini, true);
	Drive drive;
	Eigen::VectorXd qn(12);
	qn.setZero(12);
	qn(1) = -0.4;qn(3) = -0.8;qn(5) = 0.4;
	qn(7) = -0.4;qn(9) = -0.8;qn(11) = 0.4;
	ros::Rate rate(1000);
	while(ros::ok()){
	 	if(!drive.get_command){
			drive.setPosition(qn);
	 	}
		rate.sleep();
		ros::spinOnce();
	 }	
	return 0;
}


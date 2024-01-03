#include "bipedv5_control/drive_gazebo.h"

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

void Drive::setPosition(Eigen::VectorXd& pos){
		std_msgs::Float64 command;
		command.data =  pos[0];
		pub_lkp.publish(command);
		command.data =  pos[1];
		pub_lap.publish(command);
		command.data = - pos[2];
		pub_lh.publish(command);
		command.data =  - pos[3];
		pub_lk.publish(command);
		command.data = pos[4];
		pub_la.publish(command);
		command.data = - pos[5];
		pub_rkp.publish(command);
		command.data =  - pos[6];
		pub_rap.publish(command);
		command.data = - pos[7];
		pub_rh.publish(command);
		command.data = - pos[8];
		pub_rk.publish(command);
		command.data = pos[9];
		pub_ra.publish(command);
		command.data = 0;
		pub_ly.publish(command);
		pub_ry.publish(command);
	}	

int main(int argc, char **argv)
{
	sleep(6);
	ros::init(argc, argv, "drive_gazebo");
	double st,dst,sl,sa,hi,L1,L2,dt,sh,il,L3,Lhe, Lto,alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp;
	int pause_time,StepNum;
	bool sz,echo,ini,si,sta,HTwalk;
	ros::NodeHandle n_p("~");      
	n_p.param<bool>("echo", echo, true);
    n_p.param<double>("step_period", st, 6.0);
	n_p.param<double>("double_support_time", dst, 1.0);
	n_p.param<double>("step_len", sl, 0.4);
	n_p.param<double>("sl_dsp", sl_dsp, 0.1);
	n_p.param<double>("step_sa", sa, 0.1);
	n_p.param<double>("ini_height", hi, -0.75);
	n_p.param<double>("step_height", sh, 0.1);
	n_p.param<double>("ini_len", il, 0.1);
	n_p.param<double>("L1", L1, 0.3);
	n_p.param<double>("L2", L2, 0.3);
	n_p.param<double>("L3", L3, 0.1);
	n_p.param<double>("Lhe", Lhe, 0.1);
	n_p.param<double>("Lto", Lto, 0.1);
	n_p.param<double>("alpha_tot", alpha_tot, 0.1);
	n_p.param<double>("alpha_het", alpha_het, 0.1);
	n_p.param<int>("StepNum", StepNum, 8);
	n_p.param<int>("pause_time", pause_time, 10000);
	n_p.param<double>("dt", dt, 0.05);
	n_p.param<double>("toe_tp", toe_tp, 0.5);
	n_p.param<double>("heel_tp", heel_tp, 0.5);
	n_p.param<bool>("setzero", sz, true);
	n_p.param<bool>("ini", ini, true);
	n_p.param<bool>("si", si, false);
	n_p.param<bool>("static", sta, false);
	n_p.param<bool>("Heel_Toe_walk", HTwalk, false);
	Drive drive;
	Leg left_leg(st,dst, sl,sa,hi, il, sh, L1,L2, dt );
	Leg right_leg(st,dst, sl,sa, hi, il, sh, L1,L2, dt );
	Eigen::VectorXd qn,traj_now;
	qn=Eigen::VectorXd::Zero(10);
	left_leg.set_ini_phase(0.0);
	right_leg.set_ini_phase(1.0);
	
	if(HTwalk){
		left_leg.setHTParams( L3,  Lhe,  Lto,alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp);
		right_leg.setHTParams( L3,  Lhe,  Lto , alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp);
	}
	Key key;
	while(ini){
		qn.head(5)=left_leg.iniPose();
		qn.tail(5)=right_leg.iniPose();
		drive.setPosition(qn);
		if(key.readKey())
			break;
	}
	while(left_leg.getStepNum()<StepNum){
	 	if(sz){
			drive.setZero();
	 	}else if(si){
	 		qn.head(5)=left_leg.iniPose();
	 		qn.tail(5)=right_leg.iniPose();
	 		drive.setPosition(qn);
	 	}else if(sta){
			qn.head(5)=left_leg.StaticStep();
			qn.tail(5)=right_leg.StaticStep();
			drive.setPosition(qn);
		}else{
			qn.head(5)=left_leg.Step();
			qn.tail(5)=right_leg.Step();
			drive.setPosition(qn);
		}
		traj_now = left_leg.getTrajNow();
		if(echo){
			cout<<"origin qn: "<<qn[0]<<","<<qn[1]<<","<<qn[2]<<","<<qn[3]<<","<<qn[4]<<","<<qn[5]<<","<<qn[6]<<","<<qn[7]<<","<<qn[8]<<","<<qn[9]<<endl;
			cout<<"traj, x:"<<traj_now[0]<<", z: "<<traj_now[1]<<endl;
		}
		usleep(pause_time);
	 }	
	ros::spin();
	return 0;
}


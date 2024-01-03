#include "bipedv5_control/drive.h"

using namespace std;



Drive::Drive(){
	JS_receive=false;
	Jl=Eigen::MatrixXd::Zero(2,2);Jr=Eigen::MatrixXd::Zero(2,2);
	Tl=Eigen::VectorXd::Zero(2);Tr=Eigen::VectorXd::Zero(2);Fl=Eigen::VectorXd::Zero(2);Fr=Eigen::VectorXd::Zero(2);
	LF_pub = n.advertise<geometry_msgs::WrenchStamped>("bipecv5/LF",1);
	RF_pub = n.advertise<geometry_msgs::WrenchStamped>("bipecv5/RF",1);
	pub_enable = n.advertise<actuatorcontroller_ros::ActuatorArray>("/INNFOS/enableActuator", 1000);
	pub_mode = n.advertise<actuatorcontroller_ros::ActuatorModes>("/INNFOS/setControlMode", 1000);
	pub_targets = n.advertise<sensor_msgs::JointState>("/INNFOS/actuator_targets", 1000);
	sub_states = n.subscribe("/INNFOS/actuator_states",50,&Drive::states_cb,this);
	js_pub = n.advertise<sensor_msgs::JointState>("/bipedv5/joint_states",1000);
	actuatorcontroller_ros::ActuatorArray array;
	array.JointIDs.push_back(0);
	for(int i=1;i<=5;i++){
		pub_enable.publish(array);
		sleep(1);
	}

	actuatorcontroller_ros::ActuatorModes mode;
	mode.JointIDs.push_back(1);
	mode.JointIDs.push_back(2);
	mode.JointIDs.push_back(3);
	mode.JointIDs.push_back(4);  
	mode.JointIDs.push_back(5);  
	mode.JointIDs.push_back(6);
	mode.JointIDs.push_back(7);
	mode.JointIDs.push_back(8);  
	mode.JointIDs.push_back(9);  
	mode.JointIDs.push_back(10); 
	mode.JointIDs.push_back(11); 
	mode.JointIDs.push_back(12);                  
	mode.ActuatorMode = 4;//梯形位置		2是位置模式
	pub_mode.publish(mode);
	sleep(1);
	
	// actuatorcontroller_ros::ActuatorModes mode2;
	// mode2.JointIDs.push_back(1);
	// mode2.JointIDs.push_back(2);
	// mode2.JointIDs.push_back(3);
	// mode2.JointIDs.push_back(4);  
	// mode2.JointIDs.push_back(5);  
	// mode2.JointIDs.push_back(6);
	// mode2.JointIDs.push_back(7);
	// mode2.JointIDs.push_back(8);  
	// mode2.JointIDs.push_back(9);  
	// mode2.JointIDs.push_back(10);                  
	// mode2.ActuatorMode = 2;//2是位置模式
	// pub_mode.publish(mode2);
	// sleep(1);
}

void Drive::setPosition(Eigen::VectorXd pos){
		sensor_msgs::JointState state;
		state.header.stamp = ros::Time::now();
		state.name.push_back("1");	//lkp
		state.name.push_back("2");	//lap
		state.name.push_back("3");  //lh
		state.name.push_back("4");  //lk
		state.name.push_back("5");  //la
		state.name.push_back("7");  //rkp
		state.name.push_back("8"); 	//rap
		state.name.push_back("9");  //rh
		state.name.push_back("10"); 	//rk
		state.name.push_back("11"); 	//ra
		
		state.position.push_back(pos[0]/r0);
		state.position.push_back(pos[1]/r0);
		state.position.push_back(pos[2]/r1);
		state.position.push_back(-pos[3]/r2);
		state.position.push_back(pos[4]/r2);
		state.position.push_back(-pos[5]/r0);
		state.position.push_back(-pos[6]/r0);
		state.position.push_back(-pos[7]/r1);
		state.position.push_back(pos[8]/r2);
		state.position.push_back(-pos[9]/r2);
		
		for(int i=1;i<=10;i++)	
		{
			state.velocity.push_back(0);
			state.effort.push_back(0);
		}
		pub_targets.publish(state);
		//cout<<"command : "<<pos[0]<<","<<pos[1]<<","<<pos[2]<<","<<pos[3]<<","<<pos[4]<<","<<pos[5]<<endl;
	}

//初始化插值
void Drive::iniPosition(Eigen::VectorXd pos,Eigen::VectorXd start_cmd,double Td){
		sensor_msgs::JointState state;
		state.header.stamp = ros::Time::now();
		state.name.push_back("1");	//lkp
		state.name.push_back("2");	//lap
		state.name.push_back("3");  //lh
		state.name.push_back("4");  //lk
		state.name.push_back("5");  //la
		state.name.push_back("7");  //rkp
		state.name.push_back("8"); 	//rap
		state.name.push_back("9");  //rh
		state.name.push_back("10"); 	//rk
		state.name.push_back("11"); 	//ra
		state.position.push_back(Td*(pos[0]/r0)+(1-Td)*start_cmd(0));
		state.position.push_back(Td*(pos[1]/r0)+(1-Td)*start_cmd(1));
		state.position.push_back(Td*(pos[2]/r1)+(1-Td)*start_cmd(2));
		state.position.push_back(Td*(-pos[3]/r2)+(1-Td)*start_cmd(3));
		state.position.push_back(Td*(pos[4]/r2)+(1-Td)*start_cmd(4));
		state.position.push_back(Td*(-pos[5]/r0)+(1-Td)*start_cmd(5));
		state.position.push_back(Td*(-pos[6]/r0)+(1-Td)*start_cmd(6));
		state.position.push_back(Td*(-pos[7]/r1)+(1-Td)*start_cmd(7));
		state.position.push_back(Td*(pos[8]/r2)+(1-Td)*start_cmd(8));
		state.position.push_back(Td*(-pos[9]/r2)+(1-Td)*start_cmd(9));
		
		for(int i=1;i<=10;i++)	
		{
			state.velocity.push_back(0);
			state.effort.push_back(0);
		}
		pub_targets.publish(state);
		cout<<"--------- t:"<<Td<<"command : "<<Td*(pos[2]/r1)+(1-Td)*start_cmd(2)<<endl;
}
	
void Drive::states_cb(const sensor_msgs::JointState& msg){
	JS_receive = true;
	js_now = msg;
	sensor_msgs::JointState state;
	state.header.stamp = ros::Time::now();
	state.name.push_back("ly");
	state.name.push_back("lh");	
	state.name.push_back("lkp"); 
	state.name.push_back("lk");  
	state.name.push_back("lap");  
	state.name.push_back("la"); 
	state.name.push_back("ry"); 	
	state.name.push_back("rh");  
	state.name.push_back("rkp"); 
	state.name.push_back("rk"); 
	state.name.push_back("rap");  
	state.name.push_back("ra");  
	state.position.push_back(0);		//ly
	state.position.push_back(-msg.position[2]*r1);		//lh
	state.position.push_back(msg.position[0]*r0);   
	state.position.push_back(msg.position[3]*r2);		//lk
	state.position.push_back(msg.position[1]*r0);		
	state.position.push_back(msg.position[4]*r2);		//la
	state.position.push_back(0);		//ry
	state.position.push_back(msg.position[7]*r1);		//rh
	state.position.push_back(msg.position[5]*r0);
	state.position.push_back(-msg.position[8]*r2);		//rk
	state.position.push_back(msg.position[6]*r0);
	state.position.push_back(-msg.position[9]*r2);		//ra

    state.velocity.push_back(0);
    state.velocity.push_back(msg.velocity[2]*r1);
	state.velocity.push_back(msg.velocity[0]*r0);
	state.velocity.push_back(msg.velocity[3]*r2);
	state.velocity.push_back(msg.velocity[1]*r0);
	state.velocity.push_back(msg.velocity[4]*r2);
	state.velocity.push_back(0);
    state.velocity.push_back(msg.velocity[7]*r1);
	state.velocity.push_back(msg.velocity[5]*r0);
	state.velocity.push_back(msg.velocity[8]*r2);
	state.velocity.push_back(msg.velocity[6]*r0);
	state.velocity.push_back(msg.velocity[9]*r2);
	
    
    state.effort.push_back(0);
    state.effort.push_back(msg.effort[2]);
	state.effort.push_back(msg.effort[0]);
	state.effort.push_back(msg.effort[3]);
	state.effort.push_back(msg.effort[1]);
	state.effort.push_back(msg.effort[4]);
	state.effort.push_back(0);
    state.effort.push_back(msg.effort[7]);
	state.effort.push_back(msg.effort[5]);
	state.effort.push_back(msg.effort[8]);
	state.effort.push_back(msg.effort[6]);
	state.effort.push_back(msg.effort[9]);
    js_pub.publish(state);

	//根据雅可比矩阵估算接触力
	double tlh= msg.position[2]*r1*Pi/180.0;
	double trh= -msg.position[7]*r1*Pi/180.0;
	double tlk= -msg.position[3]*r2*Pi/180.0;
	double trk= msg.position[8]*r2*Pi/180.0;
	Jl(0,0) = L1*cos(tlh)+L2*cos(tlk-tlh);
	Jl(0,1) = -L2*cos(tlk-tlh);
	Jl(1,0) = L1*sin(tlh)-L2*sin(tlk-tlh);
	Jl(1,1) = L2*sin(tlk-tlh);
	Jr(0,0) = L1*cos(trh)+L2*cos(trk-trh);
	Jr(0,1) = -L2*cos(trk-trh);
	Jr(1,0) = L1*sin(trh)-L2*sin(trk-trh);
	Jr(1,1) = L2*sin(trk-trh);
	Tl(0)=  -1.91*msg.effort[2];
	Tl(1)=  3.95*msg.effort[3];
	Tr(0)=  1.91*msg.effort[7];
	Tr(1)=  -3.95*msg.effort[8];
	// cout<<"Jl:"<<Jl<<endl;
	// cout<<"Jl.transpose().inverse():"<<Jl.transpose().inverse()<<endl;
	// cout<<"Tl"<<Tl.transpose()<<endl;
	Fl = Jl.transpose().inverse() * Tl;
	Fr = Jr.transpose().inverse() * Tr;
	// for(int i = 0; i<msg.name.size(); i++){
	// 	cout<<i<<", motor "<<msg.name[i]<<" ,pos: "<<msg.position[i]<<" ,cur: "<<msg.effort[i]<<endl;
	// }
	// cout<<"Fl : "<<Fl.transpose()<<" , Fr:"<<Fr.transpose()<<endl;
	geometry_msgs::WrenchStamped wrench_msg;
	wrench_msg.header.frame_id ="world";
	wrench_msg.wrench.force.y = Fl(0);
	wrench_msg.wrench.force.z = Fl(1);
	wrench_msg.header.stamp =ros::Time::now();
	LF_pub.publish(wrench_msg);
	wrench_msg.header.frame_id ="world";
	wrench_msg.wrench.force.y = Fr(0);
	wrench_msg.wrench.force.z = Fr(1);
	wrench_msg.header.stamp =ros::Time::now();
	RF_pub.publish(wrench_msg);

}	

//原地踏步测试
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drive");
	double r0,r1,r2;
	double st,dst,sl,sa,hi,L1,L2,dt,sh,il,L3,Lhe, Lto,alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp,T_ini;
	int pause_time,StepNum;
	bool echo,ini,sta,HTwalk;
	ros::NodeHandle n_p("~");      
	n_p.param<bool>("echo", echo, true);
	n_p.param<bool>("ini", ini, true);
    n_p.param<double>("step_period", st, 6.0);
	n_p.param<double>("double_support_time", dst, 1.0);
	n_p.param<double>("step_len", sl, 0.4);
	n_p.param<double>("sl_dsp", sl_dsp, 0.1);
	n_p.param<double>("step_sa", sa, 0.12);
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
	n_p.param<double>("toe_tp", toe_tp, 0.5);
	n_p.param<double>("heel_tp", heel_tp, 0.5);
	n_p.param<double>("dt", dt, 0.05);
	n_p.param<double>("r0", r0, 10);
	n_p.param<double>("r1", r1, 10);
	n_p.param<double>("r2", r2, 10);
	n_p.param<bool>("static", sta, false);
	n_p.param<bool>("Heel_Toe_walk", HTwalk, false);
	n_p.param<double>("T_ini", T_ini, 5.0);
	Drive drive;
	drive.setLegParam(L1,L2);
	drive.setRate(r0,r1,r2);
	Leg left_leg(st,dst, sl,sa,hi, il, sh, L1,L2, dt );
	Leg right_leg(st,dst, sl,sa, hi, il, sh, L1,L2, dt );
	left_leg.set_ini_phase(0.0);
	right_leg.set_ini_phase(1.0);
	Eigen::VectorXd qn,traj_now,ini_motor_pos;
	qn=Eigen::VectorXd::Zero(10);
	ini_motor_pos=Eigen::VectorXd::Zero(10);
	ros::Rate rate(1/dt);
	double ti=0;
	//初始化，从当前位姿到目标位姿插值
	sleep(2);
	while(!drive.JS_receive){
		rate.sleep();
		ros::spinOnce();
	}
	ini_motor_pos<<drive.js_now.position[0],drive.js_now.position[1],drive.js_now.position[2],drive.js_now.position[3],drive.js_now.position[4],
	drive.js_now.position[5],drive.js_now.position[6],drive.js_now.position[7],drive.js_now.position[8],drive.js_now.position[9];
	cout<<"ini motor:"<<ini_motor_pos.transpose()<<endl;
	while(ti<T_ini){
		double p = ti/T_ini;
		if(ini){
			qn.head(5)=left_leg.iniPose();
			qn.tail(5)=right_leg.iniPose();
			drive.iniPosition(qn,ini_motor_pos,p);
		}else{
			drive.iniPosition(qn,ini_motor_pos,p);
		}
	//	cout<<"qn: "<<qn[0]<<","<<qn[1]<<","<<qn[2]<<","<<qn[3]<<","<<qn[4]<<","<<qn[5]<<","<<qn[6]<<","<<qn[6]<<","<<qn[7]<<endl;
		ti += dt;
		rate.sleep();
		ros::spinOnce();
	}
	Key key;
	while(true){
		rate.sleep();
		ros::spinOnce();
		if(key.readKey())
			break;
	}
	if(HTwalk){
		left_leg.setHTParams( L3,  Lhe,  Lto,alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp);
		right_leg.setHTParams( L3,  Lhe,  Lto , alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp);
	}
	while(left_leg.getStepNum()<StepNum){
		if(sta){
			qn.head(5)=left_leg.StaticStep();
			qn.tail(5)=right_leg.StaticStep();
		}else{
			qn.head(5)=left_leg.Step();
			qn.tail(5)=right_leg.Step();
		}
		if(echo)
			cout<<"origin qn: "<<qn[0]<<","<<qn[1]<<","<<qn[2]<<","<<qn[3]<<","<<qn[4]<<","<<qn[5]<<","<<qn[6]<<","<<qn[7]<<","<<qn[8]<<","<<qn[9]<<endl;
		else
			drive.setPosition(qn);
		rate.sleep();
		ros::spinOnce();
	}	
	ros::spin();
	return 0;
}


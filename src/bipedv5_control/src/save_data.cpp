#include "bipedv5_control/drive_bezier.h"
#include <fstream>
using namespace std;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "data");
	//数据写入
	std::string file_path = "/home/ye/bipedv3/src/biped_control/data/joint_data.txt";
    std::ofstream ofs(file_path);
    ofs << "t   q0   q1   q2" << std::endl;
	std::string file_path2 = "/home/ye/bipedv3/src/biped_control/data/traj_data.txt";
    std::ofstream ofs2(file_path2);
    ofs2 << "t   q0   q1   q2" << std::endl;
	double st,sl,sa,hi,L0,L1,L2,Lk,dt,sh,it;
	int idl,idr,pause_time;
	bool fw;
	bool echo,ini,hd;
	double r1,r2,r3;
	ros::NodeHandle n_p("~");      
    n_p.param<double>("step_period", st, 6.0);
	n_p.param<double>("ini_time", it, 5.0);
	n_p.param<double>("step_len", sl, 0.4);
	n_p.param<double>("step_sa", sa, 0.12);
	n_p.param<double>("ini_height", hi, -0.75);
	n_p.param<double>("step_height", sh, 0.1);
	n_p.param<double>("L0", L0, 0.1);
	n_p.param<double>("L1", L1, 0.3);
	n_p.param<double>("L2", L2, 0.3);
	n_p.param<double>("Lk", Lk, 0.1);
	n_p.param<int>("idl", idl, 0);
	n_p.param<int>("idr", idr, 8);
	n_p.param<int>("pause_time", pause_time, 10000);
	n_p.param<double>("dt", dt, 0.05);
	n_p.param<bool>("forward", fw, true);
	n_p.param<bool>("echo", echo, true);
	n_p.param<bool>("ini", ini, true);
	n_p.param<bool>("q_hold", hd, true);
	n_p.param<double>("r1", r1, 10);
	n_p.param<double>("r2", r2, 10);
	n_p.param<double>("r3", r3, 10);
	Walk left_leg(st,it,sl,sa,hi,sh,idl,L0,L1,L2,Lk,dt,fw,hd);	//参数：周期，步幅参数,初始轨迹序号
	left_leg.updateTrajIK();
	Walk right_leg(st,it,sl,sa,hi,sh,idr,L0,L1,L2,Lk,dt,fw,hd);	//参数：周期，步幅参数
	right_leg.updateTrajIK();
	cout<<"ini ok!"<<endl;
	Eigen::VectorXd qn(3);
	qn=Eigen::VectorXd::Zero(3);
	double t=0;
	int i=0;
	left_leg.setCubSpl(false);
	while(left_leg.getStepNum()<2){
		qn=left_leg.Step();
		cout<<"qn: "<<qn.transpose()<<endl;
		ofs<<t<<"   " <<qn[0]<<"  "<<qn[1]<<"  "<<qn[2]<<endl;
		if(i%75==0)
			ofs2<<t<<"   " <<qn[0]<<"  "<<qn[1]<<"  "<<qn[2]<<endl;
		i++;
		t+=0.005;	

	}
	return 0;
}


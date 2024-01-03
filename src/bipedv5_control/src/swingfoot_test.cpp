#include "bipedv5_control/swingfoot_trajectory.h"
#include <fstream>
using namespace std;



int main(int argc, char **argv)
{
	//数据写入
	std::string file_path = "./src/biped_control/data/swingfoot_data.txt";
    std::ofstream ofs(file_path);
    ofs << "t   x   y   z   vx   vy   vz   ax   ay   az" << std::endl;
	FootSwingTrajectory<double> traj;
	Vec3<double> p0(0,0.1,0);
	traj.setInitialPosition(p0);
	Vec3<double> pf(0.4,0.1,0);
	traj.setFinalPosition(pf);
	traj.setHeight(0.25);
	double t=0;
	double Ts=0.7;
	while(t<=(Ts+0.001)){
		if(t>Ts)
			t=Ts;
		traj.computeSwingTrajectoryBezier(t/Ts, Ts);
		auto p=traj.getPositon();
		auto v=traj.getVelocity();
		auto a=traj.getAcceleration();
		ofs<<t<<"   " <<p[0]<<"  "<<p[1]<<"  "<<p[2]<<"   " <<v[0]<<"  "<<v[1]<<"  "<<v[2]<<"   " <<a[0]<<"  "<<a[1]<<"  "<<a[2]<<endl;
		t+=0.005;	
	}
	return 0;
}


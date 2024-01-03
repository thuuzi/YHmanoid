#include "bipedv5_control/leg.h"
#include <unistd.h>

using namespace std;
using namespace Eigen;

Leg::Leg( double st,double dst, double sl,double sa, double hi,double il, double sh, double L1, double L2, double dt ): step_period_(st),
step_len_(sl),ini_heigh_(hi), ini_len_(il),step_heigh_(sh),ft_(sl,sa,sh,hi,il),ik_(L1,L2),
dt_(dt)
{
	step_num_=0;
	time_now_=0;
	dsp_ = dst/st;
	ft_.set_dsp(dsp_);
	traj_now_=Eigen::VectorXd::Zero(2);
	q_now_=Eigen::VectorXd::Zero(5);
	HTwalk=false;

}



const Eigen::VectorXd& Leg::iniPose(){
	traj_now_ = ft_.getTraj(ini_phase_);
	ik_.solve(traj_now_);
	//ik_.rad2deg();
	q_now_.tail(3) = ik_.getIK();
	return q_now_;
}


const Eigen::VectorXd Leg::Step(){
	if(time_now_>= step_period_*(step_num_+1))
		step_num_ ++;
	phase_ = 2*(time_now_- step_period_*step_num_)/step_period_ + ini_phase_;
	if(phase_>2)
		phase_ -= 2.0;
	if(HTwalk){
		Vector3d traj_res;
		traj_res = ft_.getTrajHT(phase_);	//最后一位是脚踝角度
		traj_now_=traj_res.head(2);		//y,z坐标
		ik_.solve(traj_now_,traj_res(2));
	}else{
		traj_now_=ft_.getTraj(phase_);
		ik_.solve(traj_now_);
	}
	//ik_.rad2deg();
	q_now_(0)=ft_.getSa(phase_)/2.0;
	q_now_(1)=ft_.getSa(phase_)/2.0;
	q_now_.tail(3) = ik_.getIK();
	time_now_ += dt_;		
	return q_now_;
};

const Eigen::VectorXd Leg::StaticStep(){
	if(time_now_>= step_period_*(step_num_+1))
		step_num_ ++;
	phase_ = 2*(time_now_- step_period_*step_num_)/step_period_ + ini_phase_;
	if(phase_>2)
		phase_ -= 2.0;
	traj_now_ = ft_.getTraj(phase_);
	ik_.solve(traj_now_);
//	ik_.rad2deg();
	q_now_(0)=ft_.getStaticSa(phase_)/2.0;
	q_now_(1)=ft_.getStaticSa(phase_)/2.0;
	q_now_.tail(3) = ik_.getIK();
	time_now_ += dt_;		
	return q_now_;
};

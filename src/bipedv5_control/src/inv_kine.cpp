#include "bipedv5_control/inv_kine.h"
using namespace std;


InvKine::InvKine(const double L1, const double L2):
len_thigh(L1),	
len_shank(L2){
	q_=Eigen::VectorXd::Zero(3);
}
//求前摆三个关节,默认脚掌平行地面
void InvKine::solve(const Eigen::VectorXd& p){
	double m1=(len_thigh*len_thigh-len_shank*len_shank+p[0]*p[0]+p[1]*p[1])/(2*len_thigh);
	if(m1/sqrt(p[0]*p[0]+p[1]*p[1])>=1)
		q_[0]=0;
	else
		q_[0]=Pi-asin(m1/sqrt(p[0]*p[0]+p[1]*p[1]))-atan2(-p[1],p[0]);
	q_[1]=asin((len_thigh*sin(q_[0])-p[0])/len_shank)+q_[0];
	q_[2]=q_[1]-q_[0];
}

//求前摆三个关节,给定脚掌角度
void InvKine::solve(const Eigen::VectorXd& p,double qa){
	double m1=(len_thigh*len_thigh-len_shank*len_shank+p[0]*p[0]+p[1]*p[1])/(2*len_thigh);
	if(m1/sqrt(p[0]*p[0]+p[1]*p[1])>=1)
		q_[0]=0;
	else
		q_[0]=Pi-asin(m1/sqrt(p[0]*p[0]+p[1]*p[1]))-atan2(-p[1],p[0]);
	q_[1]=asin((len_thigh*sin(q_[0])-p[0])/len_shank)+q_[0];
	q_[2]=q_[1]-q_[0]+qa;
}

void InvKine::rad2deg(){
	q_=q_*180/Pi;
}

void InvKine::deg2rad(){
	q_=q_*Pi/180;
}


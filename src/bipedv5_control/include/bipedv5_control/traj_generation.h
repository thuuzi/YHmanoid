
#include "Eigen/Core"
#include <iostream>
#include <math.h>

class TrajGeneration{
public:
    
    TrajGeneration(const double sl,const double sa,const double sh,const double hi):sl_(sl),sa_(sa),hi_(hi),sh_(sh){
	//原地踏步
	step_traj_num_=40;
	step_traj_=Eigen::MatrixXd::Zero(step_traj_num_,3);
	step_traj_<<sl,0,hi,	//支撑相								
		sl,-0.15*sa,hi,
		sl,-0.2*sa,hi,
		sl,-0.85*sa,hi,
		sl,-sa,hi,		
		0.9*sl,-sa,hi,
		0.8*sl,-sa,hi,
		0.7*sl,-sa,hi,
		0.6*sl,-sa,hi,
		0.5*sl,-sa,hi,
		0.4*sl,-sa,hi,
		0.3*sl,-sa,hi,
		0.2*sl,-sa,hi,
		0.1*sl,-sa,hi,
		0,-sa,hi,
		-0.1*sl,-sa,hi,
		-0.2*sl,-sa,hi,
		-0.3*sl,-0.85*sa,hi,
		-0.4*sl,-0.2*sa,hi,
		-0.5*sl,-0.15*sa,hi,
		-0.6*sl,0,hi,		//摆动相
		-0.6*sl,0.15*sa,hi,		
		-0.6*sl,0.2*sa,hi,		
		-0.6*sl,0.75*sa,hi,		
		-0.5*sl,0.8*sa,hi+sh*0.15,
		-0.4*sl,0.8*sa,hi+sh*0.2,
		-0.3*sl,0.8*sa,hi+sh*0.25,
		-0.2*sl,0.8*sa,hi+sh*0.5,
		-0.1*sl,0.8*sa,hi+sh*0.85,
		0,0.8*sa,hi+sh*0.9,
		0.1*sl,0.8*sa,hi+sh,
		0.2*sl,0.8*sa,hi+sh*0.9,
		0.3*sl,0.8*sa,hi+sh*0.85,
		0.6*sl,0.8*sa,hi+sh*0.5,
		0.9*sl,0.8*sa,hi+sh*0.25,
		sl,0.8*sa,hi+sh*0.2,
		sl,0.8*sa,hi+sh*0.15,
		sl,0.7*sa,hi,
		sl,0.15*sa,hi,
		sl,0.1*sa,hi;
	//直行
	traj_num_=16;
	traj_=Eigen::MatrixXd::Zero(traj_num_,3);
	traj_<<0,sa,hi,			//左腿支撑期
		0,sa,hi,
		0,sa,hi,
		0,sa,hi,
		0,sa,hi,
		-sl/8,sa/2,hi,		//双腿支撑期
		-sl/4,0,hi,
		-3*sl/8,-sa/2,hi,
		-sl/2,-sa,hi,		//右腿支撑
		-sl/4,-sa,hi+sl/6,	//sqrt(sl*sl/4-sl*sl/16),
		0,-sa,hi+sl/3,
		sl/4,-sa,hi+sl/6,	//sqrt(sl*sl/4-sl*sl/16),
		sl/2,-sa,hi,		//双腿支撑
		3*sl/8,-sa/2,hi,
		sl/4,0,hi,
		sl/8,sa/2,hi;
	//初始轨迹
	ini_traj_num_=4;
	ini_traj_=Eigen::MatrixXd::Zero(ini_traj_num_,3);
	ini_traj_<<0,0,hi+0.1,			
		0,0,hi+0.05,
		0,0,hi+0.1,
		0,0,hi;
}

    
    const Eigen::MatrixXd& getTraj() const {
        return traj_;
    }

    const Eigen::MatrixXd& getStepTraj() const {
        return step_traj_;
    }

    const Eigen::MatrixXd& getIniTraj() const {
        return ini_traj_;
    }

    const int getStepTrajNum() const {
        return  step_traj_num_;
    };

    const int getTrajNum() const {
        return  traj_num_;
    };

    const int getIniTrajNum() const {
        return  ini_traj_num_;
    };

private:
    double sl_, sa_, hi_,sh_;        
    int traj_num_,step_traj_num_;
    int ini_traj_num_;
    Eigen::MatrixXd traj_,step_traj_;
    Eigen::MatrixXd ini_traj_;
};
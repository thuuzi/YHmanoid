
#include "Eigen/Core"
#include <iostream>
#include <math.h>
#include "foot_traj.h"
#include "inv_kine.h"
#include <fstream>
using namespace std;


class Leg{
public:
    Leg( double st,double dst,  double sl, double sa, double hi,double il, double sh, double L1, double L2, double dt );

    const Eigen::VectorXd& iniPose();

    const Eigen::VectorXd Step();

    const Eigen::VectorXd StaticStep();

    void set_ini_phase(double s){
        ini_phase_= s;
    }
    const int getStepNum() const {
        return  step_num_;
    };
    
    const Eigen::VectorXd getTrajNow() const {
        return traj_now_;
    };

    void setHTParams(double L3, double Lhe, double Lto,double alpha_tot,double alpha_het,double toe_tp, double heel_tp,double sl_dsp){
        ft_.setHTParams(L3,Lhe,Lto,alpha_tot,alpha_het,toe_tp,heel_tp,sl_dsp);
        HTwalk =true;
    };

private:
    double time_now_;
    double step_period_;            //周期（两步）
    double dsp_; 
    double phase_,ini_phase_;      //当前所处相位  （0～2）
    double step_len_,ini_heigh_,step_heigh_, ini_len_; //步长，侧摆，初始高度 ,初始x位置
    double dt_;                  //电机控制时长
    FootTraj ft_;
    InvKine ik_;
    Eigen::VectorXd q_now_,traj_now_;     
    int step_num_;
    bool HTwalk;
};
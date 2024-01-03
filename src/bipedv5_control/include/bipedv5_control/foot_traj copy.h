
#include "Eigen/Core"
#include <iostream>
#include <math.h>
#include "Interpolation.h"
using namespace std;
class FootTraj{
public:
    FootTraj( double sl, double sa, double sh, double hi, double il):sl_(sl),sa_(sa),hi_(hi),sh_(sh),il_(il){
		first_step_ = true;
		//初始轨迹
		ini_traj_num_=4;
		ini_traj_=Eigen::MatrixXd::Zero(ini_traj_num_,2);
		ini_traj_<<0,hi+0.1,			
			0,hi+0.05,
			0,hi+0.1,
			0,hi;
		
	}

	void setHTParams(double L3, double Lhe, double Lto ,double alpha_tot,double alpha_het, double totp,double hetp,double sl_dsp){
		L3_=L3;Lhe_=Lhe;Lto_=Lto;sl_dsp_=sl_dsp;
		Lah_ = sqrt(L3_*L3_+Lhe_*Lhe_);
		Lat_ = sqrt(L3_*L3_+Lto_*Lto_);
		alpha_ah_ = atan(L3_/Lhe_);
		alpha_at_= atan(L3_/Lto_);
		alpha_tot_ = alpha_tot;
		alpha_het_ = alpha_het;
		pysi_ = il_ - sl_/2 + Lto_ - Lat_* cos(alpha_at_+alpha_tot_) - sl_dsp_;
		pzsi_ = hi_ - L3_ + Lat_* sin(alpha_at_+alpha_tot_);
		pyse_ = il_ + sl_/2 - Lhe_  + Lah_* cos(alpha_ah_+alpha_het_);
		pzse_ = hi_ - L3_ + Lah_* sin(alpha_ah_+alpha_het_);
		top_=totp*dsp_;
		hep_=hetp*dsp_;
		
		// cout<<"pysi_:"<<pysi_<<", pzsi_: "<<pzsi_<<" , pyse: "<<pyse_<<" , pzse_:"<<pzse_<<endl;
    }

	void set_dsp(double dsp){
		dsp_ = dsp;
	}
    


	const Eigen::Vector2d  getTraj(double s)  {		//s为相位，(0~1摆动相，1～2支撑相)
		Eigen::Vector2d traj;
		double p;
		if(s>=0 && s< dsp_){		//双腿支撑期
			if(first_step_){
				p = s/dsp_ ;
				if(p<0)
					p=0;
				traj(0) = Interpolate::Cycloid<double>( il_, il_- sl_/2, p);			
			}else{
				traj(0) = il_- sl_/2;
			}
			traj(1) = hi_;
		}else if(s<(0.5+dsp_/2)){		//抬腿
			if(first_step_)
				first_step_ = false;
			p  =  (s-dsp_)/(0.5-dsp_/2) ;
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( il_-sl_/2, il_+ sl_/2, p/2);
			traj(1) = Interpolate::Cycloid<double>( hi_, hi_+sh_ , p);
		}else if(s < 1.0){		//落腿
			p = (s-0.5-dsp_/2)/(0.5-dsp_/2);
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( il_-sl_/2, il_+ sl_/2, p/2+0.5);
			traj(1) = Interpolate::Cycloid<double>( hi_+sh_, hi_ , p );
		}else if(s < (1.0+dsp_)){		//双腿支撑期
			if(first_step_){
				p = (s-1)/dsp_ ;
				if(p<0)
					p=0;
				traj(0) = Interpolate::Cycloid<double>( il_, il_+sl_/2, p);
			}else{
				traj(0) = il_+ sl_/2;
			}
			traj(1) = hi_;
		}else if(s < 2.0){		//单腿支撑
			if(first_step_)
				first_step_ = false;
			p = (s -1 - dsp_)/(1-dsp_) ;
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( il_+sl_/2, il_- sl_/2, p);
			traj(1) = hi_;
		}
        return traj;
    }

	const Eigen::Vector3d  getTrajHT(double s)  {		//自然步态,返回traj包括y、z坐标和脚掌角度
		Eigen::Vector3d traj;
		double p;
		double alpha_to,alpha_he;
		if(s>=0 && s< (dsp_-top_)){		//双腿支撑期,双足都平行地面
			if(first_step_){
				p = s/(dsp_-top_);
				if(p<0)
					p=0;
				traj(0) = Interpolate::Cycloid<double>( il_, il_- sl_/2, p);			
			}else{
				traj(0) = il_- sl_/2;
			}
			traj(1) = hi_;
			traj(2) = 0;
		}else if(s < dsp_){		//双腿支撑期，toe-off
			if(first_step_)
				first_step_ = false;
			p  =  (s-dsp_+top_)/top_;
			if(p<0)
				p=0;
			alpha_to = Interpolate::Cycloid<double>( 0, alpha_tot_, p);
			traj(0) = il_ - sl_/2 + Lto_ - Lat_* cos(alpha_at_+alpha_to);
			traj(1) = hi_ - L3_ + Lat_* sin(alpha_at_+alpha_to);
			traj(2) = -alpha_to;
		}else if(s<(0.5+dsp_/2)){		//抬腿
			p  =  (s-dsp_)/(0.5-dsp_/2) ;
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( pysi_, pyse_, p/2);
			traj(1) = Interpolate::Cycloid<double>( pzsi_ , pzsi_+sh_ , p);
			traj(2) = Interpolate::Cycloid<double>( -alpha_tot_, alpha_het_, p/2);
		}else if(s < 1.0){		//落腿
			p = (s-0.5-dsp_/2)/(0.5-dsp_/2);
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( pysi_, pyse_,  p/2+0.5);
			traj(1) = Interpolate::Cycloid<double>( pzsi_+sh_, pzse_ , p );
			traj(2) = Interpolate::Cycloid<double>( -alpha_tot_, alpha_het_, p/2+0.5);
		}else if(s < (1.0+hep_)){		//双腿支撑期 heel-strike
			if(first_step_){
				p = (s-1)/hep_;
				if(p<0)
					p=0;
				traj(0) = Interpolate::Cycloid<double>( il_, il_+sl_/2, p);
				traj(1) = hi_;
				traj(2) = 0;
			}else{
				p = (s-1)/hep_;
				if(p<0)
					p=0;
				alpha_he = Interpolate::Cycloid<double>( alpha_het_,0, p);
				traj(0) = il_ + sl_/2 -Lhe_  + Lah_* cos(alpha_ah_+alpha_he);
				traj(1) = hi_ - L3_ + Lah_* sin(alpha_ah_+alpha_he);
				traj(2) = alpha_he;	
			}
		}else if(s < (1.0+dsp_)){		//双腿支撑期
			if(first_step_)
				first_step_=false;
			traj(0) = il_+ sl_/2;
			traj(1) = hi_;
			traj(2) = 0;
		}else if(s < 2){		//单腿支撑
			p = (s -1 - dsp_)/(1-dsp_) ;
			if(p<0)
				p=0;
			traj(0) = Interpolate::Cycloid<double>( il_+sl_/2 - sl_dsp_, il_- sl_/2, p);
			traj(1) = hi_;
			traj(2) = 0;
		}

		if(s<dsp_||((s>1)&&(s<1+dsp_))){
			if (s>1)
				p=(s-1)/dsp_;
			else
				p = s/dsp_;
			traj(0) = traj(0) - Interpolate::Cycloid<double>( 0, sl_dsp_, p);
		}
        return traj;
    }


	const double getSa(double s)  {		//s为相位，(0~1摆动相，1～2支撑相)
		double q,p;
		if(s>=0 && s< 0.5+dsp_/2){		
			if(first_step_){
				p = s/(0.5+dsp_/2) ;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( 0, -sa_ ,p);
			}else{
				p  = s + 0.5 - dsp_/2;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( sa_, -sa_ , p );
			}
		}else if(s<1.5+dsp_/2){		
			if(first_step_){
				p = (s-1.0)/(0.5+dsp_/2) ;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( 0, sa_ ,p);
			}else{
				p  = s - 0.5 - dsp_/2;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( -sa_, sa_ , p );
			}
		}else if(s < 2){
			p  = s - 1.5- dsp_/2;
			if(p<0)
				p=0;
			q = Interpolate::Cycloid<double>( sa_, -sa_ , p );
		}
        return q;
    }

	const double getStaticSa(double s)  {		//s为相位，(0~1摆动相，1～2支撑相) 准静态
		double q,p;
		if(s>=0 && s< dsp_){		//双腿支撑期
			if(first_step_){
				p = s/dsp_;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( 0, -sa_ ,p);
			}else{
				p = s/dsp_;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( sa_, -sa_ , p );
			}
		}else if(s < (1)){			//抬腿期
			q = -sa_;
		}else if(s < (1+dsp_)){		//双腿支撑期
			if(first_step_){
				p = (s-1)/dsp_;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( 0, sa_ ,p);
			}else{
				p = (s-1)/dsp_;
				if(p<0)
					p=0;
				q = Interpolate::Cycloid<double>( -sa_, sa_ , p );
			}
		}else if(s < 2){
			q = sa_;
		}
        return q;
    }

	



    const Eigen::MatrixXd& getIniTraj() const {
        return ini_traj_;
    }

    const int getIniTrajNum() const {
        return  ini_traj_num_;
    };

private:
	bool first_step_;
    double sl_,sa_, hi_,sh_,il_,sl_dsp_;   
	double Lah_,alpha_ah_,Lat_,alpha_at_,L3_,Lhe_,Lto_; 
	double alpha_tot_ ,alpha_het_ ; 
	double dsp_,top_,hep_;	   //双腿支撑期比例,toe-off比例，heel-strike比例
	double pysi_,pyse_,pzsi_,pzse_;
    int ini_traj_num_;
    Eigen::MatrixXd ini_traj_;
};
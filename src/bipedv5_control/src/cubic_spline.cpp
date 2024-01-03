#include "bipedv5_control/cubic_spline.h"

using namespace std;

CubicSpline::CubicSpline(int n):n_(n){
	x_=VectorXd::Zero(n_+1);y_=VectorXd::Zero(n_+1);m_=VectorXd::Zero(n_+1);
	h_=VectorXd::Zero(n_);mu_=VectorXd::Zero(n_);lambda_=VectorXd::Zero(n_);
	d_=VectorXd::Zero(n_);A_=MatrixXd::Zero(n_,n_);
}

void CubicSpline::update_matrix(const Eigen::VectorXd& x,const Eigen::VectorXd& y){
	x_=x;y_=y;	
	for(int i=0;i<n_;i++){
		h_(i)=x(i+1)-x(i);
		if(i>0){
			mu_(i-1)=h_(i)/(h_(i)+h_(i-1));
			lambda_(i-1)=1-mu_(i-1);
			d_(i-1)=3*(mu_(i-1)*((y_(i)-y_(i-1))/(x_(i)-x_(i-1)))+lambda_(i-1)*((y_(i+1)-y_(i))/(x_(i+1)-x_(i))));

			A_(i-1,i-1)=2;
			A_(i-1,i)=lambda_(i-1);
			if(i>1)
				A_(i-1,i-2)=mu_(i-1);
		}
	}
	//周期边界条件
	lambda_(n_-1)=h_(n_-1)/(h_(0)+h_(n_-1));
	mu_(n_-1)=1-lambda_(n_-1);
	d_(n_-1)=3*(mu_(n_-1)*((y_(n_)-y_(n_-1))/(x_(n_)-x_(n_-1)))+lambda_(n_-1)*((y_(1)-y_(0))/(x_(1)-x_(0))));
	A_(0,n_-1)=mu_(0);A_(n_-1,0)=lambda_(n_-1);A_(n_-1,n_-1)=2;A_(n_-1,n_-2)=mu_(n_-1);
	m_.tail(n_) =A_.colPivHouseholderQr().solve(d_);
	m_(0)=m_(n_);
	// cout<<"m_"<<m_<<endl;
	// cout<<"A_"<<A_<<endl;
	// cout<<"A*D"<<A_*m_.tail(n_)<<endl;
}

double CubicSpline::interpolate(int idx,double xi){		//idx输入0～15
	double a1,a2,b1,b2;
	a1=(1+2*(xi-x_(idx))/(x_(idx+1)-x_(idx)))*pow((xi-x_(idx+1))/(x_(idx)-x_(idx+1)),2);
	a2=(1+2*(xi-x_(idx+1))/(x_(idx)-x_(idx+1)))*pow((xi-x_(idx))/(x_(idx+1)-x_(idx)),2);
	b1=(xi-x_(idx))*pow((xi-x_(idx+1))/(x_(idx)-x_(idx+1)),2);
	b2=(xi-x_(idx+1))*pow((xi-x_(idx))/(x_(idx)-x_(idx+1)),2);
	double yi=y_(idx)*a1+y_(idx+1)*a2+m_(idx)*b1+m_(idx+1)*b2;
	//cout<<"idx:"<<idx<<",xi:"<<xi<<",a1:"<<a1<<",a2:"<<a2<<",b1:"<<b1<<",b2:"<<b2<<",mi:"<<m_(idx)<<endl;
	return yi;
}

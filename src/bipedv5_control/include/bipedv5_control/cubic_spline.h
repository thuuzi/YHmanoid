#include<iostream>
#include<vector>
#include "Eigen/Dense"
#include<cmath>
#include <fstream>
using namespace Eigen;


class CubicSpline{
public:
    CubicSpline(int n);
    void update_matrix(const Eigen::VectorXd& x,const Eigen::VectorXd& y);
    double interpolate(int idx,double x);


private:
   int n_;      //插值区间数量
   MatrixXd  A_;
   VectorXd  x_,y_;      //插值点
   VectorXd  m_;         //插值点处导数
   VectorXd  d_;
   VectorXd  h_,mu_,lambda_;
};

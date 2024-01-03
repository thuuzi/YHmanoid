#include "Eigen/Core"
#include <iostream>
#include <math.h>

#define Pi 3.14159265359

class InvKine{
    
public:
    InvKine(const double L1, const double L2);
    
    void solve(const  Eigen::VectorXd& p);

    void solve(const  Eigen::VectorXd& p,double qa);

    void rad2deg();

    void deg2rad();

    const Eigen::VectorXd& getIK() const {
	    return q_;
    } ;
    

private:
    double len_thigh,len_shank;
    Eigen::VectorXd q_;
};
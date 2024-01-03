/* 
 * Copyright 2017-2021 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *		 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @brief Nonlinear CoM Estimation based on encoder, force/torque or pressure, and IMU measurements
 * @author Stylianos Piperakis
 * @details Estimates the 3D CoM Position and Velocity
 */
#ifndef __CoMLKF_H__
#define __CoMLKF_H__
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class CoMLKF {

private:
	/// Linearized State Transition, Error-Covariance, Identity, State Uncertainity, Discrete Linearized State Transition matrices
	Matrix<double, 9, 9> F, Fd, P, I, Q;
	/// 3D Center of Pressure, 3D Ground Reaction Force, Linearized State-Input matrices
	Vector3d  fC,g;
	/// Linearized Measurement Model
	Matrix<double, 3, 9> Hc,Hac;
	/// Kalman Gain
	Matrix<double, 9, 3> K;
	/// Measurement Noise and Update Covariance
	Matrix<double, 3, 3> Rc,Rac,S;
	/// Innovation vector
	Matrix<double, 3, 1> z;
	/// temp variable
	double tmp;
	/// Update state estimates
	void updateVars();
	/// Compute the nonlinear dynamics
	Matrix<double,9,1> computeDyn(Matrix<double,9,1> x_,Vector3d fC_);


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//状态变量：[ c dc f] 9维
	Matrix<double, 9, 1> x;
	//噪声方差
	double com_q, comd_q, fd_q, com_r, comdd_r;
	double dt, m;
	/// flag to indicate initialization
	bool firstrun;
	void init();
	void setdt(double dtt) {
		dt = dtt;
	}
	void setParams(double m_,double g_)
	{
		m = m_;
		g(2) = g_;	//(0,0,9.8)
		F.block<3,3>(3,6) = 1.0/m * Matrix<double,3,3>::Identity();
		Fd = I + F * dt ;
		Hac.block<3,3>(0,6) = 1.0/m * Matrix<double,3,3>::Identity();
	}
	//初始化CoM位置
	void setCoMPos(Vector3d pos) {
		x(0) = pos(0);
		x(1) = pos(1);
		x(2) = pos(2);
	}
	//初始化所受外力
	void setCoMExternalForce(Vector3d force) {
		x(6) = force(0);
		x(7) = force(1);
		x(8) = force(2);
	}
	//预测步骤
	void predict(Vector3d fC_);
	//更新步骤
	void updateWithEnc(Vector3d Pos);
	void updateWithImu(Vector3d acc,Vector3d gyro, Vector3d gyrodot,  Vector3d pwr);
	double comX, comY, comZ, velX, velY, velZ, fX,fY, fZ;

};
#endif
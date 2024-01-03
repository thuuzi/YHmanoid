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
 
#include <state_estimate/CoMLKF.h>

void CoMLKF::init() {
	//设置状态转移矩阵
	F  = Matrix<double,9,9>::Zero();
	Fd  = Matrix<double,9,9>::Zero();
	F.block<3,3>(0,3) = Matrix<double,3,3>::Identity();
	//F.block<3,3>(3,6) = 1.0/m * Matrix<double,3,3>::Identity();在setParam函数中设置
	I = Matrix<double,9,9>::Identity();
	//预测过程噪声矩阵
	Q = Matrix<double,9,9>::Zero();
	Q(0, 0) = com_q * com_q;
	Q(1, 1) = Q(0,0);
	Q(2, 2) = Q(0,0);

	Q(3, 3) = comd_q * comd_q;
	Q(4, 4) = Q(3, 3);
	Q(5, 5) = Q(3, 3);

	Q(6, 6) = fd_q * fd_q;
	Q(7, 7) = Q(6, 6);
	Q(8, 8) = Q(6, 6);
	fC = Vector3d::Zero();
	g = Vector3d::Zero();
	//状态变量
	x = Matrix<double,9,1>::Zero();
	z = Matrix<double,3,1>::Zero();
	//观测噪声
	Rc = Matrix<double,3,3>::Zero();
	Rc(0, 0) = com_r;
	Rc(1, 1) = com_r;
	Rc(2, 2) = com_r;
	Rac = Matrix<double,3,3>::Zero();
	Rac(0, 0) = comdd_r;
	Rac(1, 1) = comdd_r;
	Rac(2, 2) = comdd_r;
	S = Matrix<double,3,3>::Zero();
	//质心位置观测矩阵
	Hc = Matrix<double,3,9>::Zero();
	Hc.block<3,3>(0,0) = Matrix3d::Identity();
	//质心加速度观测矩阵
	Hac = Matrix<double,3,9>::Zero();
	//卡尔曼增益
	K=Matrix<double,9,3>::Zero();
	//协方差矩阵
	P = Matrix<double,9,9>::Zero();
	P.block<3,3>(0,0)= 1e-6 * Matrix<double,3,3>::Identity();
	P.block<3,3>(3,3)= 1e-2 * Matrix<double,3,3>::Identity();
	P.block<3,3>(6,6)= 1e-1* Matrix<double,3,3>::Identity();
	//状态变量
	comX = 0.000;
	comY = 0.000;
	comZ = 0.000;

	velX = 0.000;
	velY = 0.000;
	velZ = 0.000;

	fX = 0.000;
	fY = 0.000;
	fZ = 0.000;
	
	firstrun = true;
	
	cout << "Linear CoM Estimator Initialized!" << endl;
}

//状态变量9维，x(0~2)质心位置，x(3~5)质心速度，x(6~8)3d外力
Matrix<double,9,1> CoMLKF::computeDyn(Matrix<double,9,1> x_, Vector3d fC_)
{
    Matrix<double,9,1> res = Matrix<double,9,1>::Zero();
	res.segment<3>(0) = x_.segment<3>(3);		//质心位置求导为质心速度 dc=cdot
    res.segment<3>(3)  = 1/m * fC_ - g +1/m *x_.segment<3>(6);
    return res;
}



void CoMLKF::predict(Vector3d fC_){

	fC = fC_;
	//忽略小力，减小飘移
    for(int i=0;i<2;i++){
        if(abs(fC(i))< 0.02)
            fC(i)=0;
    }
	//euler预测
	x.noalias() += computeDyn(x, fC)*dt;
	
	P = Fd * P * Fd.transpose();
	P.noalias() += Q*dt;
	updateVars();
}

//根据关节数据更新
void CoMLKF::updateWithEnc(Vector3d Pos)
{
	z.noalias() = Pos - x.segment<3>(0);
	S.noalias() = Rc + Hc * P * Hc.transpose();
	K.noalias() = P * Hc.transpose() * S.inverse();
	x += K * z;
	P.noalias() = (I - K * Hc) * P * (I - K * Hc).transpose() + K * Rc * K.transpose();
	updateVars();
}

//根据IMU加速度更新
void CoMLKF::updateWithImu(Vector3d acc,Vector3d gyro, Vector3d gyrodot,  Vector3d pwr){

	//计算质心加速度 ac=ab+wx(wxp)+dwxp
	acc += gyro.cross(gyro.cross(pwr)) + gyrodot.cross(pwr);  
	//更新
	z.noalias() = acc - (1/m * fC - g +1/m *x.segment<3>(6));
	S.noalias() =Rac + Hac * P * Hac.transpose();
	K.noalias() = P * Hac.transpose() * S.inverse();
	x += K * z;
	P.noalias() = (I - K * Hac) * P * (I - K * Hac).transpose() + K * Rac * K.transpose();
	updateVars();
}

void CoMLKF::updateVars()
{
	
	comX = x(0);
	comY = x(1);
	comZ = x(2);

	velX = x(3);
	velY = x(4);
	velZ = x(5);

	fX = x(6);
	fY = x(7);
	fZ = x(8) + fC(2);

}


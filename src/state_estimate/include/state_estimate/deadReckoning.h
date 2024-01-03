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
 * @brief leg odometry for Bipeds based on force/torque or pressure, and encoder measurement
 * @author Stylianos Piperakis
 * @details Estimates the 3D leg odometry of the base and the corresponding relative leg measurements
 */

#include <eigen3/Eigen/Dense>
namespace serow
{

class deadReckoning
{
  private:
    double Tm, Tm2, ef, wl, wr, mass, g, freq, GRF, Tm3, tau0, tau1;
    Eigen::Matrix3d C1l, C2l, C1r, C2r;
    Eigen::Vector3d RpRm, LpLm, RpRmb, LpLmb;
    Eigen::Vector3d LLegContactPoint, RLegContactPoint;
    Eigen::Matrix3d LLegContactOrientation, RLegContactOrientation;
    Eigen::Vector3d pwr, pwl, pwb, pwb_;
    Eigen::Vector3d vwr, vwl, vwb, vwb_r, vwb_l;
    Eigen::Matrix3d Rwr, Rwl, vwb_cov;
    Eigen::Vector3d pwl_, pwr_;
    Eigen::Vector3d pb_l, pb_r;
    Eigen::Matrix3d Rwl_, Rwr_;
    Eigen::Vector3d vwbKCFS;
    Eigen::Vector3d Lomega, Romega;
    Eigen::Vector3d omegawl, omegawr;

    Eigen::Matrix3d AL, AR, wedgerf, wedgelf, RRpRm, RLpLm;
    Eigen::Vector3d bL, bR, plf, prf; //FT w.r.t Foot Frame;
    bool firstrun;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** @fn deadReckoning(Eigen::Vector3d pwl0, Eigen::Vector3d pwr0, Eigen::Matrix3d Rwl0, Eigen::Matrix3d Rwr0,
                  double mass_, double tau0_ = 1.0, double tau1_ = 0.01, double freq_ = 100.0, double g_ = 9.81,
                  Eigen::Vector3d plf_ = Eigen::Vector3d::Zero(), Eigen::Vector3d prf_ = Eigen::Vector3d::Zero())
    * @brief initializes the leg odometry module
    */
    deadReckoning(Eigen::Vector3d pwl0, Eigen::Vector3d pwr0, Eigen::Matrix3d Rwl0, Eigen::Matrix3d Rwr0,
                  double mass_, double tau0_ = 1.0, double tau1_ = 0.01, double freq_ = 100.0, double g_ = 9.81,
                  Eigen::Vector3d plf_ = Eigen::Vector3d::Zero(), Eigen::Vector3d prf_ = Eigen::Vector3d::Zero())
    {
        firstrun = true;
        vwb_cov = Eigen::Matrix3d::Zero();
        vwb_l = Eigen::Vector3d::Zero();
        vwb_r = Eigen::Vector3d::Zero();

        pwl_ = pwl0;
        pwr_ = pwr0;
        Rwl_ = Rwl0;
        Rwr_ = Rwr0;
        Rwl = Rwl_;
        Rwr = Rwr_;
        pwl = pwl_;
        pwr = pwr_;
        pwb_ = Eigen::Vector3d::Zero();
        pwb = pwb_;
        freq = freq_;
        mass = mass_;
        //Joint Freq
        Tm = 1.0 / freq;
        Tm2 = Tm * Tm;
        ef = 1e-8;
        g = g_;
        C1l = Eigen::Matrix3d::Zero();
        C2l = Eigen::Matrix3d::Zero();
        C1r = Eigen::Matrix3d::Zero();
        C2r = Eigen::Matrix3d::Zero();

        Lomega = Eigen::Vector3d::Zero();
        Romega = Eigen::Vector3d::Zero();
        vwb = Eigen::Vector3d::Zero();
        pb_l = Eigen::Vector3d::Zero();
        pb_r = Eigen::Vector3d::Zero();
        vwbKCFS = Eigen::Vector3d::Zero();
        plf = plf_;
        prf = prf_;
        RpRm = Eigen::Vector3d::Zero();
        LpLm = Eigen::Vector3d::Zero();
        LLegContactPoint = Eigen::Vector3d::Zero();
        RLegContactPoint = Eigen::Vector3d::Zero();
        tau0 = tau0_;
        tau1 = tau1_;
        Tm3 = (mass * mass * g * g) * Tm2;
    }
    Eigen::Vector3d getOdom()
    {
        return pwb;
    }
    Eigen::Vector3d getLinearVel()
    {
        return vwb;
    }

	/** @fn computeBodyVelKCFS(Eigen::Matrix3d Rwb, Eigen::Vector3d omegawb, Eigen::Vector3d pbl, Eigen::Vector3d pbr, Eigen::Vector3d vbl, Eigen::Vector3d vbr, double wl_, double wr_)
     *  @brief Computes the 3D linear base velocity in the world frame with kinematic-inertial measurements
	 *  @param Rwb Rotation of the base w.r.t the world frame (can be readily computed with base IMU measurements)
     *  @param omegawb 3D angular velocity of the base in the world frame
     *  @param pbl Relative to base left leg 3D position measurement
     *  @param pbr Relative to base right leg 3D position measurement
     *  @param vbl Relative to base left leg 3D linear velocity measurement
     *  @param vbr Relative to base right leg 3D linear velocity measurement
     *  @param wl_ Left leg contact probability
     *  @param wr_ Right leg contact probability
     *  @note Rwb, omegawb can be readily computed with base IMU measurements
     */
    void computeBodyVelKCFS(Eigen::Matrix3d Rwb, Eigen::Vector3d omegawb, Eigen::Vector3d pbl, Eigen::Vector3d pbr,
                            Eigen::Vector3d vbl, Eigen::Vector3d vbr, double wl_, double wr_)
    {
        vwb_l = -wedge(omegawb) * Rwb * pbl - Rwb * vbl;
        vwb_r = -wedge(omegawb) * Rwb * pbr - Rwb * vbr;

        vwb = wl_ * vwb_l + wr_ * vwb_r;
        // for(int i=0;i<3;i++){
        //     if(abs(vwb(i)<0.01))
        //         vwb(i)=0;
        // }
    }
	/** @fn Eigen::Matrix3d getVelocityCovariance()
     *  @brief Method to get the  3D Base velocity covariance in the world frame
	 *  @return   3D Base velocity covariance in the world frame
	 */
    Eigen::Matrix3d getVelocityCovariance()
    {

        return vwb_cov;
    }

	/** @fn  void computeLegKCFS(Eigen::Matrix3d Rwb, Eigen::Matrix3d Rbl, Eigen::Matrix3d Rbr, Eigen::Vector3d omegawb, Eigen::Vector3d omegabl, Eigen::Vector3d omegabr,Eigen::Vector3d pbl, Eigen::Vector3d pbr, Eigen::Vector3d vbl, Eigen::Vector3d vbr)
     *  @brief Computes the left/right leg orientation, 3D angular velocity and 3D linear velocity w.r.t the world frame
	 *  @param Rwb Rotation of the base w.r.t the world frame 
     *  @param Rbl Relative rotation matrix of the left leg to the base frame
     *  @param Rbr Relative rotation matrix of the right leg to the base frame
     *  @param omegawb 3D angular velocity of the base in the world frame
     *  @param omegabl Relative to base left leg 3D angular velocity measurement
     *  @param omegabr Relative to base right leg 3D angular velocity measurement
     *  @param pbl Relative to base left leg 3D position measurement
     *  @param pbr Relative to base right leg 3D position measurement
     *  @param vbl Relative to base left leg 3D linear velocity measurement
     *  @param vbr Relative to base right leg 3D linear velocity measurement
     *  @note computeBodyVelKCFS() must be called first 
    */
    void computeLegKCFS(Eigen::Matrix3d Rwb, Eigen::Matrix3d Rbl, Eigen::Matrix3d Rbr, Eigen::Vector3d omegawb, Eigen::Vector3d omegabl, Eigen::Vector3d omegabr,
                        Eigen::Vector3d pbl, Eigen::Vector3d pbr, Eigen::Vector3d vbl, Eigen::Vector3d vbr)
    {
        Rwl = Rwb * Rbl;
        Rwr = Rwb * Rbr;
        omegawl = omegawb + Rwb * omegabl;
        omegawr = omegawb + Rwb * omegabr;
        
        vwl = vwb + wedge(omegawb) * Rwb * pbl + Rwb * vbl; //速度合成公式 vwb + wedge(omegawb) * Rwb * pbl 为牵连速度
        vwr = vwb + wedge(omegawb) * Rwb * pbr + Rwb * vbr;
    }


	/** @fn  Eigen::Vector3d getLFootLinearVel()
     *  @brief Method to get the  3D left leg linear velocity  in the world frame
	 *  @return   3D left leg linear velocity  in the world frame
	 */
    Eigen::Vector3d getLFootLinearVel()
    {
        return vwl;
    }

	/** @fn  Eigen::Vector3d getRFootLinearVel()
     *  @brief Method to get the  3D right leg linear velocity  in the world frame
	 *  @return   3D right leg linear velocity  in the world frame
	 */
    Eigen::Vector3d getRFootLinearVel()
    {
        return vwr;
    }
	/** @fn   Eigen::Vector3d getLFootAngularVel()
     *  @brief Method to get the  3D left leg angular velocity  in the world frame
	 *  @return   3D left leg angular velocity  in the world frame
	 */
    Eigen::Vector3d getLFootAngularVel()
    {
        return omegawl;
    }
	/** @fn   Eigen::Vector3d getRFootAngularVel()
     *  @brief Method to get the  3D right leg angular velocity  in the world frame
	 *  @return   3D right leg angular velocity  in the world frame
	 */
    Eigen::Vector3d getRFootAngularVel()
    {
        return omegawr;
    }
	/** @fn   Eigen::Vector3d getLFootIMVPPosition()
     *  @brief Method to get the  3D left leg contact point (instantaneous pivot) in the world frame
	 *  @return   3D left leg contact point in the world frame
	 */
    Eigen::Vector3d getLFootIMVPPosition()
    {
        return LLegContactPoint;
    }
	/** @fn   Eigen::Vector3d getRFootIMVPPosition()
     *  @brief Method to get the  3D right leg contact point (instantaneous pivot) in the world frame
	 *  @return   3D right leg contact point in the world frame
	 */
    Eigen::Vector3d getRFootIMVPPosition()
    {
        return RLegContactPoint;
    }

	/** @fn    Eigen::Matrix3d getLFootIMVPOrientation()
     *  @brief Method to get the  3D left leg contact  (instantaneous pivot) orientation in the world frame
	 *  @return   3D left leg contact orientation in the world frame
	 */
    Eigen::Matrix3d getLFootIMVPOrientation()
    {
        return LLegContactOrientation;
    }

	/** @fn    Eigen::Matrix3d getRFootIMVPOrientation()
     *  @brief Method to get the  3D right leg contact  (instantaneous pivot) orientation in the world frame
	 *  @return   3D right leg contact orientation in the world frame
	 */
    Eigen::Matrix3d getRFootIMVPOrientation()
    {
        return RLegContactOrientation;
    }
	/** @fn    double getRLegContactProb()
     *  @brief Method to get right leg contact probability
	 *  @return   Right leg contact probability
	 */
    double getRLegContactProb()
    {
        return wr;
    }
	/** @fn    double getLLegContactProb()
     *  @brief Method to get left leg contact probability
	 *  @return   Left leg contact probability
	 */
    double getLLegContactProb()
    {
        return wl;
    }

	/** @fn    void computeIMVP()
     *  @brief Method to compute the left and right leg contact points (instantaneous pivot)
	 */
    void computeIMVP()      //见论文dead reckoning 式（16）
    {
        Lomega = Rwl.transpose() * omegawl;
        Romega = Rwr.transpose() * omegawr;

        double temp = Tm2 / (Lomega.squaredNorm() * Tm2 + 1.0);

        C1l = temp * wedge(Lomega);
        C2l = temp * (Lomega * Lomega.transpose() + 1.0 / Tm2 * Matrix3d::Identity());

        temp = Tm2 / (Romega.squaredNorm() * Tm2 + 1.0);

        C1r = temp * wedge(Romega);
        C2r = temp * (Romega * Romega.transpose() + 1.0 / Tm2 * Matrix3d::Identity());

        //IMVP Computations
        LpLm = C2l * LpLm;

        LpLm = LpLm + C1l * Rwl.transpose() * vwl;
        RpRm = C2r * RpRm;

        RpRm = RpRm + C1r * Rwr.transpose() * vwr;
    }

	/** @fn    void computeIMVPFT(Eigen::Vector3d lf, Eigen::Vector3d rf, Eigen::Vector3d lt, Eigen::Vector3d rt)
     *  @brief Method to compute the left and right leg contact points (instantaneous pivot) taking into account the left/right leg contact wrenches
     *  @param lf Left leg 3D contact force
     *  @param rf Right leg 3D contact force
     *  @param lt Left leg 3D contact torque
     *  @param rt Right leg 3D contact torque
	 */
    //引入力传感器信息计算IMVP（AP），见论文dead_reckoning_wrench，tau0和tau1分别运动学和力传感器的权重系数（对应论文的alpha1，alpha3）
    void computeIMVPFT(Eigen::Vector3d lf, Eigen::Vector3d rf, Eigen::Vector3d lt, Eigen::Vector3d rt)
    {
        Lomega = Rwl.transpose() * omegawl;
        Romega = Rwr.transpose() * omegawr;

        wedgerf = wedge(rf);
        wedgelf = wedge(lf);

        AL.noalias() = 1.0 / Tm2 * Matrix3d::Identity();
        AL.noalias() -= tau0 * wedge(Lomega) * wedge(Lomega);
        AL.noalias() -= tau1 / Tm3 * wedgelf * wedgelf;

        bL.noalias() = 1.0 / Tm2 * LpLm;
        bL.noalias() += tau0 * wedge(Lomega) * Rwl.transpose() * vwl;
        bL.noalias() += tau1 / Tm3 * (wedgelf * lt - wedgelf * wedgelf * plf);

        LpLm.noalias() = AL.inverse() * bL;

        AR.noalias() = 1.0 / Tm2 * Matrix3d::Identity();
        AR.noalias() -= tau0 * wedge(Romega) * wedge(Romega);
        AR.noalias() -= tau1 / Tm3 * wedgerf * wedgerf;

        bR.noalias() = 1.0 / Tm2 * RpRm;
        bR.noalias() += tau0 * wedge(Romega) * Rwr.transpose() * vwr;
        bR.noalias() += tau1 / Tm3 * (wedgerf * rt - wedgerf * wedgerf * prf);

        RpRm.noalias() = AR.inverse() * bR;

    }

	/** @fn computeDeadReckoning(Eigen::Matrix3d Rwb, Eigen::Matrix3d Rbl, Eigen::Matrix3d Rbr, Eigen::Vector3d omegawb, Eigen::Vector3d bomegab,Eigen::Vector3d pbl, Eigen::Vector3d pbr,Eigen::Vector3d vbl, Eigen::Vector3d vbr,Eigen::Vector3d omegabl, Eigen::Vector3d omegabr, double lfz, double rfz,  Eigen::Vector3d lf, Eigen::Vector3d rf, Eigen::Vector3d lt, Eigen::Vector3d rt)
     *  @brief Computes the 3D Leg odometry 
	 *  @param Rwb Rotation of the base w.r.t the world frame 
     *  @param Rbl Rotation of the left leg w.r.t to the base frame
     *  @param Rbr Rotation of the right leg w.r.t to the base frame
     *  @param omegawb 3D angular velocity of the base in the world frame
     *  @param bomegab 3D angular velocity of the base in the base frame
     *  @param pbl Relative to base left leg 3D position measurement
     *  @param pbr Relative to base right leg 3D position measurement
     *  @param vbl Relative to base left leg 3D linear velocity measurement
     *  @param vbr Relative to base right leg 3D linear velocity measurement
     *  @param omegabl Relative to base left leg 3D angular velocity measurement
     *  @param omegabr Relative to base right leg 3D angular velocity measurement
     *  @param lfz left leg vertical GRF
     *  @param rfz right leg vertical GRF
     *  @param lf Left leg 3D contact force
     *  @param rf Right leg 3D contact force
     *  @param lt Left leg 3D contact torque
     *  @param rt Right leg 3D contact torque
    */
    void computeDeadReckoning(Eigen::Matrix3d Rwb, Eigen::Matrix3d Rbl, Eigen::Matrix3d Rbr,
                              Eigen::Vector3d omegawb, 
                              Eigen::Vector3d pbl, Eigen::Vector3d pbr,
                              Eigen::Vector3d vbl, Eigen::Vector3d vbr,
                              Eigen::Vector3d omegabl, Eigen::Vector3d omegabr,
                              double lfz, double rfz,  Eigen::Vector3d lf, Eigen::Vector3d rf, Eigen::Vector3d lt, Eigen::Vector3d rt)
    {
        
         
        //地面反力最大设为mg
        lfz = cropGRF(lfz);
        rfz = cropGRF(rfz);
        
        //GRF Coefficients
        wl = (lfz + ef) / (lfz + rfz + 2.0 * ef);       //ef = 1e-8
        wr = (rfz + ef) / (lfz + rfz + 2.0 * ef);
        computeBodyVelKCFS(Rwb, omegawb, pbl, pbr, vbl, vbr, wl, wr);       //假设支撑腿速度为0，计算基体速度(kcfs代表从支撑腿的运动学计算),得到vwb
        computeLegKCFS(Rwb, Rbl, Rbr, omegawb, omegabl, omegabr, pbl, pbr, vbl, vbr); //根据刚刚计算的vwb反过来计算双脚速度
        // cout<<"=============================================="<<endl;
        // cout<<"lfz : "<<lfz<<",rfz:"<<rfz<<endl;
        // cout<<"pbl : "<<pbl.transpose()<<",pbr:"<<pbr.transpose()<<endl;
        // cout<<"vbl : "<<vbl.transpose()<<",vbr:"<<vbr.transpose()<<endl;
        // cout<<"Lomega : "<<Lomega.transpose()<<endl;
        // cout<<"vwr: "<<vwr.transpose()<<"，vwl: "<<vwl.transpose()<<endl;
        // cout<<"vwb: "<<vwb.transpose()<<endl;
        if(tau1>0)
            computeIMVPFT(lf, rf, lt, rt);
        else
            computeIMVP(); 
        //假设前后两个时刻IMVP点位置不变，由此估计双脚的位置
        pwl = pwl_ - Rwl * LpLm + Rwl_ * LpLm;
        pwr = pwr_ - Rwr * RpRm + Rwr_ * RpRm;
      //  cout<<"LpLm:"<<LpLm.transpose()<<",RpRm:"<<RpRm.transpose()<<endl;
      //  cout<<"pwl:"<<pwl.transpose()<<", pwr:"<<pwr.transpose()<<endl;

        //leg odom 根据腿的位置加上腿与基体的运动学关系估计基体位置
        pb_l = pwl - Rwb * pbl;
        pb_r = pwr - Rwb * pbr;
     //   cout<<"pb_l:"<<pb_l.transpose()<<", pb_r:"<<pb_r.transpose()<<endl;
        pwb_ = pwb;
        //左右脚估计加权
        pwb = wl * pb_l + wr * pb_r;
        //根据浮动基位置差分重新估计速度
        vwb = (pwb -pwb_)/Tm;
        //根据浮动基估计误差修正左右脚估计值
        pwl += pwb - pb_l;
        pwr += pwb - pb_r;
        // RpRmb =  pbr;
        // LpLmb =  pbl;

        // RpRmb =  Rwb.transpose()*(pwr-pwb);
        // LpLmb =  Rwb.transpose()*(pwl-pwb);
        LLegContactOrientation = Rbl;
        RLegContactOrientation = Rbr;
        LLegContactPoint = pbl + Rbl*LpLm;
        RLegContactPoint = pbr + Rbr*RpRm;

        //保存当前时刻的值
        Rwl_ = Rwl;
        Rwr_ = Rwr;
        pwl_ = pwl;
        pwr_ = pwr;
        //微分计算基体速度
        // if (!firstrun)
        //     vwb = (pwb - pwb_) * freq;
        // else
        //     firstrun = false;

        vwb_cov.noalias() =  wl * (vwb_l - vwb) * (vwb_l - vwb).transpose();
        vwb_cov.noalias() += wr * (vwb_r - vwb) * (vwb_r - vwb).transpose();

    
    }

	/** @fn     double cropGRF(double f_)
	 *  @brief  Crops the measured vertical ground reaction force (GRF) in the margins [0, mass * g]
	 *  @param  f_ Measured GRF
	 *  @return  The cropped GRF
	 */
    double cropGRF(double f_)
    {
        return max(0.0, min(f_, mass * g));
    }
	/** @fn Matrix3d wedge(Vector3d v)
	 * 	@brief Computes the skew symmetric matrix of a 3-D vector
	 *  @param v  3D Twist vector 
	 *  @return   3x3 skew symmetric representation
	 */
    Eigen::Matrix3d wedge(Eigen::Vector3d v)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Zero();

        res(0, 1) = -v(2);
        res(0, 2) = v(1);
        res(1, 2) = -v(0);
        res(1, 0) = v(2);
        res(2, 0) = -v(1);
        res(2, 1) = v(0);

        return res;
    }
};

} // namespace serow

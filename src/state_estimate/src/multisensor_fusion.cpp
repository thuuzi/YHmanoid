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

#include <iostream>
#include <algorithm>
#include <state_estimate/multisensor_fusion.h>

void multisensor_fusion::loadparams()
{
    ros::NodeHandle n_p("~");       //私有命名空间
    // Load Server Parameters
    n_p.param<std::string>("modelname", modelname, "bipedv5.urdf");
    rd = new serow::robotDyn(modelname, true);         //参数modelname,has_float_base  浮动基设置为true，否则base_link质量不考虑在内
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("lfoot", lfoot_frame, "l_ankle");
    n_p.param<std::string>("rfoot", rfoot_frame, "r_ankle");
    n_p.param<double>("imu_topic_freq", imu_freq, 100.0);
    n_p.param<double>("ft_topic_freq", ft_freq, 100.0);
    n_p.param<double>("joint_topic_freq", joint_freq, 100.0);
    n_p.param<double>("freq", freq, 100.0);
    cout << "Estimation Rate is " << freq << endl;
    n_p.param<double>("VelocityThres", VelocityThres, 0.5);
    n_p.param<double>("LosingContact", LosingContact, 5.0);
    n_p.param<bool>("calibrateIMUbiases", calibrateIMU, true);
    n_p.param<bool>("calibrateFc", calibrateFc, true);

    n_p.param<bool>("save_data", save_data, false);

    n_p.param<int>("buffer_num", buffer_num, 1);
    n_p.param<int>("maxImuCalibrationCycles", maxImuCalibrationCycles, 500);
    n_p.param<int>("maxFcCalibrationCycles", maxFcCalibrationCycles, 500);
    n_p.param<bool>("computeJointVelocity", computeJointVelocity, true);


    n_p.param<bool>("ground_truth", ground_truth, false);
    n_p.param<bool>("debug_mode", debug_mode, false);       //true
    n_p.param<bool>("rviz_view", rviz_view, true);       //true

     n_p.param<double>("joint_cutoff_freq", joint_cutoff_freq, 10.0);
    std::vector<double> affine_list;
    if (ground_truth)
    {
        n_p.param<std::string>("ground_truth_odom_topic", ground_truth_odom_topic, "ground_truth");
        n_p.param<std::string>("ground_truth_com_topic", ground_truth_com_topic, "ground_truth_com");
    }

    T_B_P.setIdentity();

    n_p.getParam("T_B_P", affine_list);
    if (affine_list.size() == 16)
    {
        T_B_P(0, 0) = affine_list[0];
        T_B_P(0, 1) = affine_list[1];
        T_B_P(0, 2) = affine_list[2];
        T_B_P(0, 3) = affine_list[3];
        T_B_P(1, 0) = affine_list[4];
        T_B_P(1, 1) = affine_list[5];
        T_B_P(1, 2) = affine_list[6];
        T_B_P(1, 3) = affine_list[7];
        T_B_P(2, 0) = affine_list[8];
        T_B_P(2, 1) = affine_list[9];
        T_B_P(2, 2) = affine_list[10];
        T_B_P(2, 3) = affine_list[11];
        T_B_P(3, 0) = affine_list[12];
        T_B_P(3, 1) = affine_list[13];
        T_B_P(3, 2) = affine_list[14];
        T_B_P(3, 3) = affine_list[15];
    }
    q_B_P = Quaterniond(T_B_P.linear());
    
    T_B_A.setIdentity();
    n_p.getParam("T_B_A", affine_list);
    if (affine_list.size() == 16)
    {
        T_B_A(0, 0) = affine_list[0];
        T_B_A(0, 1) = affine_list[1];
        T_B_A(0, 2) = affine_list[2];
        T_B_A(0, 3) = affine_list[3];
        T_B_A(1, 0) = affine_list[4];
        T_B_A(1, 1) = affine_list[5];
        T_B_A(1, 2) = affine_list[6];
        T_B_A(1, 3) = affine_list[7];
        T_B_A(2, 0) = affine_list[8];
        T_B_A(2, 1) = affine_list[9];
        T_B_A(2, 2) = affine_list[10];
        T_B_A(2, 3) = affine_list[11];
        T_B_A(3, 0) = affine_list[12];
        T_B_A(3, 1) = affine_list[13];
        T_B_A(3, 2) = affine_list[14];
        T_B_A(3, 3) = affine_list[15];
    }

    T_B_G.setIdentity();
    n_p.getParam("T_B_G", affine_list);
    if (affine_list.size() == 16)
    {
        T_B_G(0, 0) = affine_list[0];
        T_B_G(0, 1) = affine_list[1];
        T_B_G(0, 2) = affine_list[2];
        T_B_G(0, 3) = affine_list[3];
        T_B_G(1, 0) = affine_list[4];
        T_B_G(1, 1) = affine_list[5];
        T_B_G(1, 2) = affine_list[6];
        T_B_G(1, 3) = affine_list[7];
        T_B_G(2, 0) = affine_list[8];
        T_B_G(2, 1) = affine_list[9];
        T_B_G(2, 2) = affine_list[10];
        T_B_G(2, 3) = affine_list[11];
        T_B_G(3, 0) = affine_list[12];
        T_B_G(3, 1) = affine_list[13];
        T_B_G(3, 2) = affine_list[14];
        T_B_G(3, 3) = affine_list[15];
    }
    n_p.param<std::string>("odom_topic", odom_topic, "odom");
    n_p.param<std::string>("imu_topic", imu_topic, "imu");
    n_p.param<std::string>("joint_state_topic", joint_state_topic, "joint_states");
    n_p.param<double>("joint_noise_density", joint_noise_density, 0.03);
    n_p.param<std::string>("lfoot_force_torque_topic", lfsr_topic, "force_torque/left");
    n_p.param<std::string>("rfoot_force_torque_topic", rfsr_topic, "force_torque/right");

    T_FT_LL.setIdentity();
    n_p.getParam("T_FT_LL", affine_list);
    if (affine_list.size() == 16)
    {
        T_FT_LL(0, 0) = affine_list[0];
        T_FT_LL(0, 1) = affine_list[1];
        T_FT_LL(0, 2) = affine_list[2];
        T_FT_LL(0, 3) = affine_list[3];
        T_FT_LL(1, 0) = affine_list[4];
        T_FT_LL(1, 1) = affine_list[5];
        T_FT_LL(1, 2) = affine_list[6];
        T_FT_LL(1, 3) = affine_list[7];
        T_FT_LL(2, 0) = affine_list[8];
        T_FT_LL(2, 1) = affine_list[9];
        T_FT_LL(2, 2) = affine_list[10];
        T_FT_LL(2, 3) = affine_list[11];
        T_FT_LL(3, 0) = affine_list[12];
        T_FT_LL(3, 1) = affine_list[13];
        T_FT_LL(3, 2) = affine_list[14];
        T_FT_LL(3, 3) = affine_list[15];
    }
    p_FT_LL = Vector3d(T_FT_LL(0, 3), T_FT_LL(1, 3), T_FT_LL(2, 3));

    T_FT_RL.setIdentity();
    n_p.getParam("T_FT_RL", affine_list);
    if (affine_list.size() == 16)
    {
        T_FT_RL(0, 0) = affine_list[0];
        T_FT_RL(0, 1) = affine_list[1];
        T_FT_RL(0, 2) = affine_list[2];
        T_FT_RL(0, 3) = affine_list[3];
        T_FT_RL(1, 0) = affine_list[4];
        T_FT_RL(1, 1) = affine_list[5];
        T_FT_RL(1, 2) = affine_list[6];
        T_FT_RL(1, 3) = affine_list[7];
        T_FT_RL(2, 0) = affine_list[8];
        T_FT_RL(2, 1) = affine_list[9];
        T_FT_RL(2, 2) = affine_list[10];
        T_FT_RL(2, 3) = affine_list[11];
        T_FT_RL(3, 0) = affine_list[12];
        T_FT_RL(3, 1) = affine_list[13];
        T_FT_RL(3, 2) = affine_list[14];
        T_FT_RL(3, 3) = affine_list[15];
    }
    p_FT_RL = Vector3d(T_FT_RL(0, 3), T_FT_RL(1, 3), T_FT_RL(2, 3));


    n_p.param<bool>("estimateCoMwithEKF", useCoMEKF, false);
    n_p.param<int>("medianWindow", medianWindow, 10);

    //Attitude Estimation for Leg Odometry 姿态估计
    n_p.param<bool>("useMahony", useMahony, true);
    n_p.param<bool>("useMadgwick", useMadgwick, true);
    n_p.param<bool>("useEKFR", useEKFR, true);
    if (useMahony)
    {
        //Mahony Filter for Attitude Estimation
        n_p.param<double>("Mahony_Kp", Kp, 0.25);
        n_p.param<double>("Mahony_Ki", Ki, 0.0);
        mh = new serow::Mahony(imu_freq, Kp, Ki);
    }
    else if(useMadgwick)
    {
        //Madgwick Filter for Attitude Estimation
        n_p.param<double>("Madgwick_gain", beta, 0.012f);
        mw = new serow::Madgwick(imu_freq, beta);
    }
    n_p.param<double>("Tau0", Tau0, 0.5);
    n_p.param<double>("Tau1", Tau1, 0.01);
    n_p.param<double>("mass", mass, 5.14);
    n_p.param<double>("gravity", g, 9.81);
}

void multisensor_fusion::loadbaseEKFparams()
{
    baseEKF = new IMUEKF;
    ros::NodeHandle n_p("~");
    n_p.param<double>("bias_ax", bias_ax, 0.0);
    n_p.param<double>("bias_ay", bias_ay, 0.0);
    n_p.param<double>("bias_az", bias_az, 0.0);
    n_p.param<double>("bias_gx", bias_gx, 0.0);
    n_p.param<double>("bias_gy", bias_gy, 0.0);
    n_p.param<double>("bias_gz", bias_gz, 0.0);

    n_p.param<double>("accelerometer_noise_density", baseEKF->acc_qx, 0.001);
    n_p.param<double>("accelerometer_noise_density", baseEKF->acc_qy, 0.001);
    n_p.param<double>("accelerometer_noise_density", baseEKF->acc_qz, 0.001);

    n_p.param<double>("gyroscope_noise_density", baseEKF->gyr_qx, 0.0001);
    n_p.param<double>("gyroscope_noise_density", baseEKF->gyr_qy, 0.0001);
    n_p.param<double>("gyroscope_noise_density", baseEKF->gyr_qz, 0.0001);

    n_p.param<double>("accelerometer_bias_random_walk", baseEKF->accb_qx, 1.0e-04);
    n_p.param<double>("accelerometer_bias_random_walk", baseEKF->accb_qy, 1.0e-04);
    n_p.param<double>("accelerometer_bias_random_walk", baseEKF->accb_qz, 1.0e-04);
    n_p.param<double>("gyroscope_bias_random_walk", baseEKF->gyrb_qx, 1.0e-05);
    n_p.param<double>("gyroscope_bias_random_walk", baseEKF->gyrb_qy, 1.0e-05);
    n_p.param<double>("gyroscope_bias_random_walk", baseEKF->gyrb_qz, 1.0e-05);

    n_p.param<double>("odom_position_noise_density_x", baseEKF->odom_px, 1.0e-01);
    n_p.param<double>("odom_position_noise_density_y", baseEKF->odom_py, 1.0e-01);
    n_p.param<double>("odom_position_noise_density_z", baseEKF->odom_pz, 1.0e-01);
    n_p.param<double>("odom_orientation_noise_density", baseEKF->odom_ax, 1.0e-01);
    n_p.param<double>("odom_orientation_noise_density", baseEKF->odom_ay, 1.0e-01);
    n_p.param<double>("odom_orientation_noise_density", baseEKF->odom_az, 1.0e-01);

    n_p.param<double>("velocity_noise_density_x", baseEKF->vel_px, 1.0e-01);
    n_p.param<double>("velocity_noise_density_y", baseEKF->vel_py, 1.0e-01);
    n_p.param<double>("velocity_noise_density_z", baseEKF->vel_pz, 1.0e-01);
    n_p.param<double>("gravity", baseEKF->ghat, 9.81);
    n_p.param<bool>("useOutlierDetection", useOutlierDetection, false);
    n_p.param<double>("mahalanobis_TH", baseEKF->mahalanobis_TH, -1.0);
}

void multisensor_fusion::loadCoMEKFparams()
{
    nipmEKF = new CoMEKF;
    ros::NodeHandle n_p("~");
    n_p.param<double>("com_position_random_walk", nipmEKF->com_q, 1.0e-04);
    n_p.param<double>("com_velocity_random_walk", nipmEKF->comd_q, 1.0e-03);
    n_p.param<double>("external_force_random_walk", nipmEKF->fd_q, 1.0);
    n_p.param<double>("com_position_noise_density", nipmEKF->com_r, 1.0e-04);
    n_p.param<double>("com_acceleration_noise_density", nipmEKF->comdd_r, 5.0e-02);
    n_p.param<double>("Ixx", I_xx, 0.00000);
    n_p.param<double>("Iyy", I_yy, 0.00000);
    n_p.param<double>("Izz", I_zz, 0.00000);
    n_p.param<double>("bias_fx", bias_fx, 0.0);
    n_p.param<double>("bias_fy", bias_fy, 0.0);
    n_p.param<double>("bias_fz", bias_fz, 0.0);
    n_p.param<bool>("useGyroLPF", useGyroLPF, false);
    n_p.param<double>("gyro_cut_off_freq", gyro_fx, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fy, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fz, 7.0);
    n_p.param<int>("maWindow", maWindow, 10);
}

void multisensor_fusion::loadCoMLKFparams()
{
    comLKF = new CoMLKF;
    ros::NodeHandle n_p("~");
    n_p.param<double>("com_position_random_walk", comLKF->com_q, 1.0e-04);
    n_p.param<double>("com_velocity_random_walk", comLKF->comd_q, 1.0e-03);
    n_p.param<double>("external_force_random_walk", comLKF->fd_q, 1.0);
    n_p.param<double>("com_position_noise_density", comLKF->com_r, 1.0e-04);
    n_p.param<double>("com_acceleration_noise_density", comLKF->comdd_r, 5.0e-02);
    n_p.param<double>("bias_fx", bias_fx, 0.0);
    n_p.param<double>("bias_fy", bias_fy, 0.0);
    n_p.param<double>("bias_fz", bias_fz, 0.0);
    n_p.param<bool>("useGyroLPF", useGyroLPF, false);
    n_p.param<double>("gyro_cut_off_freq", gyro_fx, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fy, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fz, 7.0);
    n_p.param<int>("maWindow", maWindow, 10);
}


multisensor_fusion::multisensor_fusion()
{
    firstOdom = false;
    odom_divergence = false;
}

multisensor_fusion::~multisensor_fusion()
{
    if (is_connected_)
        disconnect();
}

void multisensor_fusion::disconnect()
{
    if (!is_connected_)
        return;
    is_connected_ = false;
}

bool multisensor_fusion::connect(const ros::NodeHandle nh)
{
    ROS_INFO_STREAM("State Estimate Initializing...");
    // Initialize ROS nodes
    n = nh;
    //加载ros参数
    loadparams();
    loadbaseEKFparams();
    if (useCoMEKF)
        loadCoMEKFparams();
    else
        loadCoMLKFparams();
    //Initialization
    init();
    //Subscribe/Publish ROS Topics/Services
    subscribe();
    advertise();
    //ros::NodeHandle np("~")
    //dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<serow::VarianceControlConfig> >(np);
    //dynamic_reconfigure::Server<serow::VarianceControlConfig>::CallbackType cb = boost::bind(&multisensor_fusion::reconfigureCB, this, _1, _2);
    // dynamic_recfg_->setCallback(cb);
    is_connected_ = true;
    ROS_INFO_STREAM("State Estimate Initialized");
    return true;
}

bool multisensor_fusion::connected()
{
    return is_connected_;
}

void multisensor_fusion::subscribe()      //订阅相关传感器话题
{
    subscribeToIMU();       //将消息存到queue中
    subscribeToFSR();
    subscribeToJointState();
    subscribeToOdom();          //订阅视觉里程计

    if (ground_truth)       //false
    {
        subscribeToGroundTruth();
        subscribeToGroundTruthCoM();
    }
}

void multisensor_fusion::init()
{
    /** Initialize Variables **/
    //Kinematic TFs
    Twb = Affine3d::Identity();
    LLegGRF = Vector3d::Zero();
    RLegGRF = Vector3d::Zero();
    LLegGRT = Vector3d::Zero();
    RLegGRT = Vector3d::Zero();
    LLegGRF_w = Vector3d::Zero();
    RLegGRF_w = Vector3d::Zero();
    copl = Vector3d::Zero();
    copr = Vector3d::Zero();
    omegawb = Vector3d::Zero();
    vwb = Vector3d::Zero();
    wbb = Vector3d::Zero();
    abb = Vector3d::Zero();
    wbb_n = Vector3d::Zero();
    abb_n = Vector3d::Zero();
    omegabl = Vector3d::Zero();
    omegabr = Vector3d::Zero();
    vbl = Vector3d::Zero();
    vbr = Vector3d::Zero();
    Twl = Affine3d::Identity();
    Twr = Affine3d::Identity();
    Tbl = Affine3d::Identity();
    Tbr = Affine3d::Identity();
    Tvb = Affine3d::Identity();
    vwl = Vector3d::Zero();
    vwr = Vector3d::Zero();
    coplw = Vector3d::Zero();
    coprw = Vector3d::Zero();
    weightl = 0.000;
    weightr = 0.000;
    no_motion_residual = Vector3d::Zero();
    kinematicsInitialized = false;
    firstUpdate = true;
    firstUpdateVwb = true;
    firstUpdateOdom = true;
    firstGyrodot = true;
    baseEKF_inc = false;
    comKF_inc = false;
    FTdata_inc = false;
    base_data_inc = false;
    com_data_inc = false;
    start_walk=false;
    //Initialize the IMU based EKF
    std::cout<<"Initializing Floating Base Estimator..."<<std::endl;
    baseEKF->init();
    save_count = 0;
    //角加速度滤波器
    if (useGyroLPF)
    {
        gyroLPF = new butterworthLPF *[3];
        for (unsigned int i = 0; i < 3; i++)
            gyroLPF[i] = new butterworthLPF();
    }
    else
    {
        gyroMAF = new MovingAverageFilter *[3];     //默认采用移动平均滤波器
        for (unsigned int i = 0; i < 3; i++)
            gyroMAF[i] = new MovingAverageFilter();
    }
    if (useCoMEKF)      //质心估计方式选择 
        nipmEKF->init();
    else
        comLKF->init();
    outlier_count = 0;
    imuCalibrationCycles = 0;
    FcCalibrationCycles = 0;
    bias_a= Vector3d::Zero();
    bias_f= Vector3d::Zero();
    bias_g= Vector3d::Zero();
}

/** Main Loop **/
void multisensor_fusion::filteringThread()
{
    static ros::Rate rate(freq); //ROS Node Loop Rate
    while (ros::ok())
    {
      //  cout<<"================= EKF loop============================="<<endl;
    //    cout<<"data size, joint: "<<joint_data.size()<<", imu: "<<base_imu_data.size()<<", LFT: "<<LLeg_FT_data.size()<<", RFT: "<< RLeg_FT_data.size()<<", odom:"<<odom_data.size()<<endl;
        //处理IMU数据
        if(base_imu_data.size()>0){
            baseIMU(base_imu_data.pop());   //获取IMU数据，w、a
            imuPoseUpdate();
            if(kinematicsInitialized){
                    //浮动基估计 预测
                baseEKFPredict();
                if (useCoMEKF)          //非线性CoM估计
                    estimateWithCoMEKF();
                base_data_inc = true;
            }
            if(comKF_inc){      //更新CoM加速度，输入世界坐标系加速度、角速度、角加速度,CoM相对位置
                filterGyrodot();
                comLKF->updateWithImu(baseEKF->acc + baseEKF->g,baseEKF->gyro,Gyrodot, CoM_enc);
         //       cout<<"update CoM acc, acc:"<<(baseEKF->acc + baseEKF->g).transpose()<<", dw"<<Gyrodot.transpose()<<endl;
                com_data_inc = true;
            }
        }
        //处理力传感器数据
        if ( LLeg_FT_data.size() > 0 && RLeg_FT_data.size() > 0)
        {
            LLeg_FT(LLeg_FT_data.pop());       //获取脚的力矩、力，计算出cop和正压力
            RLeg_FT(RLeg_FT_data.pop());
            if(!FTdata_inc)
                FTdata_inc = true;
            if(kinematicsInitialized)
                FTSensorUpdate();
            if(!useCoMEKF && baseEKF_inc) {    //质心线性预测过程
                comLKFPredict();
                com_data_inc = true;
            }
        }
        //处理关节数据
        if (joint_data.size() > 0 && FTdata_inc)
        {
            joints(joint_data.pop());       //获取关节数据（位置、速度
            KinUpdate();            //计算运动学相关数据
            //速度更新
            if(baseEKF_inc){
                if (firstUpdateVwb)
                {
                    vwb<< 0,0,0;
                    firstUpdateVwb = false;
                }
                baseEKF->updateWithTwist(vwb);
                base_data_inc = true;
             //   cout<<"update with vwb:"<<vwb.transpose()<<endl;
            }
        }
        if( odom_data.size()>0){
            Odom(odom_data.pop());
            if(baseEKF_inc){
                if (firstUpdateOdom)
                {
                    pos_ini = Twb.translation();
                    q_ini = qwb;
                    pos_update = Twb.translation();
                    q_update = qwb;
                    odom_msg_ini =odom_msg;
                    q_odom_ini = q_B_P * Quaterniond(odom_msg.pose.orientation.w, odom_msg.pose.orientation.x,
                                                odom_msg.pose.orientation.y, odom_msg.pose.orientation.z);
                    cout<<"---------------- first update odom "<<pos_update.transpose()<<endl;
                    firstUpdateOdom = false;
                }else{
                    //T_B_P: 视觉传感器到基体坐标系的变换关系  增量式更新
                    pos_update =pos_ini + T_B_P.linear() * Vector3d(odom_msg.pose.position.x - odom_msg_ini.pose.position.x,
                                                            odom_msg.pose.position.y  - odom_msg_ini.pose.position.y, odom_msg.pose.position.z  - odom_msg_ini.pose.position.z);
                    //当前视觉里程计的位姿转到基体坐标系
                    q_now = q_B_P * Quaterniond(odom_msg.pose.orientation.w, odom_msg.pose.orientation.x,
                                                odom_msg.pose.orientation.y, odom_msg.pose.orientation.z);
                    q_update = q_ini * (q_now * q_odom_ini.inverse());     //增量式更新
                }
                baseEKF->updateWithOdom(pos_update, q_update); 
            //    cout<<"==== update with odom, pos:"<<pos_update.transpose()<<endl;
                base_data_inc = true;
            }
        }
        if(base_data_inc){
            //更新CoM位置
            if(comKF_inc){
                comLKF->updateWithEnc(baseEKF->Tib * CoM_enc);
                com_data_inc = true;
               // cout<<"update CoM pos:"<<(baseEKF->Tib * CoM_enc).transpose()<<endl;
            }
            //Estimated TFs for Legs 
            Twl = baseEKF->Tib * Tbl;        //世界坐标系到左脚
            Twr = baseEKF->Tib * Tbr;        //世界坐标系到右脚
            qwl = Quaterniond(Twl.linear());
            qwr = Quaterniond(Twr.linear());
        }
        rate.sleep();
    }
    //De-allocation of Heap
    deAllocate();
}

void multisensor_fusion::baseEKFPredict()   //状态变量x(0~14)：15*1 (v,theta,r,bw,ba)
{
    //Initialize the IMU EKF state
    if (baseEKF->firstrun)       //初始化初始状态
    {
        cout<<"ini EKF pwb:"<<Twb.translation().transpose()<<endl;
        baseEKF->setdt(1.0 / imu_freq);                      //x(0~5)已初始化为0，x(#~5)姿态实际未采用，而是用Rib
        baseEKF->setBodyPos(Twb.translation());      //设置浮动基位置 x(6~8)
        baseEKF->setBodyOrientation(Twb.linear());           //设置浮动基姿态 Rib,姿态不在x里更新
        baseEKF->setAccBias(Vector3d(bias_ax, bias_ay, bias_az));        //设置imu加速度偏差 x(12~14)
        baseEKF->setGyroBias(Vector3d(bias_gx, bias_gy, bias_gz));       //设置imu角速度偏差 x(9~11)
        baseEKF->firstrun = false;
    }
    //Compute the attitude and posture with the IMU-Kinematics Fusion
    //Predict with the IMU gyro and acceleration
    baseEKF->predict(wbb, abb);  //预测步骤
     //Estimated TFs for Legs 
    Twl = baseEKF->Tib * Tbl;        //世界坐标系到左脚
    Twr = baseEKF->Tib * Tbr;        //世界坐标系到右脚
    qwl = Quaterniond(Twl.linear());
    qwr = Quaterniond(Twr.linear());
    if(!baseEKF_inc)
        baseEKF_inc = true;
}
  


void multisensor_fusion::comLKFPredict()     //通过力传感器数据进行预测过程，状态变量9维，x(0~2)质心位置，x(3~5)质心速度，x(6~8)3d外力
{
    if (comLKF->firstrun)
    {
        comLKF->setdt(1.0 / ft_freq);
        comLKF->setParams(mass, g);
        comLKF->setCoMPos(CoM_kin);       //x(0~2) 质心初始位置
        comLKF->setCoMExternalForce(Vector3d(bias_fx, bias_fy, bias_fz));  //x(6~8) 所加载外力
        comLKF->firstrun = false;
        if (useGyroLPF)     //角加速度低通滤波器
        {
            gyroLPF[0]->init("gyro X LPF", freq, gyro_fx);      //gyro_fx截止频率
            gyroLPF[1]->init("gyro Y LPF", freq, gyro_fy);
            gyroLPF[2]->init("gyro Z LPF", freq, gyro_fz);
        }
        else        //移动平均滤波器
        {   
            for (unsigned int i = 0; i < 3; i++)
                gyroMAF[i]->setParams(maWindow);
        }
        cout<<"-------------- CoM LKF ini, pc:"<<CoM_kin.transpose()<<endl;
    }
    comLKF->predict(GRF_fsr);    //输入总接触力（转换至世界坐标系）
    if(!comKF_inc)
        comKF_inc=true;
}

void multisensor_fusion::estimateWithbaseEKF()     //状态变量x(0~14)：15*1 (v,theta,r,bw,ba)
{
    //Initialize the IMU EKF state
    if (baseEKF->firstrun)       //初始化初始状态
    {
        baseEKF->setdt(1.0 / freq);                      //x(0~5)已初始化为0，x(#~5)姿态实际未采用，而是用Rib
        baseEKF->setBodyPos(Twb.translation());      //设置浮动基位置 x(6~8)
        baseEKF->setBodyOrientation(Twb.linear());           //设置浮动基姿态 Rib,姿态不在x里更新
        baseEKF->setAccBias(Vector3d(bias_ax, bias_ay, bias_az));        //设置imu加速度偏差 x(12~14)
        baseEKF->setGyroBias(Vector3d(bias_gx, bias_gy, bias_gz));       //设置imu角速度偏差 x(9~11)
        baseEKF->firstrun = false;
    }

    //Compute the attitude and posture with the IMU-Kinematics Fusion
    //Predict with the IMU gyro and acceleration
    
    baseEKF->predict(wbb, abb);  //预测步骤
  
    //Update EKF 设置初始值
    if (firstUpdate)
    {
        pos_update = Twb.translation();
        q_update = qwb;
        //First Update
        firstUpdate = false;
        cout<<"fist update, pos:"<<pos_update.transpose()<<endl;
        baseEKF->updateWithLegOdom(pos_update, q_update);       //根据里程计更新，legodom表示里程计为准确值
    }
    else
    {
           //采用视觉里程计
            if (!odom_divergence)       //视觉里程计不发散
            {
                if (outlier_count < 3)
                {
                    pos_update_ = pos_update;
                    //T_B_P: 视觉传感器到基体坐标系的变换关系  增量式更新
                    pos_update += T_B_P.linear() * Vector3d(odom_msg.pose.position.x - odom_msg_.pose.position.x,
                                                            odom_msg.pose.position.y - odom_msg_.pose.position.y, odom_msg.pose.position.z - odom_msg_.pose.position.z);
                    cout<<"update, pos:"<<pos_update.transpose()<<endl;
                    //当前视觉里程计的位姿转到基体坐标系
                    q_now = q_B_P * Quaterniond(odom_msg.pose.orientation.w, odom_msg.pose.orientation.x,
                                                odom_msg.pose.orientation.y, odom_msg.pose.orientation.z);
                    //上一时刻视觉里程计的位姿转到基体坐标系
                    q_prev = q_B_P * Quaterniond(odom_msg_.pose.orientation.w, odom_msg_.pose.orientation.x,
                                                 odom_msg_.pose.orientation.y, odom_msg_.pose.orientation.z);

                    q_update_ = q_update;

                    q_update *= (q_now * q_prev.inverse());     //增量式更新

                    odom_msg_ = odom_msg;

                    outlier = baseEKF->updateWithOdom(pos_update, q_update, useOutlierDetection);    //默认不检测outlier
                    //速度更新
                    baseEKF->updateWithTwist(vwb);
                    if (outlier)
                    {
                        outlier_count++;
                        pos_update = pos_update_;
                        q_update = q_update_;
                    }
                    else
                    {
                        outlier_count = 0;
                    }
                }
                else
                {
                    odom_divergence = true;
                }
            }
            if (odom_divergence)    //里程计发散 则采用运动学更新
            {
                std::cout<<"Odom divergence, updating only with leg odometry"<<std::endl;
                pos_update += pos_leg_update;
                q_update *= q_leg_update;
                baseEKF->updateWithTwistRotation(vwb, q_update);     //根据浮动基速度（运动学计算得到）更新
                //baseEKF->updateWithTwist(vwb);
            }
    }

    //Estimated TFs for Legs 
    Twl = baseEKF->Tib * Tbl;        //世界坐标系到左脚
    Twr = baseEKF->Tib * Tbr;        //世界坐标系到右脚
    qwl = Quaterniond(Twl.linear());
    qwr = Quaterniond(Twr.linear());
}

void multisensor_fusion::estimateWithCoMEKF()     //nipmEKF 质心估计器 见论文serow2,状态变量9维，x(0~2)质心位置，x(3~5)质心速度，x(6~8)3d外力
{
    if (nipmEKF->firstrun)
    {
        nipmEKF->setdt(1.0 / freq);
        nipmEKF->setParams(mass, I_xx, I_yy, g);
        nipmEKF->setCoMPos(CoM_kin);       //x(0~2) 质心位置
        nipmEKF->setCoMExternalForce(Vector3d(bias_fx, bias_fy, bias_fz));  //x(6~8) 所加载外力
        nipmEKF->firstrun = false;
        if (useGyroLPF)     //角加速度低通滤波器
        {
            gyroLPF[0]->init("gyro X LPF", freq, gyro_fx);      //gyro_fx截止频率
            gyroLPF[1]->init("gyro Y LPF", freq, gyro_fy);
            gyroLPF[2]->init("gyro Z LPF", freq, gyro_fz);
        }
        else        //移动平均滤波器
        {   
            for (unsigned int i = 0; i < 3; i++)
                gyroMAF[i]->setParams(maWindow);
        }
    }

    //Compute the COP in the Inertial Frame
    computeGlobalCOP(Twl, Twr);
    filterGyrodot();        //滤波得到角加速度
    DiagonalMatrix<double, 3> Inertia(I_xx, I_yy, I_zz);

    nipmEKF->predict(COP_fsr, GRF_fsr, Inertia * Gyrodot);    //输入压力中心、GRF_fsr 地面反力(只用到正压力)，近似角动量变化率（均表示在世界坐标系
//    cout<<"Gyrodot:"<<Gyrodot.transpose()<<endl;
    // nipmEKF->update(                        //更新过程，观测量（在世界坐标系下）
    //     baseEKF->acc + baseEKF->g,        //实际加速度
    //     baseEKF->Tib * CoM_enc,          //编码器计算的质心相对基座坐标系的位置 
    //     baseEKF->gyro, Gyrodot);                 //角速度、角加速度

}

void multisensor_fusion::imuPoseUpdate()
{
  //  cout<<"wbb: "<<wbb.transpose()<<"abb:"<<abb.transpose()<<endl;
    if(baseEKF_inc){
        wbb_n = wbb - baseEKF->bgyr;
        abb_n = abb - baseEKF->bacc;
    //    cout<<"baseEKF->bgyr:"<<baseEKF->bgyr<<endl;
    }else{
        wbb_n << wbb(0) - bias_gx,wbb(1) - bias_gy,wbb(2) - bias_gz;
        abb_n << abb(0) - bias_ax,abb(1) - bias_ay,abb(2) - bias_az;
    }
  //  cout<<"wbb n: "<<wbb_n.transpose()<<"abb n :"<<abb_n.transpose()<<endl;
    if(abb_n(1)>0.2)
        start_walk=true;
    if (useMahony)    
    {
        mh->updateIMU(wbb_n, abb_n);
        qwb = Quaterniond(Twb.linear());  //Quaterniond(mh->getR());
        omegawb = mh->getGyro();        //转到世界坐标系下的基体角速度
    }
    else if(useMadgwick)
    {
        mw->updateIMU(wbb_n, abb_n);
        qwb = Quaterniond(mw->getR());
        omegawb = mw->getGyro();
    }else if(useEKFR){
        if(start_walk){
            qwb = baseEKF->qib;
            omegawb = baseEKF->Rib*wbb_n;
        }else{
            qwb = Quaterniond(1,0,0,0);
            omegawb = wbb_n;
        }    
    }else{
        qwb = Quaterniond(Tvb.linear());
        omegawb = Tvb.linear()*wbb_n;
    }
    Twb.linear() = qwb.toRotationMatrix();
}

void multisensor_fusion::FTSensorUpdate()
{
    //将足端力转换到世界坐标系
    RLegGRF_w = Twb.linear() * Tbr.linear() * RLegGRF;
    LLegGRF_w = Twb.linear() * Tbl.linear() * LLegGRF;
    //双脚地面反力之和 用于com估计
    GRF_fsr = RLegGRF_w + LLegGRF_w;
    //cout<<"LLegGRF: "<<LLegGRF.transpose()<<" ,RLegGRF: "<<RLegGRF.transpose()<<", GRF_fsr:"<<GRF_fsr.transpose()<<endl;
}

void multisensor_fusion::KinUpdate()
{
   
    //更新运动学
    rd->updateJointConfig(joint_state_pos_map, joint_state_vel_map, joint_noise_density);
    //运动学计算质心位置
    CoM_enc = rd->comPosition();
    //运动学计算左右脚位置
    Tbl.translation() = rd->linkPosition(lfoot_frame);      //基体到左脚 Affine3d 
    qbl = rd->linkOrientation(lfoot_frame);
    Tbl.linear() = qbl.toRotationMatrix();      //旋转矩阵
    Tbr.translation() = rd->linkPosition(rfoot_frame);       //基体到右脚
    qbr = rd->linkOrientation(rfoot_frame);
    Tbr.linear() = qbr.toRotationMatrix();
    //初始化世界坐标系位置
    if (!kinematicsInitialized)     
    {
        Twl.translation() << Tbl.translation()(0), Tbl.translation()(1), 0.00;  //世界坐标系z原点在脚(la)坐标系原点，x，y原点为初始base_link坐标系x y原点
        Twl.linear() = Tbl.linear();
        Twr.translation() << Tbr.translation()(0), Tbr.translation()(1), 0.00;
        Twr.linear() = Tbr.linear();
        cout<<"init left leg pos:"<< Twl.translation().transpose()<<", right leg pos:"<<Twr.translation().transpose()<<endl;
        dr = new serow::deadReckoning(Twl.translation(), Twr.translation(), Twl.linear(), Twr.linear(),
                                      mass, Tau0, Tau1, joint_freq, g, p_FT_LL, p_FT_RL);     //leg odom,航位推算（根据足底力和编码器的里程计
        kinematicsInitialized = true;
    }
    
    //通过pinocchio获得雅克比矩阵计算足端速度和角速度（相对基体
    omegabl = rd->getAngularVelocity(lfoot_frame);
    omegabr = rd->getAngularVelocity(rfoot_frame);
    vbl = rd->getLinearVelocity(lfoot_frame);
    vbr = rd->getLinearVelocity(rfoot_frame);
  //  std::cout<<"vbl:"<<vbl.transpose()<<",  vbr:"<<vbr.transpose()<<std::endl;  
    dr->computeDeadReckoning(Twb.linear(), Tbl.linear(), Tbr.linear(), omegawb,
                             Tbl.translation(), Tbr.translation(),
                             vbl, vbr, omegabl,omegabr,
                             LLegGRF(2), RLegGRF(2), LLegGRF, RLegGRF, LLegGRT, RLegGRT);

    //leg odom获得Twb
    Twb.translation() = dr->getOdom();
    //基体相对脚的速度，用于浮动基ekf更新
    vwb = dr->getLinearVel();  
    //脚的速度，无用         
    vwl = dr->getLFootLinearVel();
    vwr = dr->getRFootLinearVel();
    omegawl = dr->getLFootAngularVel();
    omegawr = dr->getRFootAngularVel();
    //根据运动学计算的CoM位置
    if(baseEKF_inc)
        CoM_kin =baseEKF->Tib * CoM_enc;     //根据运动学计算的CoM位置
    else
        CoM_kin =Twb * CoM_enc;     
}

void multisensor_fusion::deAllocate()
{
    for (unsigned int i = 0; i < number_of_joints; i++)
        delete[] JointVF[i];
    delete[] JointVF;
    if (useGyroLPF)
    {
        for (unsigned int i = 0; i < 3; i++)
            delete[] gyroLPF[i];
        delete[] gyroLPF;
    }
    else
    {
        for (unsigned int i = 0; i < 3; i++)
            delete[] gyroMAF[i];
        delete[] gyroMAF;
    }
    if (useCoMEKF)
        delete nipmEKF;
    else
        delete comLKF;

    delete baseEKF;
    delete rd;
    delete mw;
    delete mh;
    delete dr;
}

void multisensor_fusion::filterGyrodot()      //滤波得到角加速度
{
    if (!firstGyrodot)
    {
        //Compute numerical derivative

        Gyrodot = (baseEKF->gyro - Gyro_) * imu_freq;


        if (useGyroLPF)
        {
            Gyrodot(0) = gyroLPF[0]->filter(Gyrodot(0));
            Gyrodot(1) = gyroLPF[1]->filter(Gyrodot(1));
            Gyrodot(2) = gyroLPF[2]->filter(Gyrodot(2));
        }
        else
        {       //移动平均滤波
            gyroMAF[0]->filter(Gyrodot(0));
            gyroMAF[1]->filter(Gyrodot(1));
            gyroMAF[2]->filter(Gyrodot(2));

            Gyrodot(0) = gyroMAF[0]->x;
            Gyrodot(1) = gyroMAF[1]->x;
            Gyrodot(2) = gyroMAF[2]->x;
        }
    }
    else
    {   //初始化
        Gyrodot = Vector3d::Zero();
        firstGyrodot = false;
    }
    //缓存上一次的角速度
    Gyro_ = baseEKF->gyro;

}

void multisensor_fusion::computeGlobalCOP(Affine3d Twl_, Affine3d Twr_)
{

    //Compute the CoP wrt the Support Foot Frame
    coplw = Twl_ * copl;
    coprw = Twr_ * copr;

    if (weightl + weightr > 0.0)
    {
        COP_fsr = (weightl * coplw + weightr * coprw) / (weightl + weightr);        //双脚加权压力中心
    }
    else
    {
        COP_fsr = Vector3d::Zero();
    }
}

void multisensor_fusion::advertise()
{


    baseIMU_pub = n.advertise<sensor_msgs::Imu>("serow/base/imu", 1000);

    LLegOdom_pub = n.advertise<nav_msgs::Odometry>("serow/LLeg/odom", 1000);

    RLegOdom_pub = n.advertise<nav_msgs::Odometry>("serow/RLeg/odom", 1000);

    baseOdom_pub = n.advertise<nav_msgs::Odometry>("serow/base/odom", 1000);

    if (computeJointVelocity)
        joint_pub = n.advertise<sensor_msgs::JointState>("serow/joint_states", 1000);

 
    CoMOdom_pub = n.advertise<nav_msgs::Odometry>("serow/CoM/odom", 1000);
    ExternalWrench_pub = n.advertise<geometry_msgs::WrenchStamped>("serow/CoM/wrench", 1000);
    COP_pub = n.advertise<geometry_msgs::PointStamped>("serow/COP", 1000);

    legOdom_pub = n.advertise<nav_msgs::Odometry>("serow/base/leg_odom", 1000);



    if (debug_mode)
    {
        rel_LLegPose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/rel_LLeg/pose", 1000);

        rel_RLegPose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/rel_RLeg/pose", 1000);

        rel_CoMPose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/rel_CoM/pose", 1000);
    }

    if (rviz_view){
        ground_truth_odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/ground_truth/base/odom/pose", 1000);
        ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/base/odom/path", 1000);
        leg_odom_path_pub = n.advertise<nav_msgs::Path>("serow/base/leg_odom/path", 1000);
        leg_odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/base/leg_odom/pose", 1000);
        odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/base/odom/pose", 1000);
        odom_path_pub = n.advertise<nav_msgs::Path>("serow/base/odom/path", 1000);
        slam_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/base/orb_slam3/pose",1000);
        ground_truth_com_point_pub = n.advertise<geometry_msgs::PointStamped>("serow/ground_truth/com/point", 1000);
        ground_truth_com_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/com/path", 1000);
        com_point_pub = n.advertise<geometry_msgs::PointStamped>("serow/com/point", 1000);
        com_kin_point_pub = n.advertise<geometry_msgs::PointStamped>("serow/kin/com/point", 1000);
        com_path_pub = n.advertise<nav_msgs::Path>("serow/com/path", 1000);
    }
}

void multisensor_fusion::subscribeToJointState()
{

    joint_state_sub = n.subscribe(joint_state_topic, 1000, &multisensor_fusion::joint_stateCb, this);
    firstJointStates = true;
}

void multisensor_fusion::joint_stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_data.push(*msg);
   // cout<<"get joint, buffer: "<<joint_data.size()<<",stamp: "<<msg->header.stamp<<endl;

    if (joint_data.size() > buffer_num)
        joint_data.pop();
}

void multisensor_fusion::joints(const sensor_msgs::JointState &msg)
{

    if (firstJointStates)       //初始化joints(
    {
        number_of_joints = msg.name.size();
        joint_state_vel.resize(number_of_joints);
        joint_state_pos.resize(number_of_joints);
        if (computeJointVelocity)
        {
            JointVF = new JointDF *[number_of_joints];
            for (unsigned int i = 0; i < number_of_joints; i++)
            {
                JointVF[i] = new JointDF();
                JointVF[i]->init(msg.name[i], freq, joint_cutoff_freq);     //关节速度低通滤波器
            }
        }
        firstJointStates = false;
    }

    if (computeJointVelocity)       //关节速度通过位置微分滤波计算得到
    {
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            joint_state_pos[i] = msg.position[i];
            joint_state_vel[i] = JointVF[i]->filter(msg.position[i]);
            joint_state_pos_map[msg.name[i]] = joint_state_pos[i];
            joint_state_vel_map[msg.name[i]] = joint_state_vel[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            joint_state_pos[i] = msg.position[i];
            joint_state_vel[i] = msg.velocity[i];
            joint_state_pos_map[msg.name[i]] = joint_state_pos[i];
            joint_state_vel_map[msg.name[i]] = joint_state_vel[i];
        }
    }
}

void multisensor_fusion::subscribeToOdom()
{

    odom_sub = n.subscribe(odom_topic, 1000, &multisensor_fusion::odomCb, this);
    firstOdom = true;
}

void multisensor_fusion::odomCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odom_data.push(*msg);
//    cout<<"get odom, buffer: "<<odom_data.size()<<",stamp: "<<msg->header.stamp<<endl;
    if (odom_data.size() > buffer_num)
        odom_data.pop();
}
void multisensor_fusion::Odom(const geometry_msgs::PoseStamped &msg){
    odom_msg = msg;
    if (firstOdom)
    {
        odom_msg_ = odom_msg;       //初始位置
        firstOdom = false;
    }
}

void multisensor_fusion::subscribeToGroundTruth()
{
    ground_truth_odom_sub = n.subscribe(ground_truth_odom_topic, 1000, &multisensor_fusion::ground_truth_odomCb, this);
    firstGT = true;
}

void multisensor_fusion::ground_truth_odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    ground_truth_odom_msg = *msg;
    Tvb.translation() = Vector3d(ground_truth_odom_msg.pose.pose.position.x, ground_truth_odom_msg.pose.pose.position.y, ground_truth_odom_msg.pose.pose.position.z);
    Tvb.linear() =  Quaterniond(ground_truth_odom_msg.pose.pose.orientation.w, ground_truth_odom_msg.pose.pose.orientation.x, ground_truth_odom_msg.pose.pose.orientation.y, ground_truth_odom_msg.pose.pose.orientation.z).toRotationMatrix();

    gt_odom = Tvb.translation();
    gt_odomq = Quaterniond(Tvb.linear());
    ground_truth_odom_pub_msg.child_frame_id = base_link_frame;
    ground_truth_odom_pub_msg.header.stamp = ros::Time::now();
    ground_truth_odom_pub_msg.header.frame_id = "world";
    ground_truth_odom_pub_msg.pose.pose.position.x = gt_odom(0);
    ground_truth_odom_pub_msg.pose.pose.position.y = gt_odom(1);
    ground_truth_odom_pub_msg.pose.pose.position.z = gt_odom(2);
    ground_truth_odom_pub_msg.pose.pose.orientation.w = gt_odomq.w();
    ground_truth_odom_pub_msg.pose.pose.orientation.x = gt_odomq.x();
    ground_truth_odom_pub_msg.pose.pose.orientation.y = gt_odomq.y();
    ground_truth_odom_pub_msg.pose.pose.orientation.z = gt_odomq.z();
}

void multisensor_fusion::subscribeToGroundTruthCoM()
{
    ground_truth_com_sub = n.subscribe(ground_truth_com_topic, 1000, &multisensor_fusion::ground_truth_comCb, this);
    firstGTCoM = true;
}
void multisensor_fusion::ground_truth_comCb(const nav_msgs::Odometry::ConstPtr &msg)
{

        ground_truth_com_odom_msg = *msg;
        temp = Vector3d(ground_truth_com_odom_msg.pose.pose.position.x, ground_truth_com_odom_msg.pose.pose.position.y, ground_truth_com_odom_msg.pose.pose.position.z);
        tempq = Quaterniond(ground_truth_com_odom_msg.pose.pose.orientation.w, ground_truth_com_odom_msg.pose.pose.orientation.x, ground_truth_com_odom_msg.pose.pose.orientation.y, ground_truth_com_odom_msg.pose.pose.orientation.z);

        ground_truth_com_odom_msg.child_frame_id = "CoM";
        ground_truth_com_odom_msg.header.stamp = ros::Time::now();
        ground_truth_com_odom_msg.header.frame_id = "world";

        ground_truth_com_odom_msg.pose.pose.position.x = temp(0);
        ground_truth_com_odom_msg.pose.pose.position.y = temp(1);
        ground_truth_com_odom_msg.pose.pose.position.z = temp(2);

        ground_truth_com_odom_msg.pose.pose.orientation.w = tempq.w();
        ground_truth_com_odom_msg.pose.pose.orientation.x = tempq.x();
        ground_truth_com_odom_msg.pose.pose.orientation.y = tempq.y();
        ground_truth_com_odom_msg.pose.pose.orientation.z = tempq.z();
}


void multisensor_fusion::subscribeToIMU()
{
    imu_sub = n.subscribe(imu_topic, 1000, &multisensor_fusion::imuCb, this);
}

void multisensor_fusion::imuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    base_imu_data.push(*msg);
  //  cout<<"get imu, buffer: "<<base_imu_data.size()<<",stamp: "<<msg->header.stamp<<",  imu ax:"<<msg->linear_acceleration.x<<endl;
    if (base_imu_data.size() > buffer_num)
        base_imu_data.pop();
}

void multisensor_fusion::baseIMU(const sensor_msgs::Imu &msg)
{
    //T_B_G/A默认为单位阵
    wbb = T_B_G.linear() * Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    abb = T_B_A.linear() * Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
}

void multisensor_fusion::calibrate(){
    ros::Rate r(1000);
    while(ros::ok()){
        if(calibrateIMU){
            if(base_imu_data.size()>0){
                baseIMU(base_imu_data.pop());
                //这里获得原始的消息加速度不包含g，因此标定加上g
                if (imuCalibrationCycles < maxImuCalibrationCycles )
                {
                    bias_g += wbb;
                    bias_a += abb - Vector3d(0, 0, g);
                    imuCalibrationCycles++;
                    cout<<"= IMU calibrate: "<<imuCalibrationCycles<<"/"<<maxImuCalibrationCycles<<endl;
                }else    //标定结束
                {
                    bias_ax = bias_a(0) / imuCalibrationCycles;
                    bias_ay = bias_a(1) / imuCalibrationCycles;
                    bias_az = bias_a(2) / imuCalibrationCycles;
                    bias_gx = bias_g(0) / imuCalibrationCycles;
                    bias_gy = bias_g(1) / imuCalibrationCycles;
                    bias_gz = bias_g(2) / imuCalibrationCycles;
                    std::cout << "---------------------IMU Calibration finished " << std::endl;
                    std::cout << "Gyro biases " << bias_gx << " " << bias_gy << " " << bias_gz << std::endl;
                    std::cout << "Acc biases " << bias_ax << " " << bias_ay << " " << bias_az << std::endl;
                    calibrateIMU = false;
                }
            }
        }
        if(calibrateFc){
            if ( LLeg_FT_data.size() > 0 && RLeg_FT_data.size() > 0){
                LLeg_FT(LLeg_FT_data.pop());       //获取脚的力矩、力，计算出cop和正压力
                RLeg_FT(RLeg_FT_data.pop());
                GRF_fsr = RLegGRF + LLegGRF;
                if (FcCalibrationCycles < maxFcCalibrationCycles )
                {
                    bias_f -= GRF_fsr;
                    FcCalibrationCycles++;
                    cout<<"- Fc calibrate: "<<FcCalibrationCycles<<"/"<<maxFcCalibrationCycles<<","<<bias_f.transpose()<<endl;
                }else    //标定结束
                {
                    bias_fx = bias_f(0) / FcCalibrationCycles;
                    bias_fy = bias_f(1) / FcCalibrationCycles;
                    bias_fz = bias_f(2) / FcCalibrationCycles + mass*g;
                    std::cout << "--------------------fc Calibration finished  " << std::endl;
                    std::cout << "fc biases " << bias_fx << " " << bias_fy << " " << bias_fz << std::endl;
                    calibrateFc = false;
                }
            }
        }
        if(!calibrateFc && !calibrateIMU)
            break;
        ros::spinOnce();
        r.sleep();
    }
}

void multisensor_fusion::subscribeToFSR()
{
    //Left Foot Wrench
    lfsr_sub = n.subscribe(lfsr_topic, 1000, &multisensor_fusion::lfsrCb, this);
    //Right Foot Wrench
    rfsr_sub = n.subscribe(rfsr_topic, 1000, &multisensor_fusion::rfsrCb, this);
}

void multisensor_fusion::lfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    LLeg_FT_data.push(*msg);
  //  cout<<"get LFT, buffer: "<<LLeg_FT_data.size()<<",stamp: "<<msg->header.stamp<<endl;
    if (LLeg_FT_data.size() > buffer_num)
        LLeg_FT_data.pop();
}

void multisensor_fusion::LLeg_FT(const geometry_msgs::WrenchStamped &msg)
{
    LLegGRF(0) = msg.wrench.force.x;        //左脚的力和力矩
    LLegGRF(1) = msg.wrench.force.y;
    LLegGRF(2) = msg.wrench.force.z;
    LLegGRT(0) = msg.wrench.torque.x;
    LLegGRT(1) = msg.wrench.torque.y;
    LLegGRT(2) = msg.wrench.torque.z;
    LLegGRF = T_FT_LL.linear() * LLegGRF;
    LLegGRT = T_FT_LL.linear() * LLegGRT;

    weightl = 0;
    copl = Vector3d::Zero();            //左脚压力中心cop
    if (LLegGRF(2) >= LosingContact)        //触地状态
    {
        copl(0) = -LLegGRT(1) / LLegGRF(2);
        copl(1) = LLegGRT(0) / LLegGRF(2);
        weightl = LLegGRF(2) / g;       //用于计算压力中心左脚贡献的比例
    }
    else
    {
        copl = Vector3d::Zero();
        LLegGRF = Vector3d::Zero();
        LLegGRT = Vector3d::Zero();
        weightl = 0.0;
    }
}

void multisensor_fusion::rfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    RLeg_FT_data.push(*msg);
 //   cout<<"get RFT, buffer: "<<RLeg_FT_data.size()<<",stamp: "<<msg->header.stamp<<endl;
    if (RLeg_FT_data.size() > buffer_num)
        RLeg_FT_data.pop();

    //rfsr_msg = *msg;
}
void multisensor_fusion::RLeg_FT(const geometry_msgs::WrenchStamped &msg)
{
    RLegGRF(0) = msg.wrench.force.x;
    RLegGRF(1) = msg.wrench.force.y;
    RLegGRF(2) = msg.wrench.force.z;
    RLegGRT(0) = msg.wrench.torque.x;
    RLegGRT(1) = msg.wrench.torque.y;
    RLegGRT(2) = msg.wrench.torque.z;
    RLegGRF = T_FT_RL.linear() * RLegGRF;
    RLegGRT = T_FT_RL.linear() * RLegGRT;
    copr = Vector3d::Zero();
    weightr = 0.0;
    if (RLegGRF(2) >= LosingContact)
    {
        copr(0) = -RLegGRT(1) / RLegGRF(2);
        copr(1) = RLegGRT(0) / RLegGRF(2);
        weightr = RLegGRF(2) / g;
    }
    else
    {
        copr = Vector3d::Zero();
        RLegGRF = Vector3d::Zero();
        RLegGRT = Vector3d::Zero();
        weightr = 0.0;
    }
}
void multisensor_fusion::publishBodyEstimates()
{


    sensor_msgs::Imu tmp_imu_msg;
    tmp_imu_msg.header.stamp = ros::Time::now();
    tmp_imu_msg.header.frame_id = "world";
    tmp_imu_msg.linear_acceleration.x = baseEKF->accX;
    tmp_imu_msg.linear_acceleration.y = baseEKF->accY;
    tmp_imu_msg.linear_acceleration.z = baseEKF->accZ;
    tmp_imu_msg.angular_velocity.x = baseEKF->gyroX;
    tmp_imu_msg.angular_velocity.y = baseEKF->gyroY;
    tmp_imu_msg.angular_velocity.z = baseEKF->gyroZ;
    baseIMU_pub.publish(tmp_imu_msg);

    //  serow/base/odom:发布imu ekf结果
    ekf_odom_msg.child_frame_id = base_link_frame;
    ekf_odom_msg.header.stamp = ros::Time::now();
    ekf_odom_msg.header.frame_id = "world";
    ekf_odom_msg.pose.pose.position.x = baseEKF->rX;
    ekf_odom_msg.pose.pose.position.y = baseEKF->rY;
    ekf_odom_msg.pose.pose.position.z = baseEKF->rZ;
    ekf_odom_msg.pose.pose.orientation.x = baseEKF->qib.x();
    ekf_odom_msg.pose.pose.orientation.y = baseEKF->qib.y();
    ekf_odom_msg.pose.pose.orientation.z = baseEKF->qib.z();
    ekf_odom_msg.pose.pose.orientation.w = baseEKF->qib.w();

    ekf_odom_msg.twist.twist.linear.x = baseEKF->velX;
    ekf_odom_msg.twist.twist.linear.y = baseEKF->velY;
    ekf_odom_msg.twist.twist.linear.z = baseEKF->velZ;
    ekf_odom_msg.twist.twist.angular.x = baseEKF->gyroX;
    ekf_odom_msg.twist.twist.angular.y = baseEKF->gyroY;
    ekf_odom_msg.twist.twist.angular.z = baseEKF->gyroZ;

    //for(int i=0;i<36;i++)
    //odom_est_msg.pose.covariance[i] = 0;
    baseOdom_pub.publish(ekf_odom_msg);
   
    leg_odom_msg.child_frame_id = base_link_frame;
    leg_odom_msg.header.stamp = ros::Time::now();
    leg_odom_msg.header.frame_id = "world";
    leg_odom_msg.pose.pose.position.x = Twb.translation()(0);
    leg_odom_msg.pose.pose.position.y = Twb.translation()(1);
    leg_odom_msg.pose.pose.position.z = Twb.translation()(2);
    leg_odom_msg.pose.pose.orientation.x = qwb.x();
    leg_odom_msg.pose.pose.orientation.y = qwb.y();
    leg_odom_msg.pose.pose.orientation.z = qwb.z();
    leg_odom_msg.pose.pose.orientation.w = qwb.w();
    leg_odom_msg.twist.twist.linear.x = vwb(0);
    leg_odom_msg.twist.twist.linear.y = vwb(1);
    leg_odom_msg.twist.twist.linear.z = vwb(2);
    leg_odom_msg.twist.twist.angular.x = omegawb(0);
    leg_odom_msg.twist.twist.angular.y = omegawb(1);
    leg_odom_msg.twist.twist.angular.z = omegawb(2);
    legOdom_pub.publish(leg_odom_msg);

    if(rviz_view){
        if(odom_path_msg.poses.size()>1000)
            odom_path_msg.poses.clear();
        if(ground_truth_odom_path_msg.poses.size()>1000)
            ground_truth_odom_path_msg.poses.clear();
        if(leg_odom_path_msg.poses.size()>1000)
            leg_odom_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        
        ground_truth_odom_path_msg.header = ground_truth_odom_pub_msg.header;
        temp_pose.header = ground_truth_odom_pub_msg.header;
        temp_pose.pose = ground_truth_odom_pub_msg.pose.pose;
        ground_truth_odom_path_msg.poses.push_back(temp_pose);
        ground_truth_odom_path_pub.publish(ground_truth_odom_path_msg);
        ground_truth_odom_pose_pub.publish(temp_pose);

        leg_odom_path_msg.header = leg_odom_msg.header;
        temp_pose.header = leg_odom_msg.header;
        temp_pose.pose = leg_odom_msg.pose.pose;
        leg_odom_path_msg.poses.push_back(temp_pose);
        leg_odom_path_pub.publish(leg_odom_path_msg);
        leg_odom_pose_pub.publish(temp_pose);

        odom_path_msg.header = ekf_odom_msg.header;
        temp_pose.header = ekf_odom_msg.header;
       // cout<<"ekf n.z :"<< ekf_odom_msg.pose.pose.position.z <<endl;
        temp_pose.pose = ekf_odom_msg.pose.pose;
        odom_path_msg.poses.push_back(temp_pose);
        //cout<<"temp .pose.position.z :"<<temp_pose.pose.position.z <<endl;
        odom_path_pub.publish(odom_path_msg);
        odom_pose_pub.publish(temp_pose);

        //发布slam输出位姿
        temp_pose.pose.position.x = pos_update(0);
        temp_pose.pose.position.y = pos_update(1);
        temp_pose.pose.position.z = pos_update(2);
        temp_pose.pose.orientation.x = q_update.x();
        temp_pose.pose.orientation.y = q_update.y();
        temp_pose.pose.orientation.z = q_update.z();
        temp_pose.pose.orientation.w = q_update.w();
        slam_pose_pub.publish(temp_pose);

    }
    if(start_walk && save_data && ((save_count%10)==0)){
        ofstream fout_gt("/home/jack/bipedv5/output/base_gt.txt", ios::app);
        fout_gt.setf(ios::fixed, ios::floatfield);
        fout_gt.precision(10);
        fout_gt << ground_truth_odom_pub_msg.header.stamp.toSec() << " ";
        fout_gt.precision(5);
        fout_gt << gt_odom(0) << " "
                << gt_odom(1) << " "
                <<  gt_odom(2) << " "
                << gt_odomq.x() << " "
                << gt_odomq.y() << " "
                << gt_odomq.z() << " "
                << gt_odomq.w() << endl;
        fout_gt.close();

        ofstream fout_ekf("/home/jack/bipedv5/output/base_ekf.txt", ios::app);
        fout_ekf.setf(ios::fixed, ios::floatfield);
        fout_ekf.precision(10);
        fout_ekf << ground_truth_odom_pub_msg.header.stamp.toSec() << " ";
        fout_ekf.precision(5);
        fout_ekf <<  baseEKF->rX << " "
                <<  baseEKF->rY << " "
                <<   baseEKF->rZ << " "
                <<  baseEKF->qib.x() << " "
                << baseEKF->qib.y() << " "
                << baseEKF->qib.z()  << " "
                << baseEKF->qib.w()  << endl;
        fout_ekf.close();

        ofstream fout_slam("/home/jack/bipedv5/output/base_orbslam.txt", ios::app);
        fout_slam.setf(ios::fixed, ios::floatfield);
        fout_slam.precision(10);
        fout_slam << ground_truth_odom_pub_msg.header.stamp.toSec() << " ";
        fout_slam.precision(5);
        fout_slam <<  pos_update(0) << " "
                <<  pos_update(1) << " "
                <<   pos_update(2) << " "
                << q_update.x() << " "
                << q_update.y() << " "
                << q_update.z()  << " "
                << q_update.w()  << endl;
        fout_slam.close();
        
    }
    save_count ++;
}


void multisensor_fusion::publishLegEstimates()
{
    nav_msgs::Odometry tmp_LLeg_odom_msg;
    tmp_LLeg_odom_msg.child_frame_id = lfoot_frame;
    tmp_LLeg_odom_msg.header.stamp = ros::Time::now();
    tmp_LLeg_odom_msg.header.frame_id = "world";
    tmp_LLeg_odom_msg.pose.pose.position.x = Twl.translation()(0);
    tmp_LLeg_odom_msg.pose.pose.position.y = Twl.translation()(1);
    tmp_LLeg_odom_msg.pose.pose.position.z = Twl.translation()(2);
    tmp_LLeg_odom_msg.pose.pose.orientation.x = qwl.x();
    tmp_LLeg_odom_msg.pose.pose.orientation.y = qwl.y();
    tmp_LLeg_odom_msg.pose.pose.orientation.z = qwl.z();
    tmp_LLeg_odom_msg.pose.pose.orientation.w = qwl.w();
    tmp_LLeg_odom_msg.twist.twist.linear.x = vwl(0);
    tmp_LLeg_odom_msg.twist.twist.linear.y = vwl(1);
    tmp_LLeg_odom_msg.twist.twist.linear.z = vwl(2);
    tmp_LLeg_odom_msg.twist.twist.angular.x = omegawl(0);
    tmp_LLeg_odom_msg.twist.twist.angular.y = omegawl(1);
    tmp_LLeg_odom_msg.twist.twist.angular.z = omegawl(2);
    LLegOdom_pub.publish(tmp_LLeg_odom_msg);

    nav_msgs::Odometry tmp_RLeg_odom_msg;
    tmp_RLeg_odom_msg.child_frame_id = rfoot_frame;
    tmp_RLeg_odom_msg.header.stamp = ros::Time::now();
    tmp_RLeg_odom_msg.header.frame_id = "world";
    tmp_RLeg_odom_msg.pose.pose.position.x = Twr.translation()(0);
    tmp_RLeg_odom_msg.pose.pose.position.y = Twr.translation()(1);
    tmp_RLeg_odom_msg.pose.pose.position.z = Twr.translation()(2);
    tmp_RLeg_odom_msg.pose.pose.orientation.x = qwr.x();
    tmp_RLeg_odom_msg.pose.pose.orientation.y = qwr.y();
    tmp_RLeg_odom_msg.pose.pose.orientation.z = qwr.z();
    tmp_RLeg_odom_msg.pose.pose.orientation.w = qwr.w();
    tmp_RLeg_odom_msg.twist.twist.linear.x = vwr(0);
    tmp_RLeg_odom_msg.twist.twist.linear.y = vwr(1);
    tmp_RLeg_odom_msg.twist.twist.linear.z = vwr(2);
    tmp_RLeg_odom_msg.twist.twist.angular.x = omegawr(0);
    tmp_RLeg_odom_msg.twist.twist.angular.y = omegawr(1);
    tmp_RLeg_odom_msg.twist.twist.angular.z = omegawr(2);
    RLegOdom_pub.publish(tmp_RLeg_odom_msg);

    if (debug_mode)
    {
        geometry_msgs::PoseStamped tmp_LLeg_pose_msg;
        tmp_LLeg_pose_msg.pose.position.x = Tbl.translation()(0);
        tmp_LLeg_pose_msg.pose.position.y = Tbl.translation()(1);
        tmp_LLeg_pose_msg.pose.position.z = Tbl.translation()(2);
        tmp_LLeg_pose_msg.pose.orientation.x = qbl.x();
        tmp_LLeg_pose_msg.pose.orientation.y = qbl.y();
        tmp_LLeg_pose_msg.pose.orientation.z = qbl.z();
        tmp_LLeg_pose_msg.pose.orientation.w = qbl.w();
        tmp_LLeg_pose_msg.header.stamp = ros::Time::now();
        tmp_LLeg_pose_msg.header.frame_id = base_link_frame;
        rel_LLegPose_pub.publish(tmp_LLeg_pose_msg);

        geometry_msgs::PoseStamped tmp_RLeg_pose_msg;
        tmp_RLeg_pose_msg.pose.position.x = Tbr.translation()(0);
        tmp_RLeg_pose_msg.pose.position.y = Tbr.translation()(1);
        tmp_RLeg_pose_msg.pose.position.z = Tbr.translation()(2);
        tmp_RLeg_pose_msg.pose.orientation.x = qbr.x();
        tmp_RLeg_pose_msg.pose.orientation.y = qbr.y();
        tmp_RLeg_pose_msg.pose.orientation.z = qbr.z();
        tmp_RLeg_pose_msg.pose.orientation.w = qbr.w();
        tmp_RLeg_pose_msg.header.stamp = ros::Time::now();
        tmp_RLeg_pose_msg.header.frame_id = base_link_frame;
        rel_RLegPose_pub.publish(tmp_RLeg_pose_msg);
    }
}

void multisensor_fusion::publishJointEstimates()
{

    sensor_msgs::JointState tmp_joint_msg;
    tmp_joint_msg.header.stamp = ros::Time::now();
    tmp_joint_msg.name.resize(number_of_joints);
    tmp_joint_msg.position.resize(number_of_joints);
    tmp_joint_msg.velocity.resize(number_of_joints);

    for (unsigned int i = 0; i < number_of_joints; i++)
    {
        tmp_joint_msg.position[i] = JointVF[i]->JointPosition;
        tmp_joint_msg.velocity[i] = JointVF[i]->JointVelocity;
        tmp_joint_msg.name[i] = JointVF[i]->JointName;
    }

    joint_pub.publish(tmp_joint_msg);
}




void multisensor_fusion::publishCOP()
{
    geometry_msgs::PointStamped tmp_point_msg;
    tmp_point_msg.point.x = COP_fsr(0);
    tmp_point_msg.point.y = COP_fsr(1);
    tmp_point_msg.point.z = COP_fsr(2);
    tmp_point_msg.header.stamp = ros::Time::now();
    tmp_point_msg.header.frame_id = "world";
    COP_pub.publish(tmp_point_msg);
}

void multisensor_fusion::publishCoMEstimates()
{

    nav_msgs::Odometry tmp_CoM_odom_msg;
    tmp_CoM_odom_msg.child_frame_id = "CoM";
    tmp_CoM_odom_msg.header.stamp = ros::Time::now();
    tmp_CoM_odom_msg.header.frame_id = "world";
    if(useCoMEKF){
        tmp_CoM_odom_msg.pose.pose.position.x = nipmEKF->comX;
        tmp_CoM_odom_msg.pose.pose.position.y = nipmEKF->comY;
        tmp_CoM_odom_msg.pose.pose.position.z = nipmEKF->comZ;
        tmp_CoM_odom_msg.twist.twist.linear.x = nipmEKF->velX;
        tmp_CoM_odom_msg.twist.twist.linear.y = nipmEKF->velY;
        tmp_CoM_odom_msg.twist.twist.linear.z = nipmEKF->velZ;
    }else{
        tmp_CoM_odom_msg.pose.pose.position.x = comLKF->comX;
        tmp_CoM_odom_msg.pose.pose.position.y = comLKF->comY;
        tmp_CoM_odom_msg.pose.pose.position.z = comLKF->comZ;
        tmp_CoM_odom_msg.twist.twist.linear.x = comLKF->velX;
        tmp_CoM_odom_msg.twist.twist.linear.y = comLKF->velY;
        tmp_CoM_odom_msg.twist.twist.linear.z = comLKF->velZ;
    }
    //for(int i=0;i<36;i++)
    //odom_est_msg.pose.covariance[i] = 0;
    CoMOdom_pub.publish(tmp_CoM_odom_msg);

    geometry_msgs::WrenchStamped tmp_wrench_msg;
    tmp_wrench_msg.header.frame_id = "world";
    tmp_wrench_msg.header.stamp = ros::Time::now();
    if(useCoMEKF){
        tmp_wrench_msg.wrench.force.x = nipmEKF->fX;
        tmp_wrench_msg.wrench.force.y = nipmEKF->fY;
        tmp_wrench_msg.wrench.force.z = nipmEKF->fZ;
    }else{
        tmp_wrench_msg.wrench.force.x = comLKF->fX;
        tmp_wrench_msg.wrench.force.y = comLKF->fY;
        tmp_wrench_msg.wrench.force.z = comLKF->fZ;
    }
    ExternalWrench_pub.publish(tmp_wrench_msg);
    if (debug_mode)
    {
        geometry_msgs::PoseStamped tmp_pose_msg;
        tmp_pose_msg.pose.position.x = CoM_enc(0);
        tmp_pose_msg.pose.position.y = CoM_enc(1);
        tmp_pose_msg.pose.position.z = CoM_enc(2);
        tmp_pose_msg.header.stamp = ros::Time::now();
        tmp_pose_msg.header.frame_id = base_link_frame;
        rel_CoMPose_pub.publish(tmp_pose_msg);
    }
    if(rviz_view){
        //发布融合滤波估计CoM
        if(com_path_msg.poses.size()>1000)
            com_path_msg.poses.clear();
        geometry_msgs::PoseStamped temp_pose_com;
        com_path_msg.header = tmp_CoM_odom_msg.header;
        temp_pose_com.header = tmp_CoM_odom_msg.header;
        temp_pose_com.pose =tmp_CoM_odom_msg.pose.pose;
        com_path_msg.poses.push_back(temp_pose_com);
        com_path_pub.publish(com_path_msg);
        geometry_msgs::PointStamped temp_point_com;
        temp_point_com.header = temp_pose_com.header;
        temp_point_com.point = temp_pose_com.pose.position;
        com_point_pub.publish(temp_point_com);
        //发布运动学计算CoM
        temp_point_com.header = tmp_CoM_odom_msg.header;
        temp_point_com.point.x = CoM_kin(0);
        temp_point_com.point.y = CoM_kin(1);
        temp_point_com.point.z = CoM_kin(2);
        com_kin_point_pub.publish(temp_point_com);
        //发布实际CoM位置
        if(ground_truth_com_path_msg.poses.size()>1000)
            ground_truth_com_path_msg.poses.clear();
        ground_truth_com_path_msg.header = ground_truth_com_odom_msg.header;
        temp_pose_com.header = ground_truth_com_odom_msg.header;
        temp_pose_com.pose = ground_truth_com_odom_msg.pose.pose;
        ground_truth_com_path_msg.poses.push_back(temp_pose_com);
        ground_truth_com_path_pub.publish(ground_truth_com_path_msg);
        temp_point_com.header = temp_pose_com.header;
        temp_point_com.point = temp_pose_com.pose.position;
        ground_truth_com_point_pub.publish(temp_point_com);
    }

     if(start_walk && save_data && ((save_count%10)==0)){
        ofstream fout_cgt("/home/jack/bipedv5/output/com_gt.txt", ios::app);
        fout_cgt.setf(ios::fixed, ios::floatfield);
        fout_cgt.precision(10);
        fout_cgt << ground_truth_com_odom_msg.header.stamp.toSec() << " ";
        fout_cgt.precision(5);
        fout_cgt << ground_truth_com_odom_msg.pose.pose.position.x  << " "
                << ground_truth_com_odom_msg.pose.pose.position.y << " "
                << ground_truth_com_odom_msg.pose.pose.position.z << " "
                << 0 << " "
                << 0 << " "
                << 0 << " "
                << 1 << endl;
        fout_cgt.close();

        ofstream fout_ckf("/home/jack/bipedv5/output/com_kf.txt", ios::app);
        fout_ckf.setf(ios::fixed, ios::floatfield);
        fout_ckf.precision(10);
        fout_ckf << ground_truth_com_odom_msg.header.stamp.toSec() << " ";
        fout_ckf.precision(5);
        fout_ckf <<  comLKF->comX << " "
                <<  comLKF->comY << " "
                <<   comLKF->comZ << " "
                <<  0 << " "
                << 0 << " "
                << 0  << " "
                << 1 << endl;
        fout_ckf.close();

        ofstream fout_ckin("/home/jack/bipedv5/output/com_kin.txt", ios::app);
        fout_ckin.setf(ios::fixed, ios::floatfield);
        fout_ckin.precision(10);
        fout_ckin <<  ground_truth_com_odom_msg.header.stamp.toSec() << " ";
        fout_ckin.precision(5);
        fout_ckin <<  CoM_kin(0) << " "
                <<  CoM_kin(1) << " "
                <<  CoM_kin(2) << " "
                << 0 << " "
                << 0 << " "
                << 0 << " "
                << 1 << endl;
        fout_ckin.close();
    }  
}

void multisensor_fusion::run()        //主循环
{
    if(calibrateIMU || calibrateFc)
        calibrate();
    filtering_thread = std::thread([this] { this->filteringThread(); });     //EKF线程
    output_thread = std::thread([this] { this->outputPublishThread(); });       //输出线程
    ros::spin();
}

void multisensor_fusion::outputPublishThread()
{

    ros::Rate rate(2.0 * freq);
    while (ros::ok())
    {

        if (!base_data_inc)      //没有数据更新
            continue;
        output_lock.lock();     //互斥量，防止同时访问共享内存数据
        //Publish Data
        if (computeJointVelocity)
            publishJointEstimates();

        publishBodyEstimates();
        publishLegEstimates();
        if(com_data_inc){
            publishCoMEstimates();
            com_data_inc = false;
        }
        publishCOP();
        base_data_inc = false;
        output_lock.unlock();

        rate.sleep();
    }
}
#include <bipedv5/sensor.h>

Eigen::Vector3d LLegGRF, RLegGRF, LLegGRT, RLegGRT,LLegForceFilt,RLegForceFilt,LLegTorqueFilt,RLegTorqueFilt;
Mediator *lmdfx,*rmdfx,*lmdfy,*rmdfy,*lmdfz,*rmdfz,*lmdtx,*rmdtx,*lmdty,*rmdty,*lmdtz,*rmdtz
,*lmcx,*lmcy,*lmcvx,*lmcvy,*lmLF,*lmRF;
double tlh ,tlk,trh,trk;
ros::Publisher LF_pub,RF_pub;


void lftCb(const geometry_msgs::WrenchStampedConstPtr&  lhmsg,const geometry_msgs::WrenchStampedConstPtr&  lkmsg)
{
    double L1 = 0.275;double L2 = 0.275;
    
    Eigen::MatrixXd Jl;Eigen::VectorXd Tl,Fl;
    Jl=Eigen::MatrixXd::Zero(2,2);
	Tl=Eigen::VectorXd::Zero(2);
    Fl=Eigen::VectorXd::Zero(2);
    Tl(0) = lhmsg->wrench.torque.y;
    Tl(1) = lkmsg->wrench.torque.y; 
    Jl(0,0) = L1*cos(tlh)+L2*cos(tlk-tlh);
	Jl(0,1) = -L2*cos(tlk-tlh);
	Jl(1,0) = L1*sin(tlh)-L2*sin(tlk-tlh);
	Jl(1,1) = L2*sin(tlk-tlh);
	Fl = Jl.transpose().inverse() * Tl;
    MediatorInsert(lmLF, Fl(1));
    Fl(1) = MediatorMedian(lmLF);
    if (Fl(1) <10)        
        Fl(1)=0;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = "world";
    wrench_msg.wrench.force.y = Fl(0);
    wrench_msg.wrench.force.z = Fl(1);
    wrench_msg.header.stamp = ros::Time::now();
    LF_pub.publish(wrench_msg);
}

void rftCb(const geometry_msgs::WrenchStampedConstPtr&  rhmsg,const geometry_msgs::WrenchStampedConstPtr&  rkmsg)
{
    double L1 = 0.275;double L2 = 0.275;
    
    Eigen::MatrixXd Jr;Eigen::VectorXd Tr,Fr;
    Jr=Eigen::MatrixXd::Zero(2,2);
	Tr=Eigen::VectorXd::Zero(2);
    Fr=Eigen::VectorXd::Zero(2);
    Tr(0) = rhmsg->wrench.torque.y;
    Tr(1) = rkmsg->wrench.torque.y;
    Jr(0,0) = L1*cos(trh)+L2*cos(trk-trh);
	Jr(0,1) = -L2*cos(trk-trh);
	Jr(1,0) = L1*sin(trh)-L2*sin(trk-trh);
	Jr(1,1) = L2*sin(trk-trh);
	Fr = Jr.transpose().inverse() * Tr;
    MediatorInsert(lmRF, Fr(1));
    Fr(1) = abs(MediatorMedian(lmRF));
    if (Fr(1) <10)        
        Fr(1)=0;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = "world";
    wrench_msg.wrench.force.y = Fr(0);
    wrench_msg.wrench.force.z = Fr(1);
    wrench_msg.header.stamp = ros::Time::now();
    RF_pub.publish(wrench_msg);
}


void ftCb(const geometry_msgs::WrenchStampedConstPtr&  lmsg,const geometry_msgs::WrenchStampedConstPtr&  rmsg)
{
    LLegGRF(0) = lmsg->wrench.force.x;        //左脚的力和力矩
    LLegGRF(1) = lmsg->wrench.force.y;
    LLegGRF(2) = lmsg->wrench.force.z;
    LLegGRT(0) = lmsg->wrench.torque.x;
    LLegGRT(1) = lmsg->wrench.torque.y;
    LLegGRT(2) = lmsg->wrench.torque.z;
    RLegGRF(0) = rmsg->wrench.force.x;        //右脚的力和力矩
    RLegGRF(1) = rmsg->wrench.force.y;
    RLegGRF(2) = rmsg->wrench.force.z;
    RLegGRT(0) = rmsg->wrench.torque.x;
    RLegGRT(1) = rmsg->wrench.torque.y;
    RLegGRT(2) = rmsg->wrench.torque.z;
    MediatorInsert(lmdfx, LLegGRF(0));
    MediatorInsert(lmdfy, LLegGRF(1));
    MediatorInsert(lmdfz, LLegGRF(2));
    MediatorInsert(lmdtx, LLegGRT(0));
    MediatorInsert(lmdty, LLegGRT(1));
    MediatorInsert(lmdtz, LLegGRT(2));
    LLegForceFilt(0) = MediatorMedian(lmdfx);
    LLegForceFilt(1) = MediatorMedian(lmdfy);
    LLegForceFilt(2) = MediatorMedian(lmdfz);
    LLegTorqueFilt(0) = MediatorMedian(lmdtx);
    LLegTorqueFilt(1) = MediatorMedian(lmdty);
    LLegTorqueFilt(2) = MediatorMedian(lmdtz);
    MediatorInsert(rmdfx, RLegGRF(0));
    MediatorInsert(rmdfy, RLegGRF(1));
    MediatorInsert(rmdfz, RLegGRF(2));
    MediatorInsert(rmdtx, RLegGRT(0));
    MediatorInsert(rmdty, RLegGRT(1));
    MediatorInsert(rmdtz, RLegGRT(2));
    RLegForceFilt(0) = MediatorMedian(rmdfx);
    RLegForceFilt(1) = MediatorMedian(rmdfy);
    RLegForceFilt(2) = MediatorMedian(rmdfz);
    RLegTorqueFilt(0) = MediatorMedian(rmdtx);
    RLegTorqueFilt(1) = MediatorMedian(rmdty);
    RLegTorqueFilt(2) = MediatorMedian(rmdtz);
    double f = lmsg->wrench.force.z + rmsg->wrench.force.z;
    // cout<<"l msg t:"<<lmsg->header.stamp<<" , r msg t : "<<rmsg->header.stamp<<endl;
    // cout<<"l msg fz:"<<LLegGRF(2)<<" , r msg fz : "<<RLegGRF(2)<<endl;
    // cout<<"l msg fz filter :"<<LLegForceFilt(2)<<" , r msg fz  filter: "<<RLegForceFilt(2)<<endl;
    // cout<<"total z force: "<<f<<" filter :"<<LLegForceFilt(2)+RLegForceFilt(2)<<endl;
}

Sensor::Sensor(){
    dt = 0.001;
    fh_=0.065;
    rviz_view = true;
    buffer_num = 1000;
    gazebo_launch=false;
    modelname = "/home/jack/bipedv5/src/bipedv5/urdf/bipedv5.urdf";
    rd = new robotDyn(modelname, true);
    joint_state_sub = n.subscribe("/bipedv5/joint_states", 1, &Sensor::joint_stateCb,this); //this为回调函数所处的类，即当前类
    model_state_sub = n.subscribe("/gazebo/model_states", 1, &Sensor::model_stateCb,this);
    link_state_sub = n.subscribe("/gazebo/link_states", 1, &Sensor::link_stateCb,this);

    vwb_.setZero(); omegawb_.setZero(); pwb_.setZero();CoM.setZero();
    //传感器话题
    js_pub = n.advertise<sensor_msgs::JointState>("/bipedv5/js", 50);
    com_pub = n.advertise<nav_msgs::Odometry>("/bipedv5/CoM", 50);
    odom_pub = n.advertise<nav_msgs::Odometry>("/bipedv5/odom", 50);
    odom_pub_LLeg = n.advertise<nav_msgs::Odometry>("/bipedv5/LLeg/odom", 50);
    odom_pub_RLeg = n.advertise<nav_msgs::Odometry>("/bipedv5/RLeg/odom", 50);
    gait_phase_pub = n.advertise<std_msgs::String>("/bipedv5/gait_phase", 50);
    zmp_pub = n.advertise<geometry_msgs::PointStamped>("/bipedv5/ZMP", 50);
    wrench_pub_LLeg = n.advertise<geometry_msgs::WrenchStamped>("/bipedv5/LLeg/force_torque_states", 50);
    wrench_pub_RLeg = n.advertise<geometry_msgs::WrenchStamped>("/bipedv5/RLeg/force_torque_states", 50);
    if (rviz_view){
        com_position_pub = n.advertise<geometry_msgs::PointStamped>("bipedv5/com/position", 50);
        base_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/base/pose", 50);
        lleg_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/lleg/pose", 50);
        rleg_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/rleg/pose", 50);
    }
    ROS_INFO("sensor init OK");
}

void Sensor::joint_stateCb(const sensor_msgs::JointState msg)
{

    joint_state_msg = msg;      //关节按照urdf文件顺序
    joint_state_msg.name[0]= "ly";
    joint_state_msg.name[1]="lh";
    joint_state_msg.name[2]="lkp";
    joint_state_msg.name[3]= "lk";
    joint_state_msg.name[4]= "lap";
    joint_state_msg.name[5]= "la";
    joint_state_msg.name[6]= "ry";
    joint_state_msg.name[7]="rh";
    joint_state_msg.name[8]="rkp";
    joint_state_msg.name[9]="rk";
    joint_state_msg.name[10]="rap";
    joint_state_msg.name[11]="ra";
    //To Do
    joint_state_msg.position[0]=msg.position[5];
    joint_state_msg.position[1]=msg.position[2];
    joint_state_msg.position[2]=msg.position[4];
    joint_state_msg.position[3]=msg.position[3];
    joint_state_msg.position[4]=msg.position[1];
    joint_state_msg.position[5]=msg.position[0];
    joint_state_msg.position[6]=msg.position[11];
    joint_state_msg.position[7]=msg.position[8];
    joint_state_msg.position[8]=msg.position[10];
    joint_state_msg.position[9]=msg.position[9];
    joint_state_msg.position[10]=msg.position[7];
    joint_state_msg.position[11]=msg.position[6];

    joint_state_msg.velocity[0]=msg.velocity[5];
    joint_state_msg.velocity[1]=msg.velocity[2];
    joint_state_msg.velocity[2]=msg.velocity[4];
    joint_state_msg.velocity[3]=msg.velocity[3];
    joint_state_msg.velocity[4]=msg.velocity[1];
    joint_state_msg.velocity[5]=msg.velocity[0];
    joint_state_msg.velocity[6]=msg.velocity[11];
    joint_state_msg.velocity[7]=msg.velocity[8];
    joint_state_msg.velocity[8]=msg.velocity[10];
    joint_state_msg.velocity[9]=msg.velocity[9];
    joint_state_msg.velocity[10]=msg.velocity[7];
    joint_state_msg.velocity[11]=msg.velocity[6];
    
    joint_state_msg.effort[0]=msg.effort[5];
    joint_state_msg.effort[1]=msg.effort[2];
    joint_state_msg.effort[2]=msg.effort[4];
    joint_state_msg.effort[3]=msg.effort[3];
    joint_state_msg.effort[4]=msg.effort[1];
    joint_state_msg.effort[5]=msg.effort[0];
    joint_state_msg.effort[6]=msg.effort[11];
    joint_state_msg.effort[7]=msg.effort[8];
    joint_state_msg.effort[8]=msg.effort[10];
    joint_state_msg.effort[9]=msg.effort[9];
    joint_state_msg.effort[10]=msg.effort[7];
    joint_state_msg.effort[11]=msg.effort[6];
    js_pub.publish(joint_state_msg);
    tlh =joint_state_msg.position[1]; tlk =joint_state_msg.position[3];
    trh =joint_state_msg.position[7]; trk =joint_state_msg.position[9];
}


void Sensor::model_stateCb(const gazebo_msgs::ModelStates msg)
{
    pwb_(0)=msg.pose[1].position.x;
    pwb_(1)=msg.pose[1].position.y;
    pwb_(2)=msg.pose[1].position.z;
    qwb_ = Eigen::Quaterniond(msg.pose[1].orientation.w, msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z);
    vwb_(0)=msg.twist[1].linear.x;
    vwb_(1)=msg.twist[1].linear.y;
    vwb_(2)=msg.twist[1].linear.z;
    omegawb_(0)=msg.twist[1].angular.x;
    omegawb_(1)=msg.twist[1].angular.y;
    omegawb_(2)=msg.twist[1].angular.z;
}

//其实就是关节原点位置
void Sensor::link_stateCb(const gazebo_msgs::LinkStates msg) //gazebo/link_state获取的是urdf中关节的位置
{
    link_state_msg=msg;
    if(!gazebo_launch)
        gazebo_launch = true;
}


void Sensor::pubSensorInfo(){
    //计算并发布质心位置
    rd->setBaseToWorldState(pwb_, qwb_);
    rd->setBaseWorldVelocity(vwb_, omegawb_);
    rd->updateJointConfig(joint_state_msg.name,joint_state_msg.position,joint_state_msg.velocity);
    CoM = rd->comPosition();
    nav_msgs::Odometry com_msg;
    com_msg.header.frame_id = "world";
    com_msg.child_frame_id = "CoM";
    com_msg.pose.pose.position.x = CoM(0);
    com_msg.pose.pose.position.y = CoM(1);
    com_msg.pose.pose.position.z = CoM(2);
    MediatorInsert(lmcx, (CoM(0)-CoM_last(0))/dt);
    MediatorInsert(lmcy, (CoM(1)-CoM_last(1))/dt);
    vCoM(0) = MediatorMedian(lmcx);
    vCoM(1) = MediatorMedian(lmcy);
    vCoM(2) = 0;
    com_msg.twist.twist.linear.x =  vCoM(0) ;
    com_msg.twist.twist.linear.y =  vCoM(1) ;
    com_msg.twist.twist.linear.z = vCoM(2) ;
    MediatorInsert(lmcvx, (vCoM(0)-vCoM_last(0))/dt);
    MediatorInsert(lmcvy, (vCoM(1)-vCoM_last(1))/dt);
    aCoM(0) = MediatorMedian(lmcvx);
    aCoM(1) = MediatorMedian(lmcvy);
    aCoM(2) = 0;
    //加速度
    com_msg.twist.twist.angular.x =  aCoM(0) ;
    com_msg.twist.twist.angular.y =  aCoM(1) ;
    com_msg.twist.twist.angular.z = aCoM(2) ;
    com_msg.header.stamp = ros::Time::now();
    com_pub.publish(com_msg);
    CoM_last = CoM;
    vCoM_last = vCoM;
    //发布里程计消息
    static tf2_ros::TransformBroadcaster br;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base";
    odom_msg.pose.pose.position.x = pwb_(0);
    odom_msg.pose.pose.position.y = pwb_(1);
    odom_msg.pose.pose.position.z = pwb_(2);
    odom_msg.pose.pose.orientation.x = qwb_.x();
    odom_msg.pose.pose.orientation.y = qwb_.y();
    odom_msg.pose.pose.orientation.z = qwb_.z();
    odom_msg.pose.pose.orientation.w = qwb_.w();
    odom_msg.twist.twist.linear.x = vwb_(0);
    odom_msg.twist.twist.linear.y = vwb_(1);
    odom_msg.twist.twist.linear.z = vwb_(2);
    odom_msg.twist.twist.angular.x = omegawb_(0);
    odom_msg.twist.twist.angular.y = omegawb_(1);
    odom_msg.twist.twist.angular.z = omegawb_(2);
    odom_msg.header.stamp = ros::Time::now();
    odom_pub.publish(odom_msg);
    //发布腿部位置消息
    Eigen::Affine3d Twl,Twr;
    Twl.linear() = Quaterniond(link_state_msg.pose[7].orientation.w,link_state_msg.pose[7].orientation.x,link_state_msg.pose[7].orientation.y,link_state_msg.pose[7].orientation.z).toRotationMatrix(); 
    Twl.translation() = Vector3d( link_state_msg.pose[7].position.x, link_state_msg.pose[7].position.y, link_state_msg.pose[7].position.z);
    Twr.linear() = Quaterniond(link_state_msg.pose[13].orientation.w,link_state_msg.pose[13].orientation.x,link_state_msg.pose[13].orientation.y,link_state_msg.pose[13].orientation.z).toRotationMatrix(); 
    Twr.translation() = Vector3d( link_state_msg.pose[13].position.x, link_state_msg.pose[13].position.y, link_state_msg.pose[13].position.z);
    nav_msgs::Odometry lodom_msg;       
    lodom_msg.header.frame_id = "world";
    lodom_msg.child_frame_id = "left_foot" ;
    lodom_msg.pose.pose.position.x = link_state_msg.pose[7].position.x;
    lodom_msg.pose.pose.position.y = link_state_msg.pose[7].position.y;
    lodom_msg.pose.pose.position.z = link_state_msg.pose[7].position.z;
    lodom_msg.pose.pose.orientation.x = link_state_msg.pose[7].orientation.x;
    lodom_msg.pose.pose.orientation.y = link_state_msg.pose[7].orientation.y;
    lodom_msg.pose.pose.orientation.z = link_state_msg.pose[7].orientation.z;
    lodom_msg.pose.pose.orientation.w = link_state_msg.pose[7].orientation.w;
    lodom_msg.header.stamp = ros::Time::now();
    odom_pub_LLeg.publish(lodom_msg);
    
    nav_msgs::Odometry rodom_msg;
    rodom_msg.header.frame_id = "world";
    rodom_msg.child_frame_id = "right_foot" ;
    rodom_msg.pose.pose.position.x = link_state_msg.pose[13].position.x;
    rodom_msg.pose.pose.position.y = link_state_msg.pose[13].position.y;
    rodom_msg.pose.pose.position.z = link_state_msg.pose[13].position.z;
    rodom_msg.pose.pose.orientation.x = link_state_msg.pose[13].orientation.x;
    rodom_msg.pose.pose.orientation.y = link_state_msg.pose[13].orientation.y;
    rodom_msg.pose.pose.orientation.z = link_state_msg.pose[13].orientation.z;
    rodom_msg.pose.pose.orientation.w = link_state_msg.pose[13].orientation.w;
    rodom_msg.header.stamp = ros::Time::now();
    odom_pub_RLeg.publish(rodom_msg);
    //判断并发布步态周期
    std_msgs::String gait_msg;
    if (LLegForceFilt(2)>10 && RLegForceFilt(2)>10){
        gait_msg.data = "double_support";
    }else if (LLegForceFilt(2)>10){
        gait_msg.data = "left_support";
    }else if (RLegForceFilt(2)>10){
        gait_msg.data = "right_support";
    }else{
        gait_msg.data = "flight";
    }
    gait_phase_pub.publish(gait_msg);
    //计算CoP，ZMP
    double weightl,weightr;
    Eigen::Vector3d copl,copr, coplw, coprw ,ZMP;          
    if (LLegForceFilt(2) > 10)        //触地状态
    {
        copl(0) = (-LLegTorqueFilt(1) - LLegForceFilt(0)*fh_ )/ LLegForceFilt(2);
        copl(1) = (LLegTorqueFilt(0)- LLegForceFilt(1)*fh_)/ LLegForceFilt(2);
        copl(2) = -fh_;
        weightl = LLegForceFilt(2);       //用于计算压力中心左脚贡献的比例
    }else{  //悬空时置0
        copl = Vector3d::Zero();
        LLegGRF = Vector3d::Zero();
        LLegGRT = Vector3d::Zero();
        LLegForceFilt = Vector3d::Zero();
        LLegTorqueFilt = Vector3d::Zero();
        weightl = 0.0;
    }
    if (RLegForceFilt(2) > 10)        //触地状态
    {
        copr(0) = (-RLegTorqueFilt(1) - RLegForceFilt(0)*fh_ )/ RLegForceFilt(2);
        copr(1) = (RLegTorqueFilt(0)- RLegForceFilt(1)*fh_)/ RLegForceFilt(2);
        copr(2) = -fh_;
        weightr = RLegForceFilt(2);       //用于计算压力中心左脚贡献的比例
    }else{
        copr = Vector3d::Zero();
        RLegGRF = Vector3d::Zero();
        RLegGRT = Vector3d::Zero();
        RLegForceFilt = Vector3d::Zero();
        RLegTorqueFilt = Vector3d::Zero();
        weightr = 0.0;
    }
    coplw = Twl * copl;
    coprw = Twr * copr;

    if (weightl + weightr > 0.0)
    {
        ZMP = (weightl * coplw + weightr * coprw) / (weightl + weightr);       
    }
    else
    {
        ZMP = Vector3d::Zero();
    }
    //发送ZMP消息
    geometry_msgs::PointStamped zmp_msg;
    zmp_msg.header.frame_id = "world";
    zmp_msg.point.x = ZMP(0);
    zmp_msg.point.y = ZMP(1);
    zmp_msg.point.z = ZMP(2);    //(link_state_msg.pose[7].position.z+link_state_msg.pose[13].position.z)/2;
    zmp_msg.header.stamp = ros::Time::now();
    zmp_pub.publish(zmp_msg);
    //发送力消息
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = "world";
    wrench_msg.wrench.force.x = LLegForceFilt(0);
    wrench_msg.wrench.force.y = LLegForceFilt(1);
    wrench_msg.wrench.force.z = LLegForceFilt(2);
    wrench_msg.wrench.torque.x = LLegTorqueFilt(0);
    wrench_msg.wrench.torque.y = LLegTorqueFilt(1);
    wrench_msg.wrench.torque.z = LLegTorqueFilt(2);
    wrench_msg.header.stamp = ros::Time::now();
    wrench_pub_LLeg.publish(wrench_msg);
    wrench_msg.wrench.force.x = RLegForceFilt(0);
    wrench_msg.wrench.force.y = RLegForceFilt(1);
    wrench_msg.wrench.force.z = RLegForceFilt(2);
    wrench_msg.wrench.torque.x = RLegTorqueFilt(0);
    wrench_msg.wrench.torque.y = RLegTorqueFilt(1);
    wrench_msg.wrench.torque.z = RLegTorqueFilt(2);
    wrench_msg.header.stamp = ros::Time::now();
    wrench_pub_RLeg.publish(wrench_msg);
    if(rviz_view){
    //发布tf变换
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = pwb_(0);
        transformStamped.transform.translation.y = pwb_(1);
        transformStamped.transform.translation.z = pwb_(2);
        transformStamped.transform.rotation.x = qwb_.x();
        transformStamped.transform.rotation.y = qwb_.y();
        transformStamped.transform.rotation.z = qwb_.z();
        transformStamped.transform.rotation.w = qwb_.w();
        br.sendTransform(transformStamped);
        //CoM
        geometry_msgs::PointStamped temp_com;
        temp_com.header.stamp  = ros::Time::now();
        temp_com.header.frame_id = "world";
        temp_com.point =com_msg.pose.pose.position;
        com_position_pub.publish(temp_com);
        //base pose
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = odom_msg.header;
        temp_pose.pose = odom_msg.pose.pose;
        base_pose_pub.publish(temp_pose);
        //lleg
        temp_pose.header = lodom_msg.header;
        temp_pose.pose = lodom_msg.pose.pose;
        lleg_pose_pub.publish(temp_pose);
        //rleg
        temp_pose.header = rodom_msg.header;
        temp_pose.pose = rodom_msg.pose.pose;
        rleg_pose_pub.publish(temp_pose);
    }

}



int main(int argc, char  *argv[])
{   
    ros::init(argc,argv,"sensor");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    Sensor sensor;
    lmdfx = MediatorNew(50);
    rmdfx = MediatorNew(50);
    lmdfy = MediatorNew(50);
    rmdfy = MediatorNew(50);
    lmdfz = MediatorNew(50);
    rmdfz = MediatorNew(50);
    lmdtx = MediatorNew(50);
    rmdtx = MediatorNew(50);
    lmdty = MediatorNew(50);
    rmdty = MediatorNew(50);
    lmdtz = MediatorNew(50);
    rmdtz = MediatorNew(50);
    lmcx = MediatorNew(150);
    lmcy = MediatorNew(150);
    lmcvx = MediatorNew(200);
    lmcvy = MediatorNew(200);
    lmLF = MediatorNew(50);
    lmRF = MediatorNew(50);
    //力矩传感器同步
    message_filters::Subscriber<geometry_msgs::WrenchStamped> left_ft_sub(nh,"/bipedv5/left_foot_ft", 1);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> right_ft_sub(nh,"/bipedv5/right_foot_ft", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped,geometry_msgs::WrenchStamped> SyncPolicy; 
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), left_ft_sub, right_ft_sub);
    sync.registerCallback(boost::bind(&ftCb,_1,_2));
    
    message_filters::Subscriber<geometry_msgs::WrenchStamped> lk_ft_sub(nh,"/bipedv5/lk_ft", 1);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> lh_ft_sub(nh,"/bipedv5/lh_ft", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped,geometry_msgs::WrenchStamped> SyncPolicyL; 
    message_filters::Synchronizer<SyncPolicy> syncL(SyncPolicyL(10), lh_ft_sub, lk_ft_sub);
    syncL.registerCallback(boost::bind(&lftCb,_1,_2));

    message_filters::Subscriber<geometry_msgs::WrenchStamped> rk_ft_sub(nh,"/bipedv5/rk_ft", 1);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> rh_ft_sub(nh,"/bipedv5/rh_ft", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped,geometry_msgs::WrenchStamped> SyncPolicyR; 
    message_filters::Synchronizer<SyncPolicy> syncR(SyncPolicyR(10), rh_ft_sub, rk_ft_sub);
    syncR.registerCallback(boost::bind(&rftCb,_1,_2));
    LF_pub= nh.advertise<geometry_msgs::WrenchStamped>("/bipedv5/LF", 50);
    RF_pub= nh.advertise<geometry_msgs::WrenchStamped>("/bipedv5/RF", 50);
    while(ros::ok()){
        if(sensor.gazebo_launch){
            sensor.pubSensorInfo();
        }       
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}



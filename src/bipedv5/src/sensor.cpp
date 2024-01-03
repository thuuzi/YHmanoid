#include <bipedv5/sensor.h>
#include<gazebo_msgs/ContactsState.h>
Eigen::Vector3d LLegGRF, RLegGRF, LLegGRT, RLegGRT,LLegForceFilt,RLegForceFilt,LLegTorqueFilt,RLegTorqueFilt;
Mediator *lmcx,*lmcy,*lmcvx,*lmcvy,
*lmfx,*rmfx,*lmfy,*rmfy,*lmfz,*rmfz,*lmtx,*rmtx,*lmty,*rmty,*lmtz,*rmtz;
Mediator** imuMediator;
double tlh ,tlk,trh,trk;
ros::Publisher wrench_pub_RLeg,wrench_pub_LLeg,imu_filter_pub;
void contactCb(const gazebo_msgs::ContactsStateConstPtr& lmsg, const gazebo_msgs::ContactsStateConstPtr& rmsg){
    if(lmsg->states.size()==0){
        LLegGRF(0) = 0;
        LLegGRF(1) = 0;
        LLegGRF(2) = 0;
        LLegGRT(0) = 0;
        LLegGRT(1) = 0;
        LLegGRT(2) = 0;
    }else{
        double tempfx = 0;double tempfy = 0;double tempfz = 0;double temptx = 0;double tempty = 0;double temptz = 0;
        for(int i = 0; i <lmsg->states.size() ; i++){
            tempfx += lmsg->states[i].total_wrench.force.x;
            tempfy += lmsg->states[i].total_wrench.force.y;
            tempfz += lmsg->states[i].total_wrench.force.z;
            temptx += lmsg->states[i].total_wrench.torque.x;
            tempty += lmsg->states[i].total_wrench.torque.y;
            temptz += lmsg->states[i].total_wrench.torque.z;
        }
        LLegGRF(0) = tempfx/lmsg->states.size();
        LLegGRF(1) = tempfy/lmsg->states.size();
        LLegGRF(2) = tempfz/lmsg->states.size();
        LLegGRT(0) = temptx/lmsg->states.size();
        LLegGRT(1) = tempty/lmsg->states.size();
        LLegGRT(2) = temptz/lmsg->states.size();
    }
    if(rmsg->states.size()==0){
        RLegGRF(0) = 0;
        RLegGRF(1) = 0;
        RLegGRF(2) = 0;
        RLegGRT(0) = 0;
        RLegGRT(1) = 0;
        RLegGRT(2) = 0;
    }else{
        double tempfx = 0;double tempfy = 0;double tempfz = 0;double temptx = 0;double tempty = 0;double temptz = 0;
        for(int i = 0; i <rmsg->states.size() ; i++){
            tempfx += rmsg->states[i].total_wrench.force.x;
            tempfy += rmsg->states[i].total_wrench.force.y;
            tempfz += rmsg->states[i].total_wrench.force.z;
            temptx += rmsg->states[i].total_wrench.torque.x;
            tempty += rmsg->states[i].total_wrench.torque.y;
            temptz += rmsg->states[i].total_wrench.torque.z;
        }
        RLegGRF(0)= tempfx/rmsg->states.size();
        RLegGRF(1)= tempfy/rmsg->states.size();
        RLegGRF(2) = tempfz/rmsg->states.size();
        RLegGRT(0) = temptx/rmsg->states.size();
        RLegGRT(1) = tempty/rmsg->states.size();
        RLegGRT(2) = temptz/rmsg->states.size();
    }
    MediatorInsert(lmfx, LLegGRF(0));
    MediatorInsert(lmfy, LLegGRF(1));
    MediatorInsert(lmfz, LLegGRF(2));
    MediatorInsert(lmtx, LLegGRT(0));
    MediatorInsert(lmty, LLegGRT(1));
    MediatorInsert(lmtz, LLegGRT(2));
    LLegForceFilt(0) = MediatorMedian(lmfx);
    LLegForceFilt(1) = MediatorMedian(lmfy);
    LLegForceFilt(2) = MediatorMedian(lmfz);
    LLegTorqueFilt(0) = MediatorMedian(lmtx);
    LLegTorqueFilt(1) = MediatorMedian(lmty);
    LLegTorqueFilt(2) = MediatorMedian(lmtz);
    MediatorInsert(rmfx, RLegGRF(0));
    MediatorInsert(rmfy, RLegGRF(1));
    MediatorInsert(rmfz, RLegGRF(2));
    MediatorInsert(rmtx, RLegGRT(0));
    MediatorInsert(rmty, RLegGRT(1));
    MediatorInsert(rmtz, RLegGRT(2));
    RLegForceFilt(0) = MediatorMedian(rmfx);
    RLegForceFilt(1) = MediatorMedian(rmfy);
    RLegForceFilt(2) = MediatorMedian(rmfz);
    RLegTorqueFilt(0) = MediatorMedian(rmtx);
    RLegTorqueFilt(1) = MediatorMedian(rmty);
    RLegTorqueFilt(2) = MediatorMedian(rmtz);

    //发送力消息
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = "world";
    wrench_msg.wrench.force.x = LLegForceFilt(0);
    wrench_msg.wrench.force.y = LLegForceFilt(1);
    wrench_msg.wrench.force.z = LLegForceFilt(2);
    wrench_msg.wrench.torque.x = LLegTorqueFilt(0);
    wrench_msg.wrench.torque.y = LLegTorqueFilt(1);
    wrench_msg.wrench.torque.z = LLegTorqueFilt(2);
    wrench_msg.header.stamp = lmsg->header.stamp;
    wrench_pub_LLeg.publish(wrench_msg);
    wrench_msg.wrench.force.x = RLegForceFilt(0);
    wrench_msg.wrench.force.y = RLegForceFilt(1);
    wrench_msg.wrench.force.z = RLegForceFilt(2);
    wrench_msg.wrench.torque.x = RLegTorqueFilt(0);
    wrench_msg.wrench.torque.y = RLegTorqueFilt(1);
    wrench_msg.wrench.torque.z = RLegTorqueFilt(2);
    wrench_pub_RLeg.publish(wrench_msg);
}

Sensor::Sensor(){
    bipedv5_ini =false; 
    pub_tf = true;
    robot_ini_y = -3.0;
    robot_ini_z = 0.189823;    //脚掌在gazebo世界坐标系的高度
    dt = 0.001;
    fh_=0.065;
    HX =0.055; 
    HY =0.02;
    rviz_view = true;
    r_id = 0;
    buffer_num = 1000;
    gazebo_launch=false;
    modelname = "/home/jack/bipedv5/src/bipedv5/xacro/robot.xacro";
    rd = new robotDyn(modelname, true);
    joint_state_sub = n.subscribe("/bipedv5/joint_states", 1000, &Sensor::joint_stateCb,this); //this为回调函数所处的类，即当前类
    model_state_sub = n.subscribe("/gazebo/model_states", 1000, &Sensor::model_stateCb,this);
    link_state_sub = n.subscribe("/gazebo/link_states", 1000, &Sensor::link_stateCb,this);
    bipedv5_sub = n.subscribe("/bipedv5/ini_state", 1000, &Sensor::bipedv5IniCb,this);
    vwb_.setZero(); omegawb_.setZero(); pwb_.setZero();CoM.setZero();
    //传感器话题
    js_pub = n.advertise<sensor_msgs::JointState>("/bipedv5/js", 5000);
    com_pub = n.advertise<nav_msgs::Odometry>("/bipedv5/CoM", 5000);
    odom_pub = n.advertise<nav_msgs::Odometry>("/bipedv5/odom", 5000);
    odom_pub_LLeg = n.advertise<nav_msgs::Odometry>("/bipedv5/LLeg/odom", 5000);
    odom_pub_RLeg = n.advertise<nav_msgs::Odometry>("/bipedv5/RLeg/odom", 5000);
    gait_phase_pub = n.advertise<std_msgs::String>("/bipedv5/gait_phase", 5000);
    zmp_pub = n.advertise<geometry_msgs::PointStamped>("/bipedv5/ZMP", 5000);
    
    if (rviz_view){
        com_position_pub = n.advertise<geometry_msgs::PointStamped>("bipedv5/com/position", 5000);
        base_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/base/pose", 5000);
        lleg_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/lleg/pose", 5000);
        rleg_pose_pub = n.advertise<geometry_msgs::PoseStamped>("bipedv5/rleg/pose", 5000);
    }
    ROS_INFO("sensor init OK");
}

void Sensor::bipedv5IniCb(std_msgs::Bool msg){
    bipedv5_ini = msg.data;
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
    pwb_(0)=msg.pose[13].position.x;
    pwb_(1)=msg.pose[13].position.y;
    pwb_(2)=msg.pose[13].position.z;
    qwb_ = Eigen::Quaterniond(msg.pose[13].orientation.w, msg.pose[13].orientation.x, msg.pose[13].orientation.y, msg.pose[13].orientation.z);
    vwb_(0)=msg.twist[13].linear.x;
    vwb_(1)=msg.twist[13].linear.y;
    vwb_(2)=msg.twist[13].linear.z;
    omegawb_(0)=msg.twist[13].angular.x;
    omegawb_(1)=msg.twist[13].angular.y;
    omegawb_(2)=msg.twist[13].angular.z;
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
    com_msg.pose.pose.position.y = CoM(1)-robot_ini_y;;
    com_msg.pose.pose.position.z = CoM(2)-robot_ini_z ;
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
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base";
    odom_msg.pose.pose.position.x = pwb_(0);
    odom_msg.pose.pose.position.y = pwb_(1)-robot_ini_y;
    odom_msg.pose.pose.position.z = pwb_(2)-robot_ini_z ;
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
    Twl.linear() = Quaterniond(link_state_msg.pose[21].orientation.w,link_state_msg.pose[21].orientation.x,link_state_msg.pose[21].orientation.y,link_state_msg.pose[21].orientation.z).toRotationMatrix(); 
    Twl.translation() = Vector3d( link_state_msg.pose[21].position.x, link_state_msg.pose[21].position.y-robot_ini_y, link_state_msg.pose[21].position.z-robot_ini_z);
    Twr.linear() = Quaterniond(link_state_msg.pose[27].orientation.w,link_state_msg.pose[27].orientation.x,link_state_msg.pose[27].orientation.y,link_state_msg.pose[27].orientation.z).toRotationMatrix(); 
    Twr.translation() = Vector3d( link_state_msg.pose[27].position.x, link_state_msg.pose[27].position.y-robot_ini_y, link_state_msg.pose[27].position.z-robot_ini_z);
    nav_msgs::Odometry lodom_msg;       
    lodom_msg.header.frame_id = "world";
    lodom_msg.child_frame_id = "left_foot" ;
    lodom_msg.pose.pose.position.x = link_state_msg.pose[21].position.x;
    lodom_msg.pose.pose.position.y = link_state_msg.pose[21].position.y-robot_ini_y;
    lodom_msg.pose.pose.position.z = link_state_msg.pose[21].position.z-robot_ini_z;
    lodom_msg.pose.pose.orientation.x = link_state_msg.pose[21].orientation.x;
    lodom_msg.pose.pose.orientation.y = link_state_msg.pose[21].orientation.y;
    lodom_msg.pose.pose.orientation.z = link_state_msg.pose[21].orientation.z;
    lodom_msg.pose.pose.orientation.w = link_state_msg.pose[21].orientation.w;
    lodom_msg.header.stamp = ros::Time::now();
    odom_pub_LLeg.publish(lodom_msg);
    
    nav_msgs::Odometry rodom_msg;
    rodom_msg.header.frame_id = "world";
    rodom_msg.child_frame_id = "right_foot" ;
    rodom_msg.pose.pose.position.x = link_state_msg.pose[27].position.x;
    rodom_msg.pose.pose.position.y = link_state_msg.pose[27].position.y-robot_ini_y;
    rodom_msg.pose.pose.position.z = link_state_msg.pose[27].position.z-robot_ini_z;
    rodom_msg.pose.pose.orientation.x = link_state_msg.pose[27].orientation.x;
    rodom_msg.pose.pose.orientation.y = link_state_msg.pose[27].orientation.y;
    rodom_msg.pose.pose.orientation.z = link_state_msg.pose[27].orientation.z;
    rodom_msg.pose.pose.orientation.w = link_state_msg.pose[27].orientation.w;
    rodom_msg.header.stamp = ros::Time::now();
    odom_pub_RLeg.publish(rodom_msg);
    //判断并发布步态周期
    std_msgs::String gait_msg;
    if (LLegForceFilt(2)>0.1 && RLegForceFilt(2)>0.1){
        gait_msg.data = "double_support";
    }else if (LLegForceFilt(2)>0.1){
        gait_msg.data = "left_support";
    }else if (RLegForceFilt(2)>0.1){
        gait_msg.data = "right_support";
    }else{
        gait_msg.data = "flight";
    }
    gait_phase_pub.publish(gait_msg);
    //计算CoP，ZMP
    double weightl,weightr;
    Eigen::Vector3d copl,copr, coplw, coprw ,ZMP;          
    if (LLegForceFilt(2) > 0.1)        //触地状态
    {
        copl(0) = (-LLegTorqueFilt(1) - LLegForceFilt(0)*fh_ )/ LLegForceFilt(2) + HX;
        copl(1) = (LLegTorqueFilt(0)- LLegForceFilt(1)*fh_)/ LLegForceFilt(2) + HY;
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
    if (RLegForceFilt(2) > 0.1)        //触地状态
    {
        copr(0) = (-RLegTorqueFilt(1) - RLegForceFilt(0)*fh_ )/ RLegForceFilt(2) - HX;
        copr(1) = (RLegTorqueFilt(0)- RLegForceFilt(1)*fh_)/ RLegForceFilt(2) + HY;
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
    r_id ++;
    if(rviz_view && r_id>=10){
        r_id = 0;
        //发布tf变换，初始时刻世界坐标系到基座
        if(pub_tf && bipedv5_ini){
            static tf2_ros::StaticTransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "base_link_ini";
            transformStamped.transform.translation.x = pwb_(0);
            transformStamped.transform.translation.y = pwb_(1)-robot_ini_y;
            transformStamped.transform.translation.z = pwb_(2)-robot_ini_z;
            transformStamped.transform.rotation.x = qwb_.x();
            transformStamped.transform.rotation.y = qwb_.y();
            transformStamped.transform.rotation.z = qwb_.z();
            transformStamped.transform.rotation.w = qwb_.w();
            br.sendTransform(transformStamped);
            pub_tf = false;
        }
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

void imuCb(const sensor_msgs::Imu msg){
    sensor_msgs::Imu msg_filter;
    msg_filter = msg;
    MediatorInsert(imuMediator[0], msg.linear_acceleration.x);
    MediatorInsert(imuMediator[1], msg.linear_acceleration.y);
    MediatorInsert(imuMediator[2], msg.linear_acceleration.z);
    msg_filter.linear_acceleration.x = MediatorMedian(imuMediator[0]);
    msg_filter.linear_acceleration.y = MediatorMedian(imuMediator[1]);
    msg_filter.linear_acceleration.z = MediatorMedian(imuMediator[2]);
    MediatorInsert(imuMediator[3], msg.angular_velocity.x);
    MediatorInsert(imuMediator[4], msg.angular_velocity.y);
    MediatorInsert(imuMediator[5], msg.angular_velocity.z);
    msg_filter.angular_velocity.x = MediatorMedian(imuMediator[3]);
    msg_filter.angular_velocity.y = MediatorMedian(imuMediator[4]);
    msg_filter.angular_velocity.z = MediatorMedian(imuMediator[5]);
    imu_filter_pub.publish(msg_filter);
}

int main(int argc, char  *argv[])
{   
    ros::init(argc,argv,"sensor");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    Sensor sensor;
    lmcx = MediatorNew(20);
    lmcy = MediatorNew(30);
    lmcvx = MediatorNew(20);
    lmcvy = MediatorNew(20);
    lmfx = MediatorNew(10);
    rmfx = MediatorNew(10);
    lmfy = MediatorNew(10);
    rmfy = MediatorNew(10);
    lmfz = MediatorNew(10);
    rmfz = MediatorNew(10);
    lmtx = MediatorNew(10);
    rmtx = MediatorNew(10);
    lmty = MediatorNew(10);
    rmty = MediatorNew(10);
    lmtz = MediatorNew(10);
    rmtz = MediatorNew(10);
    message_filters::Subscriber<gazebo_msgs::ContactsState> left_contact_sub(nh,"/bipedv5/left_contact", 1000);
    message_filters::Subscriber<gazebo_msgs::ContactsState> right_contact_sub(nh,"/bipedv5/right_contact", 1000);
    typedef sync_policies::ApproximateTime<gazebo_msgs::ContactsState,gazebo_msgs::ContactsState> SyncPolicy_con; 
    message_filters::Synchronizer<SyncPolicy_con> sync_con(SyncPolicy_con(10), left_contact_sub, right_contact_sub);
    sync_con.registerCallback(boost::bind(&contactCb,_1,_2));
    wrench_pub_LLeg = nh.advertise<geometry_msgs::WrenchStamped>("/bipedv5/LLeg/force_torque_states", 5000);
    wrench_pub_RLeg = nh.advertise<geometry_msgs::WrenchStamped>("/bipedv5/RLeg/force_torque_states", 5000);
    //imu滤波
    imu_filter_pub = nh.advertise<sensor_msgs::Imu>("/bipedv5/imu_filter",1000);
    imuMediator = new Mediator *[6];
    for(int i = 0; i < 6; i++)
        imuMediator[i] = MediatorNew(8);

    ros::Subscriber imu_sub=nh.subscribe("/bipedv5/imu",1000,imuCb);
    while(ros::ok()){
        if(sensor.gazebo_launch){
            sensor.pubSensorInfo();
        }       
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}



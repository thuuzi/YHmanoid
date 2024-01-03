#! /usr/bin/env python

import rospy
import actionlib
import copy
import lipm_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from math import sin,cos,tan


NF = 70
CoM = False
ZMP = False
LLeg = False
RLeg = False
LLeg_odom = Odometry()
RLeg_odom = Odometry()
CoM_odom = Odometry()
ZMP_odom = PointStamped()
def LLegcallback(data):
    global LLeg_odom
    LLeg_odom = data
    global LLeg 
    LLeg = True

def RLegcallback(data):
    global RLeg_odom
    RLeg_odom = data
    global RLeg
    RLeg = True


def CoMcallback(data):
    global CoM_odom
    CoM_odom = data
    global CoM 
    CoM = True

def ZMPcallback(data):
    global ZMP_odom
    ZMP_odom = data
    global ZMP
    ZMP = True


def action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/bipedv5/command', lipm_msgs.msg.MotionPlanAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("Server is UP")


    # Creates a goal to send to the action server.
    goal = lipm_msgs.msg.MotionPlanGoal()
    temp =  lipm_msgs.msg.StepTarget()


    goal.lfoot.position.x = LLeg_odom.pose.pose.position.x
    goal.lfoot.position.y = LLeg_odom.pose.pose.position.y
    goal.lfoot.position.z = LLeg_odom.pose.pose.position.z
    goal.lfoot.orientation.x = LLeg_odom.pose.pose.orientation.x
    goal.lfoot.orientation.y = LLeg_odom.pose.pose.orientation.y
    goal.lfoot.orientation.z = LLeg_odom.pose.pose.orientation.z
    goal.lfoot.orientation.w = LLeg_odom.pose.pose.orientation.w
    
    goal.rfoot.position.x = RLeg_odom.pose.pose.position.x
    goal.rfoot.position.y = RLeg_odom.pose.pose.position.y
    goal.rfoot.position.z = RLeg_odom.pose.pose.position.z
    goal.rfoot.orientation.x = RLeg_odom.pose.pose.orientation.x
    goal.rfoot.orientation.y = RLeg_odom.pose.pose.orientation.y
    goal.rfoot.orientation.z = RLeg_odom.pose.pose.orientation.z
    goal.rfoot.orientation.w = RLeg_odom.pose.pose.orientation.w


    goal.CoM.pose.pose.position.x = CoM_odom.pose.pose.position.x #CoM_odom.pose.pose.position.x
    goal.CoM.pose.pose.position.y = CoM_odom.pose.pose.position.y
    goal.CoM.pose.pose.position.z = CoM_odom.pose.pose.position.z
    
    goal.COP.x =  ZMP_odom.point.x #CoM_odom.pose.pose.position.x
    goal.COP.y =  ZMP_odom.point.y #CoM_odom.pose.pose.position.y
    goal.COP.z = ZMP_odom.point.z


    goal.CoM.twist.twist.linear.x = 0 #CoM_odom.twist.twist.linear.x
    goal.CoM.twist.twist.linear.y = 0 #CoM_odom.twist.twist.linear.y
    goal.CoM.twist.twist.linear.z = 0 #CoM_odom.twist.twist.linear.z



    goal.footsteps = []

    i=1
    theta = 0.0
    lastxl= LLeg_odom.pose.pose.position.x
    lastxr =RLeg_odom.pose.pose.position.x 
    while(i<NF):
        temp.leg = 1        #1是右腿摆动
        
        if i == 1 :
            temp.pose.position.y = RLeg_odom.pose.pose.position.y + 0.05*i
        else:
            temp.pose.position.y = RLeg_odom.pose.pose.position.y + 0.1*(i-1)+0.1
            theta += 0.15
        temp.pose.position.x = lastxr - 0.1*tan(theta)
        lastxr = temp.pose.position.x
        temp.pose.position.z = RLeg_odom.pose.pose.position.z
        temp.pose.orientation.x = 0.0
        temp.pose.orientation.y = 0.0
        temp.pose.orientation.z = sin(theta/2)
        temp.pose.orientation.w = cos(theta/2)
        goal.footsteps.append(copy.deepcopy(temp))      #深层复制

        temp.leg = 0            #0是左脚摆动
        temp.pose.position.y = LLeg_odom.pose.pose.position.y + 0.1*i
        temp.pose.position.x = lastxl - 0.1*tan(theta)
        lastxl = temp.pose.position.x
        temp.pose.position.z = LLeg_odom.pose.pose.position.z
        temp.pose.orientation.x = 0.0
        temp.pose.orientation.y = 0.0
        temp.pose.orientation.z = sin(theta/2)
        temp.pose.orientation.w = cos(theta/2)
        goal.footsteps.append(copy.deepcopy(temp))
        i=i+1



  
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal Sent")

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("Result receivded")

    # Prints out the result of executing the action
    return client.get_result() 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('r')
        rospy.Subscriber("/bipedv5/LLeg/odom", Odometry, LLegcallback)
        rospy.Subscriber("/bipedv5/RLeg/odom", Odometry, RLegcallback)
        rospy.Subscriber("/bipedv5/CoM", Odometry, CoMcallback)
        rospy.Subscriber("/bipedv5/ZMP", PointStamped, ZMPcallback)

        rate = rospy.Rate(50) # 10hz
        while not rospy.is_shutdown():
            print("loop")
            rate.sleep()
            if(LLeg and RLeg and CoM and ZMP):
                break

        print("Out")

        result = action_client()
        print("Result:", action_client)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

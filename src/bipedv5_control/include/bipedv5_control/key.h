#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/Bool.h"
#include <sstream>

using namespace std;
#define space 32
#define zero 48
// #define one 49
#define two 50
#define three 51
#define four 52
#define five 53
#define six 54
#define seven 55
#define eight 56
#define nine 57

#define Ww 119
#define Aa 97
#define Ss 115
#define Dd 100
#define Pp 112
#define Jj 106
#define Kk 107
#define Ll 108
#define Mm 109
#define Nn 110
#define Xx 120

class Key
{
public:
  Key(){
    ini_pub = n.advertise<std_msgs::Bool>("/bipedv5/ini_state",10);
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    cout<<"---------------------------------------------------------------------------"<<endl;
    cout<<"Press S to Start..."<<endl;
  };

    bool readKey(){
        if (read(kfd, &c, 1) > 0)
        {
            cout<<"press key:"<<c<<endl;
            if(c == Kk){
              std_msgs::Bool ini_msg;
              ini_msg.data = true;
              ini_pub.publish(ini_msg);
            }
            if(c == Ss)
               return true;

        }
        return false;
    }

private:
  char c;
  struct termios cooked, raw;
  int kfd = 0;
  ros::Publisher ini_pub;
  ros::NodeHandle n;
};

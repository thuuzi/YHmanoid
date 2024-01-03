#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "hct_control/StringTime.h"
#include <sstream>

using namespace std;
#define space 32
#define zero 48
#define one 49
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
  Key();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Publisher pub2;
};

Key::Key()
{
  pub = nh_.advertise<hct_control::StringTime>("/key", 1);
  pub1 = nh_.advertise<std_msgs::Int8>("hst", 5);
  // pub2 = nh_.advertise<std_msgs::String>("fang", 10);


}

int kfd = 0;
struct termios cooked, raw;

std_msgs::String msg;
std::string msg_front = "Hello"; //消息前缀

std_msgs::Int8 model;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "key");
  Key key_;

  signal(SIGINT, quit);

  sleep(6);
  key_.keyLoop();
  return (0);
}

void Key::keyLoop()
{
  char c;
  bool get_it = false;
  bool hst_it = false;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  for (;;)
  {
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    hct_control::StringTime s_;
       

    s_.header.stamp = ros::Time::now();
    switch (c)
    {
      case zero:
      s_.data = "zero";
      model.data = 0;
      get_it = true;
      hst_it = true;
      puts("HST: Stop");
      break;
    case one:
      s_.data = "one";
      model.data = 2;
      get_it = true;      
      hst_it = true;
      puts("HST: Turn left");
      break;
    case two:
      s_.data = "two";     
      model.data = -2;
      get_it = true;      
      hst_it = true;
      puts("HST: Turn right"); 
      break;
    case three:
      s_.data = "three";
      get_it = true;
      break;
    case four:
      s_.data = "four";
      get_it = true;
      break;
    case five:
      s_.data = "five";
      model.data = -1;
      get_it = true;      
      hst_it = true;
      puts("HST: Down");
      break;
    case six:
      s_.data = "six";
      get_it = true;
      break;
    case seven:
      s_.data = "seven";
      get_it = true;
      break;;

    case eight:
      s_.data = "eight";
      model.data = 1;
      get_it = true;      
      hst_it = true;
      puts("HST: Up");
      break;
    case nine:
      s_.data = "nine";
      get_it = true;
      break;

    case Ww:
      s_.data = "Ww";
      get_it = true;
      puts("Encos: Advance");
      break;
    case Aa:
      s_.data = "Aa";
      get_it = true;
      puts("Encos: Speed UP");
      break;
    case Ss:
      s_.data = "Ss";
      get_it = true;
      puts("Encos: Backward");
      break;
    case Dd:
      s_.data = "Dd";
      get_it = true;
      puts("Encos: Slow Down");
      break;
    case Pp:
      s_.data = "Pp";
      get_it = true;
      break;
    case Jj:
      s_.data = "Jj";
      get_it = true;
      break;
    case Kk:
      s_.data = "Kk";
      get_it = true;
      break;
    case Ll:
      s_.data = "Ll";
      get_it = true;
      break;
    case Mm:
      s_.data = "Mm";
      get_it = true;
      break;
    case Nn:
      s_.data = "Nn";
      get_it = true;
      break;
    case Xx:
      s_.data = "Xx";
      get_it = true;
      break;
    case space:
      s_.data = "space";
      get_it = true;
      puts("Encos: Stop");
      break;
    }


    if (get_it == true)
    {
      // /////////////话题发布测试/////////////
      // //使用 stringstream 拼接字符串与编号
      // std::stringstream ss;
      // ss << msg_front;
      // msg.data = ss.str();
      // //发布消息
      // pub2.publish(msg);
      // ROS_INFO("发送的消息:%s",msg.data.c_str());
      // ////////////////////////////////////
      pub.publish(s_);      
      get_it = false;
      if (hst_it == true)
      {
        pub1.publish(model);
        hst_it = false;
      }
      
    }
  }
  return;
}

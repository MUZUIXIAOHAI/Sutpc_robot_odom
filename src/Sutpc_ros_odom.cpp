//
//  Sutpc_ros_odom.cpp
//  sutpc_ros_odom
//
//  Created by 钟海兴 on 2018/12/29.
//  Copyright © 2018 钟海兴. All rights reserved.
//

#include "Sutpc_ros_odom.hpp"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define MAXSIZE 1024
#define ADV_MSG_LENGTH 11

#define X_Offsite 2.0
#define Y_Offsite 3.0

class send_status
{
public:
    serial_port sp1;
    serial_port sp3;

    char barrier_f;
    char buff[ADV_MSG_LENGTH];
    char rcv_buff1[MAXSIZE];
    char rcv_buff_save1[MAXSIZE];
    
    char rcv_buff3[MAXSIZE];
    char rcv_buff_save3[MAXSIZE];
    
    char lamp_color;
    char lamp_time;
    int save_end1;
    int save_end3;
    double location[3];
    double speed[2];
    char ADVnum;
    
    int left_enc;
    int right_enc;
    int last_left_enc;
    int last_right_enc;
    int last_time;
    float cur_odom_x;         //当前odom的x坐标
    float cur_odom_y;         //当前odom的y坐标
    float cur_odom_th;        //当前odom的角度
    
    char ADVstatus;
    char speedflag;
    char powerstate;
    
    int motor_flag;
    int TestOdemFlag;
    int last_odom_x;
    int last_odom_y;
    float cur_odom_th_z;
  
  //串口下发的速度给定量
  unsigned int speed_x;
  unsigned int speed_y;
  unsigned int speed_z;
  //串口下发的速度方向
  char dir_speed_x;
  char dir_speed_y;
  char dir_speed_z;
    
    fd_set rd1,rd3;
};

send_status ss;

//初始化函数
void initial_all();
//订阅速度主题后，驱动底盘运动
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sutpc_odom_node");
    
    initial_all();

    ros::NodeHandle node;
    ros::Subscriber sub2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed);
  
    
    
}

/*初始化odom初始参数
 *无输入
 *无输出
 */
void initial_all()
{
  ss.save_end1=0;
  ss.save_end3=0;
  for(int i=0;i<3;i++)
    ss.location[i]=0.0;
  for(int i=0;i<2;i++)
    ss.speed[i]=0.0;
  ss.sp1.open_port(11);
  ss.sp1.set_port();

  ss.last_time=clock();
  ss.last_left_enc=0;
  ss.last_right_enc=0;
  ss.cur_odom_x=0.0;         //当前odom的x坐标
  ss.cur_odom_y=0.0;         //当前odom的y坐标
  ss.cur_odom_th=0.0;        //当前odom的角度
  
  ss.ADVnum=0x01;
  ss.ADVstatus=0x41;
  ss.speedflag=0x00;
  ss.powerstate=0x00;
  ss.lamp_color=0x04;
  ss.lamp_time=0x20;
}

/*订阅速度主题后，修改下发的全局速度参数
 *输入：速度主题
 *输出：无
 */
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input)
{
  //下发底盘驱动的各轴速度给定量
  unsigned int speed_x = 0;
  unsigned int speed_y = 0;
  unsigned int speed_z = 0;
  //各轴速度方向
  char dir_speed_x;
  char dir_speed_y;
  char dir_speed_z;
  
  float d = 0.1;//轮子直径
  
  //判断各轴移动方向
  if (cmd_input->linear.x < 0) {
    dir_speed_x = 0x01;
  }
  else{
    dir_speed_x = 0x00;
  }
  if (cmd_input->linear.y < 0) {
    dir_speed_y = 0x01;
  }
  else{
    dir_speed_y = 0x00;
  }
  if (cmd_input->angular.z < 0) {
    dir_speed_z = 0x01;
  }
  else{
    dir_speed_z = 0x00;
  }
  
  //x,y轴速度为 V = n*100/2700 * (pi*d)  （此处n为10ms移动脉冲）
  speed_x = abs(cmd_input->linear.x)/(M_PI*d)*27;
  speed_y = abs(cmd_input->linear.y)/(M_PI*d)*27;
//  speed_z =
  
  //给定到下发串口数据
  ss.speed_x = speed_x;
  ss.speed_y = speed_y;
  ss.speed_z = speed_z;
  
  
}

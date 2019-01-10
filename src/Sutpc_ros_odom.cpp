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

#define CONTROL_MOTOR_MSG_LENGTH 10
#define MAXSIZE 1024
#define ADV_MSG_LENGTH 12


#define RAD2DEG(x) ((x)*180./M_PI)

#define X_Offsite 2.0
#define Y_Offsite 3.0

class send_status
{
public:
    serial_port sp1;
    serial_port sp3;

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
  
    //*****下发数据*****//
    
    //串口下发的速度给定量
    unsigned int speed_x;
    unsigned int speed_y;
    unsigned int speed_z;
    //串口下发的速度方向
    char dir_speed;
    //串口下发数据帧
    unsigned char Send_buff[CONTROL_MOTOR_MSG_LENGTH];
    //速度控制模式
    char control_motor_mode;
    
    //*****采集数据*****//
    
    //各电机的速度大小
    int16_t motor_speed_A;
    int16_t motor_speed_B;
    int16_t motor_speed_C;
    int16_t motor_speed_D;
    //各电机的速度方向 只有三种状态 1代表速度数据为正，0代表停止，2代表速度数据为负
    char motor_speed_A_dir;
    char motor_speed_B_dir;
    char motor_speed_C_dir;
    char motor_speed_D_dir;
    //Z轴陀螺仪数据
    int32_t Z_gyro_speed;
    
    
    fd_set rd1,rd3;
};

send_status ss;

//初始化函数
void initial_all();
//订阅速度主题后，驱动底盘运动
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input);
void velCallback_motorspeed_test(const geometry_msgs::Twist::ConstPtr & cmd_input);
//操作串口1 下发控制数据帧以及收取采集数据帧
void sp1operation();

//*****工具函数*****//
int16_t get_motor_speed(u_char speed, u_char dir);


int main(int argc, char **argv)
{
    //定义节点名称
    ros::init(argc, argv, "sutpc_odom_node");
    //初始化参数
    initial_all();
  
    ros::NodeHandle node;
    //订阅速度主题
//    ros::Subscriber sub2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed);
    ros::Subscriber sub2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed_test);
    //发布里程计主题
    ros::Publisher odom_pub= node.advertise<nav_msgs::Odometry>("odom", 10); //定义要发布/odom主题
    nav_msgs::Odometry odom;//定义里程计对象
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
      0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
      0,  0,    99999, 0,     0,     0,  // covariance on gps_z
      0,  0,    0,     99999, 0,     0,  // large covariance on rot x
      0,  0,    0,     0,     99999, 0,  // large covariance on rot y
      0,  0,    0,     0,     0,     0.01};  // large covariance on rot z
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
      odom.pose.covariance[i] = covariance[i];;
    }
    
    ros::Rate loop_rate(10);
    int cur_time=clock();
    int last_time=clock()-10;
    while (ros::ok())
    {
//        cur_time=clock();
//        //间隔一百毫秒下发串口数据，数据下发频率为10HZ
//        if((int(cur_time - last_time)) >= 100000)
//        {
//            last_time = cur_time;
//            //发送控制数据帧并接收采集数据帧
//            sp1operation();
//            //计算里程计
//            //calculate_odom(odom);
//            //发布里程计主题
//            //odom_pub.publish(odom);
//        }
        ros::spinOnce();
    }
    
    return 0;
    
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
    
    //速度控制模式1，x,y,z轴速度给定模式
    ss.control_motor_mode = 0x01;
//    //速度控制模式2，对每个电机进行单独速度闭环控制
//    ss.control_motor_mode = 0x02;
    
}


/*订阅速度主题，每接收到一次速度就执行一次
 *输入：速度主题
 *输出：发送串口相应数据帧
 */
void velCallback_motorspeed_test(const geometry_msgs::Twist::ConstPtr & cmd_input)
{
    
    if (cmd_input->linear.y < 0) {
        ss.speed_z = 50;
    }
    else{
        ss.speed_z = 100;
    }
    if (cmd_input->angular.z < 0) {
        ss.speed_z = 150;
    }
    else{
        ss.speed_z = 250;
    }
    ss.speed_x = 0;
    ss.speed_y = 0;
    
    //z轴方向,true为正
    bool speed_z_dir = true;
    
    if (speed_z_dir == true) {
        dir_speed_temp |= 0x01;
    }
    else{
        dir_speed_temp &= 0xfe;
    }
    int cur_time=clock();
    int last_time=clock()-10;
    //发送运动控制帧
    sp1operation();
    //执行5秒钟
    while((int(cur_time - last_time)) <= 3000000){
        cur_time=clock();
    }
    //停止旋转
    ss.speed_z = 0;
    //发送运动控制帧
    sp1operation();
    
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
    //各轴速度方向,低三位分别代表各轴速度方向
    unsigned char dir_speed_temp;

    float d = 0.1;//轮子直径

    //判断各轴移动方向
    if (cmd_input->linear.y < 0) {
        dir_speed_temp |= 0x01<<2;
    }
    else{
        dir_speed_temp &= 0xfb;
    }
    if (cmd_input->linear.x < 0) {
        dir_speed_temp |= 0x01<<1;
    }
    else{
        dir_speed_temp &= 0xfd;
    }
    if (cmd_input->angular.z < 0) {
        dir_speed_temp |= 0x01;
    }
    else{
        dir_speed_temp &= 0xfe;
    }

    //x,y轴速度为 V = n*100/2700 * (pi*d)  （此处n为10ms移动脉冲）
    speed_y = fabs(cmd_input->linear.x)/(M_PI*d)*27.0;
    speed_x = fabs(cmd_input->linear.y)/(M_PI*d)*27.0;
    //  speed_z =


    //给定到下发串口数据
    ss.dir_speed = dir_speed_temp;
    ss.speed_x = speed_x;
    ss.speed_y = speed_y;

    ROS_INFO("about the x,y speed , x:%x ,y:%x ", ss.speed_x, ss.speed_y);
  
}

/*串口发送底盘驱动控制帧，并接收采集数据帧
 *输入：无
 *输出：无
 */
void sp1operation()
{
    //ROS_INFO("I heard a laser scan");
    ss.Send_buff[0] = 0xff;                     //帧头
    ss.Send_buff[1] = 0xfe;                     //帧头
    ss.Send_buff[2] = ss.control_motor_mode;    //速度控制模式
    ss.Send_buff[3] = ss.speed_x/256;           //x轴速度高八位
    ss.Send_buff[4] = ss.speed_x%256;           //x轴速度低八位
    ss.Send_buff[5] = ss.speed_y/256;           //y轴速度高八位
    ss.Send_buff[6] = ss.speed_y%256;           //y轴速度低八位
    ss.Send_buff[7] = ss.speed_z/256;           //z轴速度高八位
    ss.Send_buff[8] = ss.speed_z%256;           //z轴速度低八位
    ss.Send_buff[9] = ss.dir_speed;             //各轴速度方向
    

    //发送控制数据帧
    write(ss.sp1.return_port(), ss.Send_buff, CONTROL_MOTOR_MSG_LENGTH);
    //发送完成后读取接收数据帧
    FD_ZERO(&ss.rd1);
    FD_SET(ss.sp1.return_port(),&ss.rd1);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    //fd method to read sp
    //while(FD_ISSET(ss.sp.return_port(),&ss.rd1))
    int retval=0;
    retval=select(ss.sp1.return_port()+1,&ss.rd1,NULL,NULL,&tv);
    if(retval < 0)
        perror("select error!\n");
    else
    {
        if(retval && FD_ISSET(ss.sp1.return_port(),&ss.rd1))
        {
            //从串口缓冲区中读取数据长度
            int s1_recv_len=read(ss.sp1.return_port(),ss.rcv_buff1,MAXSIZE);
            //从串口缓冲区中读取数据
            for(int i=0; i<s1_recv_len; i++ ){
                ss.rcv_buff_save1[ss.save_end1]=ss.rcv_buff1[i];
                ss.save_end1++;
                if(ss.save_end1>=MAXSIZE)
                    ss.save_end1=0;
            }
            //检测采集数据帧是否达到标准数据长度，根据开发手册，返回数据帧为12字节
            if(ss.save_end1>=ADV_MSG_LENGTH)
            {
                //
                for(int i=0;i<=ss.save_end1-ADV_MSG_LENGTH;i++)
                {
                    //检测帧头
                    if((unsigned char)ss.rcv_buff_save1[i]==0xff&&(unsigned char)ss.rcv_buff_save1[i+1]==0xfe)
                    {
                        //采集数据帧无bcc校验，直接读取数据
                        ss.motor_speed_A = get_motor_speed(ss.rcv_buff_save1[i+2], ss.rcv_buff_save1[i+3]);
                        ss.motor_speed_B = get_motor_speed(ss.rcv_buff_save1[i+4], ss.rcv_buff_save1[i+5]);
                        ss.motor_speed_C = get_motor_speed(ss.rcv_buff_save1[i+6], ss.rcv_buff_save1[i+7]);
                        ss.motor_speed_D = get_motor_speed(ss.rcv_buff_save1[i+8], ss.rcv_buff_save1[i+9]);
                        ss.Z_gyro_speed = ss.rcv_buff_save1[i+10]*256 + ss.rcv_buff_save1[i+11] - 32768;
                        //测试读取结果
                        //ROS_INFO("here are motors' status, A:%x  B:%x  C:%x  D:%x.", ss.motor_speed_A, ss.motor_speed_B, ss.motor_speed_C, ss.motor_speed_D);
                        
                        ss.save_end1=0;
                    }
                }
            }
        }
    }
}

/*工具函数：得到各电机的速度，带方向
 *输入：不带方向的速度
 *输出：带方向的速度
 */
int16_t get_motor_speed(u_char speed, u_char dir){
    int16_t output_speed;
    //各电机的速度方向 只有三种状态 1代表速度数据为正，0代表停止，2代表速度数据为负
    switch (dir) {
        case 1:
            output_speed = speed;
            break;
            
        case 0:
            output_speed = 0;
            break;
            
        case 2:
            output_speed = -speed;
            break;
            
        default:
            break;
    }
    
    return output_speed;
}

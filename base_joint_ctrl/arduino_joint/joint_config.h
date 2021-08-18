/**************************************************************
***  接线：
      enPin接En（闭环驱动板屏幕上的En选项选择L或Hold）
      stpPin接Stp
      dirPin接Dir
      V+和Gnd接10~28V供电
      非工业套餐（不带光耦隔离）要把Arduino控制板的Gnd和闭环驱动板的Gnd接在一起（共地）

*** 注意事项：
      Arduino控制板和闭环驱动板的两个Gnd要接在一起
      先接好线再通电，不要带电拔插！！！
      上电时，先通10~28V供电，再通Arduino控制板USB供电！！！避免一些效应造成损坏
      断电时，先断Arduino控制板USB供电，再断10~28V供电。

*** 串口地址0xe0~0xe9
    UartBaud
    Disable
    9600
    19200
    25000
    38400
    57600
    115200

*** 串口
  数据位 8
  校验位 None
  停止位 1
  SERIAL_8N1
  Serial.readBytes(buffer, length)
  Serial.available() 获取串口接收缓冲区中的字节数
***************************************************************/
#ifndef __JOINT_CONFIG_H
#define __JOINT_CONFIG_H
#include <Arduino.h>
/***电机驱动 Emm42_V3.6.x***/
//点击反转标志 127最高挡位速度反转：7F|80=FF,90挡位速度反转：5A|80=DA,00挡位速度反转：00|80=80
#define MOTOR_ROLL_OVER 0x80
//串口位置控制 0xE0FD {1} {2} {3} 1为速度转向 {2}{3}为脉冲数，3200个脉冲(0x0C80)为360°
#define MOTOR_DEGREE 0xFD
//电机停止正反转
#define MOTOR_STOP 0xF7
//电机以一定的速度正反转
#define MOTOR_RUN 0xF6
//串口控制下驱动板en使能 0xE0F300关闭,0xE0F301使能
#define MOTOR_EN_ENABLE 0xF3
//获取en使能状态 01使能 02关闭
#define MOTOR_EN_TAG 0x3A
//角度误差 int16_t 65536=360°
#define MOTOR_ANGLE_ERROR 0x39
//获取电机角度实时位置 int32_t 65535=360°，N圈则xN
#define MOTOR_ANGLE 0x36
//获取编码器位置 uint16_t 65535=360°
#define MOTOR_ENCODER 0x30
//设置1~256细分:0x8407=7细分,0x840C=12细分,0x844e=78细分
#define MOTOR_PULSE_PARTITION 0x84
//电机堵转标志位 uint8_t堵转标志
#define MOTOR_STOP_TAG 0x3E
/***电机驱动END***/
//定义电机
#define NUM_MOTOR_SERIAL3_1 0xE0
#define NUM_MOTOR_SERIAL3_2 0xE1

uint8_t initDevice(void);
//复位电机
uint8_t reload(uint8_t motor);
#endif

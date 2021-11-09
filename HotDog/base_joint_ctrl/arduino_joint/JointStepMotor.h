/**
   步进电机驱动控制
*/
#ifndef __JOINT_STEP_MOTOR_H
#define __JOINT_STEP_MOTOR_H
#define BASE_INTERVAL_MS 5
#include <Arduino.h>

class JointStepMotor {
  public:
    JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin);
    JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin, float defaultDriveRatio);
    JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin, float defaultDriveRatio, uint8_t defaultSpeedLevel);
    /**
       获取关节角度
    */
    float getJointDegree();
    /**
       正向旋转
       @param forwardDegree 关节旋转角度
              speedLevel    电机转速
    */
    void forward(float forwardDegree);
    void forward(float forwardDegree, uint8_t speedLevel);
    /**
      反向旋转
      @param reverseDegree 关节旋转角度
             speedLevel    电机转速
    */
    void reverse(float reverseDegree);
    void reverse(float reverseDegree, uint8_t speedLevel);
    /**
       电机旋转
       @param rotateDegree  电机旋转角度
              reverseTag    反转标志:0正转，1反转
              speedLevel    电机转速
    */
    void rotate(float rotateDegree, boolean reverseTag);
    void rotate(float rotateDegree, boolean reverseTag, uint8_t speedLevel);
    uint8_t getEnPin();
    uint8_t getStpPin();
    uint8_t getDirPin();
  private:
    //电机当前相对角度
    float motorDegree;
    //默认电机转速挡位：0~10
    uint8_t defaultSpeedLevel = 5;
    float defaultDriveRatio = 1.0f;
    uint8_t enPin, stpPin, dirPin;
    //初始化电机接口
    void initMotor();
    void runDegree(float degree, uint8_t speedLevel);
    void runOne(uint8_t stpPin, uint8_t speedLevel);
};


#endif

#include "JointStepMotor.h"


JointStepMotor::JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin) {
  this->enPin = enPin;
  this->stpPin = stpPin;
  this->dirPin = dirPin;
  this->initMotor();
}
JointStepMotor::JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin, float defaultDriveRatio) {
  this->enPin = enPin;
  this->stpPin = stpPin;
  this->dirPin = dirPin;
  this->defaultDriveRatio = defaultDriveRatio;
  this->initMotor();
}
JointStepMotor::JointStepMotor(uint8_t enPin, uint8_t stpPin, uint8_t dirPin, float defaultDriveRatio, uint8_t defaultSpeedLevel) {
  this->enPin = enPin;
  this->stpPin = stpPin;
  this->dirPin = dirPin;
  this->defaultDriveRatio = defaultDriveRatio;
  this->defaultSpeedLevel = defaultSpeedLevel;
  this->initMotor();
}

void JointStepMotor::initMotor() {
  pinMode(enPin, OUTPUT);  digitalWrite(enPin , LOW);  // initialize the En pin as an output
  pinMode(stpPin, OUTPUT);  digitalWrite(stpPin, LOW);  // initialize the Stp pin as an output
  //  tone(stpPin, 1100, 1000);
  pinMode(dirPin, OUTPUT);  digitalWrite(dirPin, LOW);  // initialize the Dir pin as an output
}
uint8_t JointStepMotor::getEnPin() {
  return this->enPin;
}

uint8_t JointStepMotor::getStpPin() {
  return this->stpPin;
}
uint8_t JointStepMotor::getDirPin() {
  return this->dirPin;
}
/**
   正向旋转
   @param forwardDegree 关节旋转角度
          speedLevel    电机转速
*/
void JointStepMotor::forward(float forwardDegree);
void JointStepMotor::forward(float forwardDegree, uint8_t speedLevel);
/**
  反向旋转
  @param reverseDegree 关节旋转角度
         speedLevel    电机转速
*/
void JointStepMotor::reverse(float reverseDegree);
void JointStepMotor::reverse(float reverseDegree, uint8_t speedLevel);
/**
   电机旋转
   @param rotateDegree  电机旋转角度
          reverseTag    反转标志:0正转，1反转
          speedLevel    电机转速
*/
void JointStepMotor::rotate(float rotateDegree, boolean reverseTag) {
  if (reverseTag) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  runDegree(rotateDegree, defaultSpeedLevel);
}
void JointStepMotor::rotate(float rotateDegree, boolean reverseTag, uint8_t speedLevel);


void JointStepMotor::runDegree(float degree, uint8_t speedLevel) {
  Serial.print("Moter");
  Serial.print(dirPin);
  long times = degree * 3200 / 360;
  Serial.print("Times:");
  Serial.println(times);
  //    for (long i = 0; i < times; i++) {
  //      runOne(stpPin, speedLevel);
  //    }
  long frequency = 32000;
  tone(stpPin, frequency, times / frequency * 1000);
  delay(2000);
//  noTone(stpPin);
  Serial.println("go out");
}
/**
   发送一个脉冲
*/
void JointStepMotor::runOne(uint8_t stpPin, uint8_t speedLevel) {
  //  unsigned long currentMillis = micros();
  //  unsigned long previousMillis = micros();
  //  boolean highTag = false;
  //  /**********************************************************
  //  ***  高低电平的时间间隔，即脉冲时间的一半(控制电机转动速度)
  //  **********************************************************/
  //  unsigned long interval = BASE_INTERVAL_MS * (1 + speedLevel);
  //  /**********************************************************
  //  ***  取反D6（Stp引脚）
  //  **********************************************************/
  //  digitalWrite(stpPin, HIGH);
  //  while (currentMillis - previousMillis < 2 * interval) {
  //    if (!highTag & currentMillis - previousMillis >= interval) {
  //      // STP引脚取反
  //      digitalWrite(stpPin, LOW);
  //      highTag = true;
  //      times0++;
  //    }
  //    currentMillis = micros();
  //  }
  /**********************************************************
  ***  高低电平的时间间隔，即脉冲时间的一半(控制电机转动速度)
  **********************************************************/
  int delayTime = 10;
  /**********************************************************
  ***  取反D6（Stp引脚）
  **********************************************************/
  digitalWrite(stpPin, HIGH);
  delayMicroseconds(delayTime); //600us
  digitalWrite(stpPin, LOW);
  delayMicroseconds(delayTime); //600us

}

unsigned long _lastStepTime = 0;

/**
   关节控制单元
   Arduino Mega2560
   Serial 0 RX 1 TX
   Serial1 18 TX 19 RX
   Serial2 16 TX 17 RX
   Serial3 14 TX 15 RX
*/
#include <stdint.h>
#include "joint_config.h"

struct _motor {
  short enPin;
  short stpPin;
  short dirPin;
}
motor1 = {5, 6, 7},
motor2 = {1, 2, 3};

void setup() {
  initDevice();
  pinMode(motor1.enPin , OUTPUT);  digitalWrite(motor1.enPin , LOW);  // initialize the En pin as an output
  pinMode(motor1.stpPin, OUTPUT);  digitalWrite(motor1.stpPin, LOW);  // initialize the Stp pin as an output
  pinMode(motor1.dirPin, OUTPUT);  digitalWrite(motor1.dirPin, LOW);  // initialize the Dir pin as an output
}

uint16_t motorPositionCode = 0x0;
long i = 0;  bool cntDir = false;
void loop() {
  double motorDegree = getMotorDegree(NUM_MOTOR_SERIAL3_1);
  runDegree(90, motor1);
  if (cntDir) {
    digitalWrite(motor1.dirPin, LOW);
    cntDir = false;
  } else {
    digitalWrite(motor1.dirPin, HIGH);
    cntDir = true;
    //切换方向转动
  }
  delay(2000);
}

void runDegree(float degree, _motor motor) {
  long times = degree * 3200 / 360;
  //  Serial.print("Times:");
  //  Serial.print(times);
  //  Serial.print("\r\n");
  for (i = 0; i < times; i++) {
    runOne(motor);
  }
}
/**
   发送一个脉冲
*/
void runOne(_motor motor) {
  /**********************************************************
  ***  高低电平的时间间隔，即脉冲时间的一半(控制电机转动速度)
  **********************************************************/
  int delayTime = 100;
  delayMicroseconds(delayTime); //600us
  /**********************************************************
  ***  取反D6（Stp引脚）
  **********************************************************/
  digitalWrite(motor.stpPin, !digitalRead(motor.stpPin));
  digitalWrite(motor.stpPin, !digitalRead(motor.stpPin));
  delayMicroseconds(delayTime); //600us
}

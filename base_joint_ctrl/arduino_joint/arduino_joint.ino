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
#include "JointStepMotor.h"
struct _motor {
  short enPin;
  short stpPin;
  short dirPin;
}
motor1 = {10, 11, 12},
motor2 = {2, 3, 4};


JointStepMotor stepper1(2, 3, 4);
JointStepMotor stepper2(6, 7, 8);

void setup() {
  initDevice();

}

uint16_t motorPositionCode = 0x0;
long i = 0;  bool cntDir = false;
void loop() {
  Serial.println(stepper1.getEnPin() + stepper1.getDirPin() + stepper1.getStpPin());
  Serial.println(stepper2.getEnPin() + stepper2.getDirPin() + stepper2.getStpPin());

//  stepper1.rotate(360*20.0f, false);
//  stepper2.rotate(360*20.0f, false);
  delay(10000);
}

void motorRun() {
  double motorDegree = getMotorDegree(NUM_MOTOR_SERIAL3_1);
  runDegree(90 * 5, motor1);
  if (cntDir) {
    digitalWrite(motor1.dirPin, LOW);
    cntDir = false;
  } else {
    digitalWrite(motor1.dirPin, HIGH);
    cntDir = true;
    //切换方向转动
  }
  delay(5000);
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
  int delayTime = 10;
  delayMicroseconds(delayTime); //600us
  /**********************************************************
  ***  取反D6（Stp引脚）
  **********************************************************/
  digitalWrite(motor.stpPin, !digitalRead(motor.stpPin));
  digitalWrite(motor.stpPin, !digitalRead(motor.stpPin));
  delayMicroseconds(delayTime); //600us
}

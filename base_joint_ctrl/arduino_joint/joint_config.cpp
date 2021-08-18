#include "joint_config.h"

uint8_t initDevice(void) {
  //控制台输出信号
  Serial.begin(19200);
  //串口信号
  Serial3.begin(38400, SERIAL_8N1);

  reload(NUM_MOTOR_SERIAL3_1);

  return 0;
}

uint8_t reload(uint8_t motor) {
  uint8_t serialCode[3] = {motor, 0x00, 0x00};
  uint8_t motorAngleCode[5];
  //停止电机
  serialCode[1] = MOTOR_DEGREE;
  Serial3.write(serialCode, 2);
  Serial3.flush();
  //Todo 转到限位开关/堵转位置
  //读电机角度位置
  serialCode[1] = MOTOR_ANGLE;
  Serial3.write(serialCode, 2);
  if (Serial3.available() > 0) {
    Serial3.readBytes(motorAngleCode, 5);
  }
  //获取编码器位置
  serialCode[1] = MOTOR_ENCODER;
  Serial3.write(serialCode, 2);
  if (Serial3.available() > 0) {
    Serial3.readBytes(motorAngleCode, 3);
  }
  //计算当前编码器角度
  Serial.print("code:");
  Serial.println(motorAngleCode[0], HEX);
  if (motorAngleCode[0] == motor) {
    //高8位
    uint16_t encoderAngle = motorAngleCode[1] << 8 | motorAngleCode[2];
    //转换为脉冲
    uint16_t pulse = encoderAngle * 3200 / 65535;
    serialCode[1] = 0x5A | MOTOR_ROLL_OVER;
    serialCode[2] = pulse >> 8;
    serialCode[3] = pulse - serialCode[2];
    //旋转至0°
  }
}

#include "joint_config.h"
//清除缓冲区
void cleanBuffer() {
  while (Serial.read() >= 0);
}
void cleanBuffer3() {
  while (Serial3.read() >= 0);
}

uint8_t initDevice(void) {
  //控制台输出信号
  Serial.begin(115200);
  //串口信号
  Serial3.begin(38400, SERIAL_8N1);

  //  reload(NUM_MOTOR_SERIAL3_2);
  cleanBuffer();
  return 0;
}

uint8_t reload(uint8_t motor) {
  uint8_t serialCode[3] = {motor, 0x00, 0x00};
  byte motorAngleCode[4];
  //停止电机
  serialCode[1] = MOTOR_DEGREE;
  Serial.println(serialCode[0], HEX);
  Serial.println(serialCode[1], HEX);
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
    Serial.println(motorAngleCode[0], HEX);
    //转换为脉冲
    uint16_t pulse = encoderAngle * 3200 / 65535;
    serialCode[1] = 0x5A | MOTOR_ROLL_OVER;
    serialCode[2] = pulse >> 8;
    serialCode[3] = pulse - serialCode[2];
    //旋转至0°
  }
}

uint32_t getMotorPulse(uint8_t motor) {
  byte serialCode[2] = {motor, 0};
  byte startCode[4];
  serialCode[1] = MOTOR_ENCODER;
  Serial3.write(serialCode, 2);
  if (Serial3.available() > 0) {
    byte data = Serial3.read();
    if (motor == data) {
      Serial3.readBytes(startCode, 2);
      uint16_t motorPositionCode = (startCode[0] << 8) + startCode[1];
      uint32_t pulse = ((double)motorPositionCode ) / 65535 * 3200;
      Serial.println(startCode[0], HEX);
      Serial.println(startCode[1], HEX);
      Serial.println(motorPositionCode, DEC);
      Serial.println(pulse, DEC);
      return pulse;
    } else {
      Serial.print("waste data:");
      Serial.println(data, HEX);
      return 0;
    }
  }
}

void motorRunAngle(uint8_t motor, uint32_t pulse, byte motorRollOverTag) {
  //  byte command[5] = {motor, MOTOR_DEGREE, 0x5A | motorRollOverTag, pulse >> 8, pulse << 8 >> 8};
  byte command[5] = {NUM_MOTOR_SERIAL3_1, MOTOR_DEGREE, 0x5A | 0x80, pulse >> 8, pulse - (pulse >> 8 << 8)};
  Serial3.write(command, 5);
}

//获取编码器值
uint16_t getMotorEncoderCode(uint8_t motor) {
  //组装命令
  byte bufferData[2] = {motor, MOTOR_ENCODER};
  uint16_t resultData = 0;
  //清空缓存
  cleanBuffer3();
  //发起请求
  Serial3.write(bufferData, 2);
  //获取缓冲区数据
  if (Serial3.available() > 0) {
    uint8_t dataHead = Serial3.read();
    if (motor == dataHead) {
      resultData = resultData | Serial3.read() << 8 | Serial3.read();
    }
  }
  //清空缓存
  cleanBuffer3();
  return resultData;
}


//获取编码器对应角度
double getMotorDegree(uint8_t motor){
  uint16_t encoderCode=getMotorEncoderCode(motor);
  return (double)(encoderCode/65535)*360;
}

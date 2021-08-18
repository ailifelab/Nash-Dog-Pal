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

void setup() {
  initDevice();

}

uint16_t motorPositionCode = 0x0;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("roll!");
  byte command[5] = {NUM_MOTOR_SERIAL3_1, MOTOR_DEGREE, 0x64, 0x03, 0x20};
  Serial3.write(command, 5);
//  int sizeBuf = Serial3.available();
//  if (sizeBuf > 0) {
//    for (int i = 0; i < sizeBuf; i++) {
//      byte data = Serial3.read();
//      Serial.print("returned:");
//      Serial.println(data, HEX);
//    }
//  }
  delay(6000);
  Serial.println("back!");
  uint32_t pulse = 0;
  pulse = getMotorPulse(NUM_MOTOR_SERIAL3_1);
  if (pulse < 3201) {
    Serial.print("motor pulse:");
    Serial.println(pulse, DEC);
    if (pulse > 0) {
      byte command[5] = {NUM_MOTOR_SERIAL3_1, MOTOR_DEGREE, 0x64 | 0x80, pulse >> 8, pulse - (pulse >> 8 << 8)};
      Serial.print(command[0], HEX);
      Serial.print(command[1], HEX);
      Serial.print(command[2], HEX);
      Serial.print(command[3], HEX);
      Serial.println(command[4], HEX);

      Serial3.write(command, 5);
    }
    delay(6000);
  }
}

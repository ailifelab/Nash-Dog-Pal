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

void loop() {
  // put your main code here, to run repeatedly:
  //获取上位机命令
  Serial.read();

}

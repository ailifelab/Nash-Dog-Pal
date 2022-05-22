#include <Servo.h>
#include <stdlib.h>
#include "MultiStepper.h"
#include "AccelStepper.h"
#include "Jt808Util.h"
#define SERVO_PIN 3
#define STEP_MOTOR_EN_L 19
#define STEP_MOTOR_STP_L 9
#define STEP_MOTOR_DIR_L 8
#define STEP_MOTOR_EN_R 22
#define STEP_MOTOR_STP_R 10
#define STEP_MOTOR_DIR_R 7
#define STEP_LIGHT_FRONT 11
#define STEP_LIGHT_BACK_LEFT 5
#define STEP_LIGHT_BACK_RIGHT 6
#define BTN_UP 2
#define BTN_DOWN 4
#define MAX_SPEED       6400
#define MAX_ACCEL       500
//通讯标识 标识 校验码 外设类型编号 命令类型 用户数据 标识
const byte CODE_FLAG = 0x7e;
//外设类型编号
const byte CODE_TYPE = 0xfa;
//命令类型：舵机角度调整 7E00FA44007E
const byte CODE_CMD_SERVO = 0x44;
//命令类型：左步进电机旋转 7E00FA450008007E
const byte CODE_STEP_L = 0x45;
//命令类型：右步进电机旋转 7E00FA460008007E
const byte CODE_STEP_R = 0x46;
//命令类型：原地旋转
const byte CODE_STEP_ROUND = 0x47;
//命令类型：直线行进
const byte CODE_STEP_FORWARD = 0X48;
// 命令类型：stop move
const byte CODE_STOP = byte(0x49);
// 消息类型：心跳信息 7E4AFA50007E
const byte  CODE_TICK = byte(0x50);
// 命令类型:breath front lights
const byte CODE_LIGHT_FRONT = byte(0x51);
// 命令类型:breath back lights
const byte CODE_LIGHT_BACK = byte(0x52);
// 命令类型:按钮动作
const byte CODE_BTN = byte(0x53);
//轮距
const short SHAFT_LENGTH = 151;
//车轮直径
const short WHEEL_PERIMETER = 82;
//传动比
const short GEAR_RATIO = 20;
byte *pcMsg = new byte[10] { byte(0x00), byte(0x00), byte(0xfa) };
//舵机
Servo myservo;
//左电机
AccelStepper stepper_l(AccelStepper::DRIVER, STEP_MOTOR_STP_L, STEP_MOTOR_DIR_L);
//右电机
AccelStepper stepper_r(AccelStepper::DRIVER, STEP_MOTOR_STP_R, STEP_MOTOR_DIR_R);
//缓存命令
byte * cmdCache = (byte *)calloc(10, sizeof(byte));
int cacheIndex = 0;
//串口命令接收完成标识
bool cmdEndTag = false;
Jt808Util jt808Util;
//灯闪烁目标时间
unsigned long aimTime = 0L;
//左侧灯闪烁标记
bool backLeftLightFlash = false;
//右侧灯闪烁标记
bool backRightLightFlash = false;
//按键触发时间
long btnUpStartTime = 0;
long btnDownStartTime = 0;
bool btnUpSent = false;
bool btnDownSent = false;
int lightBrightness = 0;
/**
   设置灯闪烁
*/
void setLightControl(int pinCode, int brightData) {
  if (0 == brightData) {
    analogWrite(pinCode, 0);
  } else if (brightData < int(0x64)) {
    // flash light
    aimTime = 1000 * brightData + millis();
    if (pinCode == STEP_LIGHT_BACK_LEFT) {
      backLeftLightFlash = true;
    } else if (pinCode == STEP_LIGHT_BACK_RIGHT) {
      backRightLightFlash = true;
    }
  } else {
    // always lightup
    analogWrite(pinCode, brightData);
  }
}

/**
   执行灯闪烁
*/
void lightFlash() {
  unsigned long nowTime = millis();
  if (nowTime >= aimTime) {
    if (backLeftLightFlash) {
      analogWrite(STEP_LIGHT_BACK_LEFT, 0);
      backLeftLightFlash = false;
    }

    if (backRightLightFlash) {
      analogWrite(STEP_LIGHT_BACK_RIGHT, 0);
      backRightLightFlash = false;
    }
  } else {
    unsigned long deltaTime = aimTime - nowTime;

    // 3000,2500,2000,1500,1000,500,0
    if (deltaTime % 1000 == 0) {
      if (backLeftLightFlash) {
        analogWrite(STEP_LIGHT_BACK_LEFT, 255);
      }

      if (backRightLightFlash) {
        analogWrite(STEP_LIGHT_BACK_RIGHT, 255);
      }
    } else if (deltaTime % 500 == 0) {
      if (backLeftLightFlash) {
        analogWrite(STEP_LIGHT_BACK_LEFT, 0);
      }

      if (backRightLightFlash) {
        analogWrite(STEP_LIGHT_BACK_RIGHT, 0);
      }
    }
  }
}

/**
   控制灯明暗
*/
void breathLed(int pinCode, int brightness, int breathSpeed) {
  if (brightness > lightBrightness) {
    for (short i = lightBrightness; i <= brightness; i++) {
      analogWrite(pinCode, i);
      delay(breathSpeed);
    }
  } else {
    for (short i = lightBrightness; i > brightness; i--) {
      analogWrite(pinCode, i);
      delay(breathSpeed);
    }
  }
  lightBrightness = brightness;
}
/**
   按钮检测
*/
void checkButton() {
  int upStat = digitalRead(BTN_UP);
  if (upStat == LOW) {
    if (btnUpStartTime == 0) {
      btnUpStartTime = millis();
      //7E4DFA53007E
      Serial.write(CODE_FLAG);
      Serial.write(0x4D);
      Serial.write(CODE_TYPE);
      Serial.write(CODE_BTN);
      Serial.write(0x00);
      Serial.write(CODE_FLAG);
    }
  } else {
    long endTime = millis() - btnUpStartTime;
    if (endTime > 300) {
      btnUpStartTime = 0;
    }
  }
  int downStat = digitalRead(BTN_DOWN);
  if (downStat == LOW) {
    if (btnDownStartTime == 0) {
      btnDownStartTime = millis();
      //7E4EFA53017E
      Serial.write(CODE_FLAG);
      Serial.write(0x4E);
      Serial.write(CODE_TYPE);
      Serial.write(CODE_BTN);
      Serial.write(0x01);
      Serial.write(CODE_FLAG);
    }
  } else {
    long endTime = millis() - btnDownStartTime;
    if (endTime > 300) {
      btnDownStartTime = 0;
    }
  }
}

/**
   初始化
*/
void setup() {
  myservo.attach(SERVO_PIN);
  Serial.begin(115200, SERIAL_8N1); //连接到串行端口，波特率为115200
  //左侧电机设置
  stepper_l.setEnablePin(STEP_MOTOR_EN_L);
  stepper_l.enableOutputs();
  stepper_l.setMaxSpeed(15000);
  stepper_l.setAcceleration(1000.0);
  //右侧电机设置
  stepper_r.setEnablePin(STEP_MOTOR_EN_R);
  stepper_r.enableOutputs();
  stepper_r.setMaxSpeed(15000);
  stepper_r.setAcceleration(1000.0);
  pinMode(STEP_LIGHT_FRONT, OUTPUT);
  pinMode(STEP_LIGHT_BACK_LEFT, OUTPUT);
  pinMode(STEP_LIGHT_BACK_RIGHT, OUTPUT);
  analogWrite(STEP_LIGHT_FRONT, 0);
  analogWrite(STEP_LIGHT_BACK_LEFT, 0);
  analogWrite(STEP_LIGHT_BACK_RIGHT, 0);
  pinMode(BTN_UP, INPUT);
  digitalWrite(BTN_UP, HIGH);
  pinMode(BTN_DOWN, INPUT);
  digitalWrite(BTN_DOWN, HIGH);
  myservo.write(0);
  breathLed(STEP_LIGHT_FRONT, 50, 10);
  //ready
  Serial.write(CODE_FLAG);
  Serial.write(0x4A);
  Serial.write(CODE_TYPE);
  Serial.write(CODE_TICK);
  Serial.write(0x00);
  Serial.write(CODE_FLAG);
}

/**
   程序执行
*/
void loop() {
  //接收串口数据
  while (Serial.available()) {
    //读一个字节
    byte inByte = Serial.read();
    //数据尾
    if (CODE_FLAG == inByte && cacheIndex > 3) {
      cmdCache[cacheIndex++] = inByte;
      cacheIndex = 0;
      cmdEndTag = true;
      break;
    } else {
      cmdCache[cacheIndex++] = inByte;
    }
  }
  //解析串口数据
  if (cmdEndTag) {
    int realSize = 10;
    //检查有效的数据
    if (jt808Util.checkData(cmdCache, realSize)) {
      byte *infoDataPointer;
      infoDataPointer = jt808Util.getInfoData(cmdCache, realSize);
      int8_t lightType;
      long   moveToPosition;

      // 角度值
      int angle = 0;

      // infoDataPointer[0] is device code
      byte  codeCmd = *(infoDataPointer + 1);
      long  currentPositionL = 0L;
      long  currentPositionR = 0L;
      byte *data;
      byte *msgData;
      switch (codeCmd) {
        //舵机角度调整
        case CODE_CMD_SERVO:
          if (*(infoDataPointer + 2) > byte(0x4B)) {
            angle = 75;
          } else {
            angle = (int) * (infoDataPointer + 2);
          }
          // 控制舵机转动相应的角度
          myservo.write(angle);
          break;
        //左侧电机控制
        case CODE_STEP_L:
          moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);
          if (cmdCache[4] == byte(0x01)) {
            // 反转
            moveToPosition = -moveToPosition;
          }
          stepper_l.setSpeed(5000);
          stepper_l.move(moveToPosition);
          break;
        //右侧电机控制
        case CODE_STEP_R:
          moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);
          if (cmdCache[4] == byte(0x01)) {
            // 反转
            moveToPosition = -moveToPosition;
          }
          stepper_r.setSpeed(5000);
          stepper_r.move(moveToPosition);
          break;
        //原地旋转
        case CODE_STEP_ROUND:
          angle = jt808Util.getMovePosition(infoDataPointer, realSize);

          // 1/16 0.9degree 6400steps for wheel 1 circle
          // step=L/D*6400*angle/360 L:shaft length,D:wheel perimeter
          // moveToPosition:left wheel
          moveToPosition = 6400L * angle * GEAR_RATIO * SHAFT_LENGTH
                           / WHEEL_PERIMETER  / 360;

          if (cmdCache[4] == byte(0x01)) {
            // counterclockwise right toward,left backward
            moveToPosition = -moveToPosition;
          }
          stepper_l.setSpeed(5000);
          stepper_r.setSpeed(5000);
          stepper_l.move(moveToPosition);
          stepper_r.move(-moveToPosition);
          break;
        //直线运行
        case CODE_STEP_FORWARD:
          moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);
          if (cmdCache[4] == byte(0x01)) {
            // backward
            moveToPosition = -moveToPosition;
          }
          stepper_l.setSpeed(5000);
          stepper_r.setSpeed(5000);
          stepper_l.move(moveToPosition);
          stepper_r.move(moveToPosition);
          break;
        // stop move
        case CODE_STOP:
          currentPositionL = stepper_l.currentPosition();
          currentPositionR = stepper_r.currentPosition();
          stepper_l.moveTo(currentPositionL + 10 * GEAR_RATIO);
          stepper_r.moveTo(currentPositionR + 10 * GEAR_RATIO);
          break;
        case CODE_TICK:
          if ( stepper_l.isRunning() || stepper_r.isRunning()) {
            currentPositionL = stepper_l.distanceToGo();
            currentPositionR = stepper_r.distanceToGo();
            byte *data = jt808Util.getBytesData(currentPositionL, realSize);
            byte *msgData = jt808Util.generateMsgCode(CODE_TYPE, data, realSize);
            for (int i = 0; i < realSize; i++) {
              if (i == 3) {
                Serial.write(CODE_STEP_L);
              }
              Serial.write(msgData[i]);
            }
            free(data);
            free(msgData);
            data = jt808Util.getBytesData(currentPositionR, realSize);
            msgData = jt808Util.generateMsgCode(CODE_TYPE, data, realSize);
            for (int i = 0; i < realSize; i++) {
              if (i == 3) {
                Serial.write(CODE_STEP_R);
              }
              Serial.write(msgData[i]);
            }
            free(data);
            free(msgData);
          } else {
            Serial.write(CODE_FLAG);
            Serial.write(0x4A);
            Serial.write(CODE_TYPE);
            Serial.write(CODE_TICK);
            Serial.write(0x00);
            Serial.write(CODE_FLAG);
          }
          break;
        // control the front light
        case CODE_LIGHT_FRONT:
          moveToPosition = (int) * (infoDataPointer + 2);

          if (moveToPosition > 255) {
            moveToPosition = 255;
          } else if (moveToPosition < 100) {
            moveToPosition = 0;
          } else {
            moveToPosition = (int) * (infoDataPointer + 2);
          }
          breathLed(STEP_LIGHT_FRONT, moveToPosition, 10);
          break;

        // control the back light,7EXXFA52KKSS7E
        // KK:00->left,01->right,02->both; SS:00->close,
        // 01~63->flash for SS times,64~ff->bright strength
        case CODE_LIGHT_BACK:
          lightType = int(*(infoDataPointer + 2));
          angle = int(*(infoDataPointer + 3));

          if (0 == lightType) {
            setLightControl(STEP_LIGHT_BACK_LEFT, angle);
          } else if (1 == lightType) {
            setLightControl(STEP_LIGHT_BACK_RIGHT, angle);
          } else if (2 == lightType) {
            setLightControl(STEP_LIGHT_BACK_LEFT,  angle);
            setLightControl(STEP_LIGHT_BACK_RIGHT, angle);
          }

          break;
      }
      if (infoDataPointer != NULL) {
        free(infoDataPointer);
        infoDataPointer = NULL;
      }
    }
    cmdEndTag = false;
    for (int i = 0; i < 10; i++) {
      cmdCache[i] = 0;
    }
  }
  //接收按钮动作
  checkButton();
  //控制灯闪烁
  lightFlash();
  //控制电机运转
  stepper_l.run();
  stepper_r.run();
}

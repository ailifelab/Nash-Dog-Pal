#include <iostream>
#include "Jt808Util.h"
using namespace std;

#define SERVO_PIN 3
#define STEP_MOTOR_EN_L 19
#define STEP_MOTOR_STP_L 5
#define STEP_MOTOR_DIR_L 8
#define STEP_MOTOR_EN_R 20
#define STEP_MOTOR_STP_R 6
#define STEP_MOTOR_DIR_R 7
#define STEP_LIGHT_FRONT 9
#define STEP_LIGHT_BACK_LEFT 10
#define STEP_LIGHT_BACK_RIGHT 11
#define MAX_SPEED       6400
#define MAX_ACCEL       500

// 通讯标识 标识 校验码 外设类型编号 命令类型 用户数据 标识
const byte CODE_FLAG = byte(0x7e);

// 外设类型编号
const byte CODE_TYPE = byte(0xfa);

// 命令类型：舵机角度调整 7E00FA44007E
const byte CODE_CMD_SERVO = byte(0x44);

// 命令类型：左步进电机旋转 7E00FA450008007E
const byte CODE_STEP_L = byte(0x45);

// 命令类型：右步进电机旋转 7E00FA460008007E
const byte CODE_STEP_R = byte(0x46);

// 命令类型：原地旋转
const byte CODE_STEP_ROUND = byte(0x47);

// 命令类型：直线行进
const byte CODE_STEP_FORWARD = byte(0X48);

// 命令类型：stop move
const byte CODE_STOP = byte(0x49);

// 命令类型：心跳信息 7E4AFA50007E
const byte CODE_TICK = byte(0x50);

// 命令类型:breath front lights
const byte CODE_LIGHT_FRONT = byte(0x51);

// 命令类型:breath back lights
const byte CODE_LIGHT_BACK = byte(0x52);

const short SHAFT_LENGTH = 151;
const short WHEEL_PERIMETER = 82;
const short GEAR_RATIO = 20;
byte *pcMsg = new byte[10]{ byte(0x00), byte(0x00), byte(0xfa) };
Jt808Util jt808Util;

class LightUtil {
public:

    void setLightControl(int pinCode,
                         int brightData);
    void lightFlash();
};
#include <chrono>
#include <ctime>
using namespace std::chrono;
unsigned long aimTime = 0L;
bool backLeftLightFlash = false;
bool backRightLightFlash = false;


void LightUtil::setLightControl(int pinCode, int brightData) {
    if (0 == brightData) {
        // analogWrite(pinCode, 0);
        cout << "close, pin:" << pinCode << endl;
    } else if (brightData < int(0x64)) {
        // flash light
        // aimTime = 500 * int(brightData) + mills();
        system_clock::time_point time_point_now = system_clock::now();
        system_clock::duration   duration_since_epoch =
            time_point_now.time_since_epoch();
        time_t microseconds_since_epoch = duration_cast<microseconds>(
            duration_since_epoch).count();             // 将时长转换为微秒数
        time_t tm_millisec = microseconds_since_epoch; // 当前时间的毫秒数
        aimTime = 1000000 * brightData + tm_millisec;

        if (pinCode == STEP_LIGHT_BACK_LEFT) {
            backLeftLightFlash = true;
        } else if (pinCode == STEP_LIGHT_BACK_RIGHT) {
            backRightLightFlash = true;
        }
        cout << "flash light start, aimTime:" << aimTime << ", back left: " <<
            backLeftLightFlash << ", back right: " << backRightLightFlash << endl;
    } else {
        // always lightup
        // analogWrite(pinCode, brightData);
        cout << "always lightup, pin:" << pinCode << endl;
    }
}

void LightUtil::lightFlash() {
    // unsigned long nowTime=mills();
    system_clock::time_point time_point_now = system_clock::now();
    system_clock::duration   duration_since_epoch =
        time_point_now.time_since_epoch();
    time_t microseconds_since_epoch = duration_cast<microseconds>(
        duration_since_epoch).count();       // 将时长转换为微秒数
    long nowTime = microseconds_since_epoch; // 当前时间的毫秒数

    if (nowTime >= aimTime) {
        if (backLeftLightFlash) {
            // analogWrite(STEP_LIGHT_BACK_LEFT, 0);
            cout << "close back left light." << endl;
            backLeftLightFlash = false;
        }

        if (backRightLightFlash) {
            // analogWrite(STEP_LIGHT_BACK_RIGHT, 0);
            cout << "close back right light." << endl;
            backRightLightFlash = false;
        }
    } else {
        unsigned long deltaTime = (aimTime - nowTime) / 1000;

        // 3000,2500,2000,1500,1000,500,0
        if (deltaTime % 1000 == 0) {
            if (backLeftLightFlash) {
                // analogWrite(STEP_LIGHT_BACK_LEFT, 255);
                cout << "back left light bright." << deltaTime << endl;
            }

            if (backRightLightFlash) {
                // analogWrite(STEP_LIGHT_BACK_RIGHT, 255);
                cout << "back right light bright." << deltaTime << endl;
            }
        } else if (deltaTime % 500 == 0) {
            if (backLeftLightFlash) {
                // analogWrite(STEP_LIGHT_BACK_LEFT, 0);
                cout << "back left light off." << deltaTime << endl;
            }

            if (backRightLightFlash) {
                // analogWrite(STEP_LIGHT_BACK_RIGHT, 0);
                cout << "back right light off." << deltaTime << endl;
            }
        }
    }
}

LightUtil lightUtil;
int main() {
    int realSize = 10;
    std::byte *cmdCache = (std::byte *)calloc(realSize, sizeof(byte));

    cmdCache[0] = byte(0x7E); // 标识
    cmdCache[1] = byte(0xb0); // 校验码
    cmdCache[2] = byte(0xFA); // 外设类型编号
    cmdCache[3] = byte(0x52); // 命令类型
    cmdCache[4] = byte(0x02); // 用户数据
    cmdCache[5] = byte(0x62);
    cmdCache[6] = byte(0x7e); // 标识

    // 检查有效的数据
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
        // 舵机角度调整
        case CODE_CMD_SERVO:

            if (*(infoDataPointer + 2) > byte(0x4B)) {
                angle = 75;
            } else {
                angle = (int)*(infoDataPointer + 2);
            }

            // 控制舵机转动相应的角度
            // myservo.write(angle);
            cout << "angle: " << angle << endl;
            break;

        // 左侧电机控制
        case CODE_STEP_L:
            moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);

            if (cmdCache[4] == byte(0x01)) {
                // 反转
                moveToPosition = -moveToPosition;
            }

            // stepper_l.setSpeed(5000);
            // stepper_l.move(moveToPosition);
            cout << "left position: " << moveToPosition << endl;
            break;

        // 右侧电机控制
        case CODE_STEP_R:
            moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);

            if (cmdCache[4] == byte(0x01)) {
                // 反转
                moveToPosition = -moveToPosition;
            }

            // stepper_r.setSpeed(5000);
            // stepper_r.move(moveToPosition);
            cout << "right position: " << moveToPosition << endl;
            break;

        // 原地旋转
        case CODE_STEP_ROUND:
            moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);

            // 1/16 0.9degree 6400steps for wheel 1 circle
            // step=L/D*6400*angle/360 L:shaft length,D:wheel perimeter
            // moveToPosition:left wheel
            moveToPosition = 6400L * moveToPosition * GEAR_RATIO * SHAFT_LENGTH
                             / WHEEL_PERIMETER  / 360;

            if (cmdCache[4] == byte(0x01)) {
                // counterclockwise right toward,left backward
                moveToPosition = -moveToPosition;
            }

            // stepper_l.setSpeed(5000);
            // stepper_r.setSpeed(5000);
            // stepper_l.move(moveToPosition);
            // stepper_r.move(-moveToPosition);
            cout << "left position: " << moveToPosition << endl;
            cout << "right position: " << -moveToPosition << endl;
            break;

        // 直线运行
        case CODE_STEP_FORWARD:
            moveToPosition = jt808Util.getMovePosition(infoDataPointer, realSize);

            if (cmdCache[4] == byte(0x01)) {
                // backward
                moveToPosition = -moveToPosition;
            }

            // stepper_l.setSpeed(5000);
            // stepper_r.setSpeed(5000);
            // stepper_l.move(moveToPosition);
            // stepper_r.move(moveToPosition);
            cout << "all position: " << moveToPosition << endl;
            break;

        // stop move
        case CODE_STOP:

            // currentPositionL = stepper_l.currentPosition();
            // currentPositionR = stepper_r.currentPosition();
            // stepper_l.moveTo(currentPositionL + 10 * GEAR_RATIO);
            // stepper_r.moveTo(currentPositionR + 10 * GEAR_RATIO);
            cout << "stop position(left): " <<
            (currentPositionL + 10 * GEAR_RATIO) << endl;
            break;

        // get system status
        case CODE_TICK:

            /**
               if (stepper_l.isRunning() || stepper_r.isRunning()) {
                currentPositionL = stepper_l.distanceToGo();
                currentPositionR = stepper_r.distanceToGo();
                byte *data = jt808Util.getBytesData(currentPositionL, realSize);
                byte *msgData = jt808Util.generateMsgCode(CODE_TYPE,
                                                          data,
                                                          realSize);

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
             */
            currentPositionL = 130901L;
            data = jt808Util.getBytesData(currentPositionL, realSize);
            msgData = jt808Util.generateMsgCode(CODE_TYPE, data, realSize);
            cout << "system status: ";

            for (int i = 0; i < realSize; i++) {
                if (i == 3) {
                    cout << int(CODE_STEP_L) << ",";
                }
                cout << int(msgData[i]) << ",";
            }
            cout << endl;

            if (data != NULL) {
                free(data);
            }

            if (msgData != NULL) {
                free(msgData);
            }
            break;

        // control the front light
        case CODE_LIGHT_FRONT:
            moveToPosition = (int)*(infoDataPointer + 2);

            if (moveToPosition > 255) {
                moveToPosition = 255;
            } else if (moveToPosition < 100) {
                moveToPosition = 0;
            } else {
                moveToPosition = (int)*(infoDataPointer + 2);
            }

            // analogWrite(STEP_LIGHT_FRONT, moveToPosition);
            cout << "front light: " << moveToPosition << endl;
            break;

        // control the back light,7EXXFA52KKSS7E
        // KK:00->left,01->right,02->both; SS:00->close,
        // 01~63->flash for SS times,64~ff->bright strength
        case CODE_LIGHT_BACK:
            lightType = int8_t(*(infoDataPointer + 2));
            angle = int8_t(*(infoDataPointer + 3));

            if (0 == lightType) {
                cout << "light left" << endl;
                lightUtil.setLightControl(STEP_LIGHT_BACK_LEFT, angle);
            } else if (1 == lightType) {
                cout << "light right" << endl;
                lightUtil.setLightControl(STEP_LIGHT_BACK_RIGHT, angle);
            } else if (2 == lightType) {
                cout << "light 2" << endl;
                lightUtil.setLightControl(STEP_LIGHT_BACK_LEFT,  angle);
                lightUtil.setLightControl(STEP_LIGHT_BACK_RIGHT, angle);
            }

            break;
        }

        if (infoDataPointer != NULL) {
            free(infoDataPointer);
            infoDataPointer = NULL;
        }
    }

    //    while (backLeftLightFlash || backLeftLightFlash) {
    //        lightUtil.lightFlash();
    //    }


    /**
       cmdEndTag = false;

       for (int i = 0; i < 10; i++) {
        cmdCache[i] = 0;
       }
     */
    if (cmdCache != NULL) {
        free(cmdCache);
        cmdCache = NULL;
    }
    return 0;
}

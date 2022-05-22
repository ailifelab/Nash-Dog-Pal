#ifndef JT808_UTIL_H
#define JT808_UTIL_H

#include <Arduino.h>
class Jt808Util {
  public:
    Jt808Util();
    // 封装消息体
    byte * generateMsgCode(byte device, byte infoData[], int& realSize);

    // 校验数据
    bool checkData(byte msg[], int& realSize);

    // 获取消息体
    byte* getInfoData(byte msg[], int& realSize);

    // generate position by message
    long getMovePosition(byte * msg, const int realSize);

    // transfer data to bytes array
    byte * getBytesData(long data, int& realSize);
};
#endif

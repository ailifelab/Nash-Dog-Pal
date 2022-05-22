#include "Jt808Util.h"
#include <stdlib.h>
Jt808Util::Jt808Util() {}

// 通讯标识头
const byte CODE_FLAG = byte(0x7e);
const byte CODE_FLAG_TRN = byte(0x7d);

/**
   封装消息体
   @param infoData 消息内容数组指针
   @param realSize 消息内容数组长度
*/
byte * Jt808Util::generateMsgCode(byte device, byte infoData[], int& realSize) {
  // 0x7e 0x7d 转换
  int size7e = 0;
  int size7d = 0;

  for (int i = 0; i < realSize; i++) {
    if (CODE_FLAG == infoData[i]) {
      size7e++;
    } else if (CODE_FLAG_TRN == infoData[i]) {
      size7d++;
    }
  }
  int   infoData2Size = size7e + size7d + realSize + 2;
  byte *infoData2 = new byte[infoData2Size];
  infoData2[1] = device;
  int r = 2;

  for (int i = 0; i < realSize; i++) {
    if (CODE_FLAG_TRN == infoData[i]) {
      infoData2[r] = CODE_FLAG_TRN;
      r++;
      infoData2[r] = byte(0x01);
      r++;
    } else if (CODE_FLAG == infoData[i]) {
      infoData2[r] = CODE_FLAG_TRN;
      r++;
      infoData2[r] = byte(0x02);
      r++;
    } else {
      infoData2[r] = infoData[i];
      r++;
    }
  }

  // 生成校验码 累加和取低8位
  long checkAdd = 0L;

  for (int i = 0; i < infoData2Size; i++) {
    checkAdd += long(infoData2[i]);
  }
  byte  checkCode = byte(checkAdd);
  int   startIndex = 2;
  byte *msg;
  infoData2Size = infoData2Size + 2;

  if (CODE_FLAG == checkCode) {
    startIndex++;
    infoData2Size++;
    msg = (byte *)malloc(infoData2Size);
    msg[1] = CODE_FLAG_TRN;
    msg[2] = byte(0x02);
  } else if (CODE_FLAG_TRN == checkCode) {
    startIndex++;
    infoData2Size++;
    msg = (byte *)malloc(infoData2Size);
    msg[1] = CODE_FLAG_TRN;
    msg[2] = byte(0x01);
  }  else {
    msg = (byte *)malloc(infoData2Size);
    msg[1] = checkCode;
  }
  int j = 1;

  for (int i = startIndex; i < infoData2Size; i++) {
    msg[i] = infoData2[j];
    j++;
  }
  delete[](infoData2);
  msg[0] = CODE_FLAG;
  msg[infoData2Size - 1] = CODE_FLAG;
  realSize = infoData2Size;
  return msg;
}

/**
   检测校验位数据是否正确
   @param msg 消息数组指针
   @param realSize 消息数组长度
*/
bool Jt808Util::checkData(byte msg[], int& realSize) {
  int startIndex = 0;

  if (CODE_FLAG != msg[0]) {
    return false;
  }

  if (CODE_FLAG_TRN == msg[1]) {
    startIndex = 3;
  } else {
    startIndex = 2;
  }
  long checkAdd = 0L;

  for (int i = startIndex; i < realSize; i++) {
    if (CODE_FLAG == msg[i]) {
      break;
    } else {
      checkAdd += long(msg[i]);
    }
  }
  byte checkCode = byte(checkAdd);

  if (checkCode == CODE_FLAG) {
    return msg[1] == CODE_FLAG_TRN && msg[2] == byte(0x02);
  } else if (checkCode == CODE_FLAG_TRN) {
    return msg[1] == CODE_FLAG_TRN && msg[2] == byte(0x01);
  } else {
    return msg[1] == checkCode;
  }
}

/**
   获取消息体
   @param msg 消息数组指针
   @param realSize 消息数组长度
*/
byte * Jt808Util::getInfoData(byte msg[], int& realSize) {
  int startIndex = 2;

  // 获取用户数据长度
  int dataSize = 0;
  int oldSize = 0;

  for (int i = startIndex; i < realSize; i++) {
    if (msg[i] == CODE_FLAG) {
      break;
    } else if (msg[i] == CODE_FLAG_TRN) {
      // do nothing
      oldSize++;
    } else {
      dataSize++;
    }
  }
  realSize = dataSize;

  // 解析用户数据，还原0x7e和0x7d
  byte *infoData = (byte *)calloc(dataSize, sizeof(byte));

  //    byte *infoData = (byte *)malloc(dataSize);
  if (infoData == NULL) {
    return NULL;
  }
  int j = 0;

  for (int i = startIndex; i < startIndex + dataSize + oldSize; i++) {
    if (msg[i] == CODE_FLAG) {
      break;
    } else if (msg[i] == CODE_FLAG_TRN) {
      i++;

      if (msg[i] == byte(0x01)) {
        infoData[j] = CODE_FLAG_TRN;
      } else if (msg[i] == byte(0x02)) {
        infoData[j] = CODE_FLAG;
      }
    } else {
      infoData[j] = *(msg + i);
    }
    j++;
  }
  return infoData;
}

/**
    generate position by message
*/
long Jt808Util::getMovePosition(byte     *msg,
                                const int realSize) {
  int   moveToPosition = 0;
  short startIndex = 0;

  for (int k = realSize - 1; k > 2; k--) {
    moveToPosition +=
      (long(msg[k]) << 8 * startIndex);
    startIndex++;
  }
  return moveToPosition;
}

byte * Jt808Util::getBytesData(long data, int& realSize) {
  byte tag = byte(0x00);

  if (data < 0) {
    tag = byte(0x01);
    data = -data;
  }
  byte *value = (byte *)malloc(8);
  int   index0 = 0;
  bool  end = false;

  for (int i = 7; i >= 0; i--) {
    value[7 - i] = byte(data  >> i * 8);

    if (int(value[7 - i]) != 0) {
      end = true;
    } else if ((!end) && (int(value[7 - i]) == 0)) {
      index0++;
    }
  }
  realSize = 9 - index0;
  byte *result = (byte *)malloc(realSize);
  result[0] = tag;

  for (int i = index0; i < 8; i++) {
    result[i - index0 + 1] = value[i];
  }

  if (value != NULL) {
    free(value);
    value = NULL;
  }
  return result;
}

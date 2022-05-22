# generate jt808 based code.7E00FA44007E
import numpy as np

# 通讯标识 标识 校验码 外设类型编号 命令类型 用户数据 标识
CODE_FLAG = 0x7E
# 外设类型编号
CODE_TYPE = 0xFA
# 命令类型：舵机角度调整 7E00FA44007E
CODE_CMD_SERVO = 0x44
# 命令类型：左步进电机旋转 7E00FA450008007E
CODE_STEP_L = 0x45
# 命令类型：右步进电机旋转 7E00FA460008007E
CODE_STEP_R = 0x46
# 命令类型：原地旋转
CODE_STEP_ROUND = 0x47
# 命令类型：直线行进
CODE_STEP_FORWARD = 0x48
# 命令类型：停止电机(10*减速比脉冲数)
CODE_STEP = 0x49
# 命令类型：查询电机状态
CODE_TICK = 0x50
# 命令类型：调节LED亮度
CODE_LIGHT_FRONT = 0x51
# 命令类型: 调节后部LED亮度
CODE_LIGHT_BACK = 0x52
# 命令类型:按钮动作
CODE_BTN = 0x53
# 命令类型:获取坐标
CODE_TICK_GYRO = 0x54;
# 初始化命令行
data = np.array([0x7E, 0x00], dtype="uint8")
# 输入外设类型编号，不需要输入
m = input("输入外设类型编号（默认0xfa):")
if m == "":
    n = np.array([CODE_TYPE], dtype="uint8")
else:
    n = bytearray.fromhex(m)
for i in range(len(n)):
    if n[i] == 126:
        data = np.append(data, 0x7D)
        data = np.append(data, 0x02)
    elif n[i] == 125:
        data = np.append(data, 0x7D)
        data = np.append(data, 0x01)
    else:
        data = np.append(data, n[i])
# 选择命令类型
print("请选择命令模式:")
print("1.舵机角度调整")
print("2.左步进电机旋转")
print("3.右步进电机旋转")
print("4.原地旋转")
print("5.直线行进")
print("6.立即停止电机")
print("7.获取电机状态")
print("8.调节LED亮度")
print("9.调节后部LED亮度")
print("10.按钮动作")
print("11.获取坐标")
n = int(input("选择："))
if 1 == n:
    data = np.append(data, CODE_CMD_SERVO)
elif 2 == n:
    data = np.append(data, CODE_STEP_L)
elif 3 == n:
    data = np.append(data, CODE_STEP_R)
elif 4 == n:
    data = np.append(data, CODE_STEP_ROUND)
elif 5 == n:
    data = np.append(data, CODE_STEP_FORWARD)
elif 6 == n:
    data = np.append(data, CODE_STEP)
elif 7 == n:
    data = np.append(data, CODE_TICK)
elif 8 == n:
    data = np.append(data, CODE_LIGHT_FRONT)
elif 9 == n:
    data = np.append(data, CODE_LIGHT_BACK)
    print("7EXXFA52KKSS7E")
elif 10 == n:
    data = np.append(data, CODE_BTN)
elif 11 == n:
    data = np.append(data, CODE_TICK_GYRO)
# 输入要发送的数据命令
m = input("输入用户数据:")
n = bytearray.fromhex(m)
for i in range(len(n)):
    if n[i] == 126:
        data = np.append(data, 0x7D)
        data = np.append(data, 0x02)
    elif n[i] == 125:
        data = np.append(data, 0x7D)
        data = np.append(data, 0x01)
    else:
        data = np.append(data, n[i])
# 截取要计算校验码的片段，并计算校验码
newArray = data[2:]
checkInt = np.sum(newArray)
checkBytes = checkInt.astype("uint8")
# 创建有校验码的数组
data = np.array([CODE_FLAG, checkBytes], dtype="uint8")
# 合并数据
data = np.append(data, newArray)
data = np.append(data, CODE_FLAG)
print("串口发送的HEX：" + "".join("{:02X}".format(i) for i in data))

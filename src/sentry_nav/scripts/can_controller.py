#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import can
import struct
from geometry_msgs.msg import Twist

# 全局变量
can_bus = None

# ====================== 配置参数（从协议文档提取）======================
NODE_ID = 1                  # CANopen 节点ID（默认1，可通过上位机修改）
CAN_BAUDRATE = 500000       # 波特率
SPEED_INDEX = 0x2000        # 速度控制索引
SUBINDEX_LEFT = 0x01        # 左轮（通道1）子索引
SUBINDEX_RIGHT = 0x02       # 右轮（通道2）子索引
MAX_SPEED_CMD = 1000        # 最大速度指令值（协议规定）
WHEEL_BASE = 0.6           # 差速小车轮距（m，根据实际调整）
# ======================================================================

def constrain(value, min_val, max_val):
    """限制数值范围，避免超出协议上限"""
    return max(min_val, min(value, max_val))

def sdo_write_speed(node_id, subindex, speed_cmd):
    """
    发送 CANopen SDO 写消息，控制指定通道电机速度
    :param node_id: CANopen 节点ID
    :param subindex: 子索引（01=左轮，02=右轮）
    :param speed_cmd: 速度指令（-1000~1000）
    """
    global can_bus

    # 1. 构造 SDO 写消息数据
    # Data[0] = 0x23（4字节数据标识）
    # Data[1-2] = 索引（低字节在前，0x2000 → 0x00, 0x20）
    # Data[3-4] = 速度指令（低字节在前，S16 格式）
    # Data[5-7] = 0x00（保留）
    index_low = SPEED_INDEX & 0xFF
    index_high = (SPEED_INDEX >> 8) & 0xFF
    speed_bytes = struct.pack('<h', speed_cmd)  # 转为小端序16位有符号整数
    can_data = [
        0x23,
        index_low, index_high,
        subindex,
        speed_bytes[0], speed_bytes[1],
        0x00, 0x00
    ]
    # can_data = [0x23,0x00,0x20,0x01,0xfe,0x01,0x00,0x00]
    rospy.loginfo(can_data)

    # 2. 构造 CAN 帧（标准帧）
    can_id = 0x601
    try:
        msg = can.Message(
            arbitration_id=can_id,
            data=can_data,
            is_extended_id=False
        )
        can_bus.send(msg)
        rospy.loginfo(
            "SDO发送：节点ID=%d, 子索引=0x%02X, 速度指令=%d, CAN数据=%s",
            node_id, subindex, speed_cmd, [hex(x) for x in can_data]
        )
    except can.CanError as e:
        rospy.logerr("CAN发送失败：%s", str(e))

def cmd_vel_callback(msg):
    """订阅 /cmd_vel 话题，执行差速逆解并发送 CANopen 指令"""
    # 1. 解析线速度和角速度
    v_x = msg.linear.x       # 前进/后退线速度（m/s）
    omega_z = msg.angular.z  # 左转/右转角速度（rad/s）
    rospy.loginfo("接收速度指令：线速度=%.2fm/s, 角速度=%.2frad/s", v_x, omega_z)

    # 2. 差速运动学逆解（计算左右轮速度指令比例）
    # 逻辑：线速度对应基础指令，角速度对应差速指令
    left_speed_ratio = v_x - (omega_z * WHEEL_BASE / 2.0)
    right_speed_ratio = v_x + (omega_z * WHEEL_BASE / 2.0)

    # 3. 归一化到协议指令范围（-1000~1000）
    # 假设最大线速度对应指令1000，可根据实际速度标定调整比例系数
    max_phy_speed = 1.0  # 小车最大物理速度（m/s，需根据实际标定）
    left_cmd = constrain(int(left_speed_ratio / max_phy_speed * MAX_SPEED_CMD), -MAX_SPEED_CMD, MAX_SPEED_CMD)
    right_cmd = constrain(int(right_speed_ratio / max_phy_speed * MAX_SPEED_CMD), -MAX_SPEED_CMD, MAX_SPEED_CMD)

    # 4. 发送 CANopen 速度指令
    sdo_write_speed(NODE_ID, SUBINDEX_LEFT, left_cmd)
    sdo_write_speed(NODE_ID, SUBINDEX_RIGHT, right_cmd)

def init_canopen_bus():
    """初始化 CANopen 总线"""
    global can_bus
    try:
        can_bus = can.interface.Bus(
            channel='can0',  # 真实硬件用can0，测试用vcan0
            bustype='socketcan',
            bitrate=CAN_BAUDRATE
        )
        rospy.loginfo("CANopen总线初始化成功：通道can0，波特率%d", CAN_BAUDRATE)
        return True
    except Exception as e:
        rospy.logfatal("CANopen总线初始化失败：%s", str(e))
        return False

def main():
    global can_bus

    # 1. 初始化 ROS 节点
    rospy.init_node('canopen_driver', anonymous=True)
    rospy.loginfo("CANopen 差速小车控制节点启动...")

    # 2. 初始化 CANopen 总线
    if not init_canopen_bus(): 
        return

    # 3. 订阅 /cmd_vel 话题
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback, queue_size=10)

    # 4. 保持节点运行
    rospy.spin()

    # 5. 关闭 CAN 总线
    if can_bus:
        can_bus.shutdown()
        rospy.loginfo("CANopen总线已关闭")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("节点被中断")

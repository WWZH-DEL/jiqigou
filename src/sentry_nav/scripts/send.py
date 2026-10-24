#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

# 串口配置
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际串口设备修改
BAUD_RATE = 115200

# 消息类型定义
ROS_MSG_TYPE_CMD_VEL = 1  # 速度控制消息类型




class CmdVelToStm32:
    def __init__(self):
        """初始化节点和串口"""
        rospy.init_node('cmd_vel_to_stm32', anonymous=True)
        
        # 订阅cmd_vel话题
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 打开串口
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            rospy.loginfo(f"串口已打开: {SERIAL_PORT}")
        except Exception as e:
            rospy.logerr(f"无法打开串口: {e}")
            rospy.signal_shutdown("串口连接失败")
    
    def calculate_checksum(self, data):
        """计算校验和（异或校验）"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def cmd_vel_callback(self, msg):
        """处理接收到的cmd_vel消息"""
        try:
            # 提取速度值
            linear_x = msg.linear.x 
            linear_y = 0
            angular_z = -msg.angular.z
            
            # 构建消息数据（速度控制消息结构体）
            data = struct.pack('<fff', linear_x, linear_y, angular_z)
            
            # 计算数据长度
            data_len = len(data)
            
            # 构建消息头和数据
            msg_buffer = bytearray()
            msg_buffer.append(0xAA)  # 起始字节
            msg_buffer.append(ROS_MSG_TYPE_CMD_VEL)  # 消息类型
            msg_buffer.append(data_len & 0xFF)  # 数据长度低8位
            msg_buffer.append((data_len >> 8) & 0xFF)  # 数据长度高8位
            msg_buffer.extend(data)  # 数据部分
            
            # 计算校验和
            checksum = self.calculate_checksum(msg_buffer[1:])  # 从消息类型开始计算
            msg_buffer.append(checksum)
            
            # 发送消息
            self.ser.write(msg_buffer)
            rospy.loginfo(f"发送速度指令: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}")
            
        except Exception as e:
            rospy.logerr(f"发送消息失败: {e}")
    
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        finally:
            # 关闭串口
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                rospy.loginfo("串口已关闭")

if __name__ == '__main__':
    try:
        node = CmdVelToStm32()
        node.run()
    except rospy.ROSInterruptException:
        pass

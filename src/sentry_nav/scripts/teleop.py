#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 自定义初始速度（核心：直接设置，无需修改其他文件）
INIT_LINEAR_SPEED = 0.6  # 初始线速度（m/s）
INIT_ANGULAR_SPEED = 0.4 # 初始角速度（rad/s）

# 按键映射
key_bindings = {
    'i': (INIT_LINEAR_SPEED, 0),        # 前进
    ',': (-INIT_LINEAR_SPEED, 0),       # 后退
    'j': (0, INIT_ANGULAR_SPEED),       # 左转
    'l': (0, -INIT_ANGULAR_SPEED),      # 右转
    'k': (0, 0),                        # 停止
}

def get_key():
    """获取键盘输入"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

def main():
    # 初始化ROS节点
    rospy.init_node('custom_teleop', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率（与原工具一致）

    twist = Twist()
    rospy.loginfo("自定义键盘控制节点启动...")
    rospy.loginfo(f"初始速度：线速度={INIT_LINEAR_SPEED}m/s，角速度={INIT_ANGULAR_SPEED}rad/s")
    rospy.loginfo("按键说明：i=前进, ,=后退, j=左转, l=右转, k=停止, Ctrl+C=退出")

    while not rospy.is_shutdown():
        key = get_key()
        if key in key_bindings:
            linear, angular = key_bindings[key]
            twist.linear.x = linear
            twist.angular.z = angular
        elif key == '\x03':  # Ctrl+C退出
            break

        # 发布速度指令
        pub.publish(twist)
        rate.sleep()

    # 退出时发送零速度指令
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)
    rospy.loginfo("节点退出，小车停止\n")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


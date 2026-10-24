#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import lcm
from cmd_vel_msg import cmd_vel_msg

# 初始化lcm
lc = lcm.LCM()

def cmd_vel_cb(msg: Twist):
    # 封装lcm数据
    lcm_data = cmd_vel_msg()
    lcm_data.vx = msg.linear.x
    lcm_data.wz = msg.angular.z
    # lcm发布，通道名 CMD_VEL_LCM
    lc.publish("CMD_VEL_LCM", lcm_data.encode())
    rospy.loginfo("vx=%.3f, wz=%.3f", lcm_data.vx, lcm_data.wz)

if __name__ == "__main__":
    rospy.init_node("cmdvel_to_lcm_py")
    sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb, queue_size=10)
    rospy.loginfo("Subscribe /cmd_vel and forward to LCM channel: CMD_VEL_LCM")
    rospy.spin()

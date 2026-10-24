#!/usr/bin/env python3
import rospy
import yaml
import os
from geometry_msgs.msg import PoseStamped

waypoints = []
# 固定绝对路径，避免保存位置错乱
filename = "src/sentry_nav/config/waypoint.yaml"

def goal_callback(msg):
    """仅保存位置+原始四元数，无任何计算"""
    wp = {
        "x": msg.pose.position.x,    # 位置x
        "y": msg.pose.position.y,    # 位置y
        "ox": msg.pose.orientation.x,# 四元数x
        "oy": msg.pose.orientation.y,# 四元数y
        "oz": msg.pose.orientation.z,# 四元数z
        "ow": msg.pose.orientation.w # 四元数w
    }
    waypoints.append(wp)
    rospy.loginfo(f"已添加第{len(waypoints)}个点 → x:{wp['x']:.2f}, y:{wp['y']:.2f}")

def save_waypoints():
    if not waypoints:
        rospy.logwarn("无导航点可保存！")
        return
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'w') as f:
        yaml.dump({"waypoints": waypoints}, f, default_flow_style=False)
    rospy.loginfo(f"✅ 共{len(waypoints)}个点保存完成 → {filename}")
    rospy.signal_shutdown("保存完成，退出")

if __name__ == '__main__':
    rospy.init_node('waypoint_saver_node')
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.on_shutdown(save_waypoints)
    rospy.loginfo("📌 捕获RViz导航点（仅存位置+四元数，Ctrl+C保存）")
    rospy.spin()


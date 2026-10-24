#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from mbf_msgs.msg import GetPathAction, GetPathGoal, ExePathAction, ExePathGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class NavCommander:
    def __init__(self):
        rospy.init_node('nav_commander')
        
        # --- ⚠️ 修改点 1：Action 客户端路径 ---
        # 因为 Launch 文件里 name="move_base1"，所以路径前缀要加 1
        self.path_client = actionlib.SimpleActionClient('/move_base1/get_path', GetPathAction)
        self.exe_client = actionlib.SimpleActionClient('/move_base1/exe_path', ExePathAction)
        
        rospy.loginfo("Waiting for MBF action servers at /move_base1...")
        self.path_client.wait_for_server()
        self.exe_client.wait_for_server()
        
        # 订阅 RViz 目标点
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        
        # 参数定义
        self.MODE_FAST = 4.0
        self.MODE_SLOW = 0.8
        self.CURVATURE_THRESHOLD = 0.5
        
        rospy.loginfo("NavCommander Ready. Waiting for goal...")

    def goal_callback(self, msg):
        """当 RViz 点击目标时触发"""
        rospy.loginfo("收到目标点: [%.2f, %.2f]", msg.pose.position.x, msg.pose.position.y)
        self.navigate(msg)

    def calculate_curvature(self, path):
        # 计算路径曲率逻辑 (保持不变)
        if len(path.poses) < 10: return 0.0
        lookahead_dist = 1.5
        points = []
        current_dist = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            current_dist += dist
            points.append((p1.x, p1.y))
            if current_dist >= lookahead_dist: break
        if len(points) < 2: return 0.0
        start_p = points[0]
        end_p = points[-1]
        chord_len = math.sqrt((end_p[0] - start_p[0])**2 + (end_p[1] - start_p[1])**2)
        if chord_len < 0.1: return 10.0
        return (current_dist - chord_len) / chord_len

    def set_planner_speed(self, max_vel_x):
        # --- ⚠️ 修改点 2：动态参数服务路径 ---
        # 命名空间必须匹配：/节点名/规划器名
        try:
            from dynamic_reconfigure.client import Client
            # 注意这里也是 move_base1
            client = Client("/move_base1/TebLocalPlanner", timeout=2.0)
            config = client.update_configuration({"max_vel_x": max_vel_x})
            rospy.loginfo(f"速度已调整为: {max_vel_x} m/s")
        except Exception as e:
            rospy.logerr(f"参数更新失败 (请检查命名空间): {e}")

    def navigate(self, target_pose):
        # 1. 规划路径
        goal = GetPathGoal()
        goal.target_pose = target_pose
        goal.use_start_pose = False
        
        self.path_client.send_goal(goal)
        self.path_client.wait_for_result()
        
        if self.path_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("规划失败！")
            return

        # 2. 分析曲率并调速
        path = self.path_client.get_result().path
        curvature = self.calculate_curvature(path)
        
        if curvature > self.CURVATURE_THRESHOLD:
            rospy.loginfo("检测到急弯 -> 减速模式")
            self.set_planner_speed(self.MODE_SLOW)
        else:
            rospy.loginfo("检测到直道 -> 加速模式")
            self.set_planner_speed(self.MODE_FAST)

        # 3. 执行路径
        exe_goal = ExePathGoal()
        exe_goal.path = path
        self.exe_client.send_goal(exe_goal)
        self.exe_client.wait_for_result()

if __name__ == '__main__':
    try:
        commander = NavCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

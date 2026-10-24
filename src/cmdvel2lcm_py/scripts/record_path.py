#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import os
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ActualPathRecorder:
    def __init__(self):
        rospy.init_node('actual_path_recorder', anonymous=True)
        
        # 固定保存路径（按你的要求指定）
        self.save_path = "/home/wzh/catkin_ws/src/sentry_nav/save_path/recorded_path.json"
        self.record_rate = rospy.get_param('~record_rate', 10)  # 录制频率（Hz）
        self.pos_threshold = rospy.get_param('~pos_threshold', 0.05)  # 位置阈值（米），小于此值视为重复
        self.ori_threshold = rospy.get_param('~ori_threshold', 0.03)  # 姿态阈值（弧度），小于此值视为重复
        self.recorded_poses = []  # 存储录制的位姿
        self.recording = False    # 录制状态标记
        self.last_record_time = 0.0  # 上一次录制的时间（控制频率）
        
        # 订阅实际路径话题
        self.path_sub = rospy.Subscriber('/actual_path', Path, self.path_callback)
        
        # 注册退出回调（确保节点关闭时保存数据）
        rospy.on_shutdown(self.save_recorded_path)
        
        rospy.loginfo("路径录制节点已启动，等待/actual_path话题数据...")
        rospy.loginfo(f"录制数据将保存至: {self.save_path}")

    def pose_distance(self, pose1, pose2):
        """
        计算两个位姿的偏差（位置欧氏距离 + 姿态四元数夹角）
        返回：(位置距离, 姿态夹角)
        """
        # 计算位置距离（欧氏距离）
        pos_dist = math.hypot(
            pose1['x'] - pose2['x'],
            pose1['y'] - pose2['y'],
            pose1['z'] - pose2['z']
        )
        
        # 计算四元数夹角（姿态偏差）
        dot = pose1['qx']*pose2['qx'] + pose1['qy']*pose2['qy'] + pose1['qz']*pose2['qz'] + pose1['qw']*pose2['qw']
        dot = min(1.0, max(-1.0, dot))  # 防止数值溢出
        ori_dist = 2 * math.acos(abs(dot))
        
        return pos_dist, ori_dist

    def path_callback(self, msg):
        """接收/actual_path话题回调，记录位姿数据（去重+频率控制）"""
        if not self.recording:
            self.recording = True
            rospy.loginfo("开始录制路径...")
        
        # 1. 控制录制频率：只在达到指定频率时处理数据
        current_time = rospy.get_time()
        if current_time - self.last_record_time < 1.0 / self.record_rate:
            return
        
        # 2. 提取最新的位姿（取Path最后一个PoseStamped，避免重复遍历）
        if not msg.poses:
            return
        latest_pose = msg.poses[-1]
        pose_data = {
            'x': latest_pose.pose.position.x,
            'y': latest_pose.pose.position.y,
            'z': latest_pose.pose.position.z,
            'qx': latest_pose.pose.orientation.x,
            'qy': latest_pose.pose.orientation.y,
            'qz': latest_pose.pose.orientation.z,
            'qw': latest_pose.pose.orientation.w,
            'header': {
                'seq': latest_pose.header.seq,
                'stamp': latest_pose.header.stamp.to_sec(),
                'frame_id': latest_pose.header.frame_id
            }
        }
        
        # 3. 去重判断：无历史数据 或 偏差超过阈值才保存
        if not self.recorded_poses:
            self.recorded_poses.append(pose_data)
            self.last_record_time = current_time
        else:
            last_pose = self.recorded_poses[-1]
            pos_dist, ori_dist = self.pose_distance(pose_data, last_pose)
            # 位置偏差或姿态偏差超过阈值，才记录新点
            if pos_dist > self.pos_threshold or ori_dist > self.ori_threshold:
                self.recorded_poses.append(pose_data)
                self.last_record_time = current_time

    def save_recorded_path(self):
        """保存录制的路径到JSON文件"""
        if not self.recorded_poses:
            rospy.logwarn("未录制到任何路径数据，跳过保存")
            return
        
        # 确保保存目录存在
        save_dir = os.path.dirname(self.save_path)
        if save_dir and not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        # 写入JSON文件（固定保存路径）
        with open(self.save_path, 'w') as f:
            json.dump({
                'frame_id': 'map',  # 统一使用map坐标系
                'poses': self.recorded_poses,
                'record_time': rospy.get_time()
            }, f, indent=4)
        
        rospy.loginfo(f"路径录制完成！共录制 {len(self.recorded_poses)} 个位姿点")
        rospy.loginfo(f"数据已保存至: {os.path.abspath(self.save_path)}")

    def run(self):
        """节点主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        recorder = ActualPathRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("路径录制节点被中断")
    except Exception as e:
        rospy.logerr(f"录制节点异常: {str(e)}")


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class RecordedPathPublisher:
    def __init__(self):
        rospy.init_node('recorded_path_publisher', anonymous=True)
        
        # ===================== 固定读取路径（和录制节点完全一样）=====================
        self.load_path = "/home/nuc-admin/catkin_ws/src/sentry_nav/save_path/recorded_path.json"   # 这里必须和录制节点路径一致  recorded_path3  recorded_out
        
        # 其他参数也固定
        self.publish_topic = "/global_path"
        self.publish_rate = 10
        self.loop_publish = True  # 自动循环发布，不用命令行设置
        
        # 初始化发布器
        self.path_pub = rospy.Publisher(self.publish_topic, Path, queue_size=10)
        
        # 加载录制的路径数据
        self.recorded_path = self.load_recorded_path()
        if not self.recorded_path:
            rospy.logerr("加载路径失败，节点退出")
            return
        
        rospy.loginfo(f"成功加载路径文件: {self.load_path}")
        rospy.loginfo(f"共加载 {len(self.recorded_path['poses'])} 个位姿点")
        rospy.loginfo(f"将发布至话题: {self.publish_topic} (循环发布)")

    def load_recorded_path(self):
        """从JSON文件加载录制的路径数据"""
        if not os.path.exists(self.load_path):
            rospy.logerr(f"路径文件不存在: {self.load_path}")
            return None
        
        try:
            with open(self.load_path, 'r') as f:
                data = json.load(f)
            return data
        except json.JSONDecodeError:
            rospy.logerr(f"路径文件格式错误: {self.load_path}")
            return None
        except Exception as e:
            rospy.logerr(f"加载路径异常: {str(e)}")
            return None

    def publish_path(self):
        """发布加载的路径数据"""
        rate = rospy.Rate(self.publish_rate)
        
        # 构造Path消息
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = self.recorded_path['frame_id']
        path_msg.header.stamp = rospy.Time.now()
        
        # 填充位姿数据
        for pose_data in self.recorded_path['poses']:
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = pose_data['header']['seq']
            pose_stamped.header.stamp = rospy.Time.from_sec(pose_data['header']['stamp'])
            pose_stamped.header.frame_id = pose_data['header']['frame_id']
            
            # 位置
            pose_stamped.pose.position.x = pose_data['x']
            pose_stamped.pose.position.y = pose_data['y']
            pose_stamped.pose.position.z = pose_data['z']
            
            # 姿态
            pose_stamped.pose.orientation.x = pose_data['qx']
            pose_stamped.pose.orientation.y = pose_data['qy']
            pose_stamped.pose.orientation.z = pose_data['qz']
            pose_stamped.pose.orientation.w = pose_data['qw']
            
            path_msg.poses.append(pose_stamped)
        
        # 循环发布
        while not rospy.is_shutdown():
            path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(path_msg)
            rate.sleep()

    def run(self):
        if self.recorded_path:
            self.publish_path()
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = RecordedPathPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("路径发布节点被中断")
    except Exception as e:
        rospy.logerr(f"发布节点异常: {str(e)}")


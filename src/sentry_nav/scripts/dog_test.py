#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from threading import Lock

class PurePursuitController:
    def __init__(self):
        # ===================== 机器狗运动核心参数 =====================
        self.Ld = 0.8          # 前视距离（适配机器狗尺寸，减小以提高精度）
        self.min_v = 0.4       # 机器狗最低前进速度（低于此值无法运动）
        self.max_v = 1.0       # 最大前进/后退速度（降低适配机器狗稳定性）
        self.max_vy = 0.3      # 最大横向侧移速度（左右）
        self.max_w = 0.8       # 最大角速度（适配机器狗转向特性）
        self.goal_tolerance = 0.15  # 目标点停止阈值（更精准）
        self.turn_yaw_tolerance = 0.1 # 角度误差阈值（弧度，提高转向精度）
        
        # 避障相关参数（适配机器狗）
        self.obstacle_stop_distance = 0.8  # 前方停止距离（缩短适配机器狗尺寸）
        self.obstacle_angle_range = math.radians(20)  # 检测角度范围（扩大至±20度）
        self.obstacle_min_points = 2  # 判定有障碍物的最小点数
        self.obstacle_detected = False  # 障碍物检测标记
        self.obstacle_lock = Lock()  # 线程锁
        
        # 状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.global_path = []
        self.is_turning = False  # 调头状态标记
        self.current_path_idx = 0  # 当前路径索引
        self.reached_goal = False  # 目标点到达标记

    def update_state(self, x, y, yaw):
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def set_path(self, path):
        self.global_path = path
        self.is_turning = False  # 新路径到来，重置调头状态
        self.current_path_idx = 0  # 重置循环索引
        self.reached_goal = False  # 重置目标点到达标记

    def update_obstacle_status(self, detected):
        """更新障碍物检测状态（线程安全）"""
        with self.obstacle_lock:
            self.obstacle_detected = detected

    def get_obstacle_status(self):
        """获取障碍物检测状态（线程安全）"""
        with self.obstacle_lock:
            return self.obstacle_detected

    def solve(self):
        # 1. 优先级1：障碍物检测 - 有障碍物则全停
        if self.get_obstacle_status():
            return 0.0, 0.0, 0.0

        # 2. 优先级2：无路径/到达目标点 - 全停
        if not self.global_path or self.reached_goal:
            return 0.0, 0.0, 0.0

        # 3. 计算到最终目标点的距离（精准停止逻辑）
        gx, gy = self.global_path[-1]
        dist_goal = np.hypot(self.current_x - gx, self.current_y - gy)
        if dist_goal < self.goal_tolerance:
            self.reached_goal = True
            return 0.0, 0.0, 0.0  # 到达目标点，全停

        # 4. 找到路径上最近的点（提高跟踪精度）
        nearest_idx = self._find_nearest_idx()
        if nearest_idx < 0:
            return 0.0, 0.0, 0.0

        # 5. 寻找前视点（循环搜索，适配机器狗短前视距离）
        target_idx = self._find_lookahead_idx_cyclic(nearest_idx)
        if target_idx < 0:
            return 0.0, 0.0, 0.0

        # 6. 目标点坐标
        tx, ty = self.global_path[target_idx]

        # ===================== 全向运动坐标转换 ======================
        # 将前视点转换到机器人局部坐标系（全向运动核心）
        dx = tx - self.current_x
        dy = ty - self.current_y
        # 局部坐标系：x=前进/后退，y=左/右侧移
        local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

        # ===================== 调头/转向逻辑（适配全向运动） ======================
        # 判定：前视点在后方且距离目标点远，触发转向（机器狗无需原地调头，用横向+旋转）
        if local_x < -0.1 and dist_goal > self.goal_tolerance * 2:
            self.is_turning = True
        else:
            self.is_turning = False

        if self.is_turning:
            # 计算目标偏航角
            target_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
            yaw_error = target_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            # 转向时：前进速度0，横向速度0，仅旋转
            v_cmd = 0.0
            vy_cmd = 0.0
            if abs(yaw_error) > self.turn_yaw_tolerance:
                w_cmd = np.clip(yaw_error * 1.5, -self.max_w, self.max_w)
            else:
                self.is_turning = False
                w_cmd = 0.0
            return v_cmd, vy_cmd, w_cmd

        # ===================== 全向纯追踪核心计算 ======================
        # 1. 前进/后退速度控制（最低0.4）
        if abs(local_x) < 0.05:  # 前视点过近，前进速度0
            v_cmd = 0.0
        else:
            # 按前视点距离比例计算速度，保证最低0.4
            v_cmd = self.max_v * (local_x / abs(local_x))  # 方向由local_x符号决定
            # 限制速度范围（最低0.4，最高max_v）
            v_cmd = np.clip(v_cmd, self.min_v if v_cmd > 0 else -self.max_v, 
                           self.max_v if v_cmd > 0 else -self.min_v)

        # 2. 横向侧移速度控制（左右）
        # 根据局部y方向偏差计算侧移速度，比例控制
        vy_cmd = self.max_vy * (local_y / abs(local_y) if abs(local_y) > 0.02 else 0)
        vy_cmd = np.clip(vy_cmd, -self.max_vy, self.max_vy)

        # 3. 角速度控制（优化版纯追踪公式，适配全向运动）
        if abs(local_x) < 0.01:
            w_cmd = 0.0
        else:
            # 全向运动的角速度计算，降低比例以提高稳定性
            kappa = 2 * local_y / (self.Ld ** 2) if abs(self.Ld) > 0.01 else 0
            w_cmd = kappa * self.max_v
            w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

        # 最终速度校验：如果前进速度低于最小值，强制置0（机器狗无法运动）
        if 0 < abs(v_cmd) < self.min_v:
            v_cmd = 0.0

        return v_cmd, vy_cmd, w_cmd

    # 优化最近点查找（提高精度，增加距离阈值过滤）
    def _find_nearest_idx(self):
        if not self.global_path:
            return -1
        min_dist = float('inf')
        nearest_idx = 0
        for i, (x, y) in enumerate(self.global_path):
            dist = np.hypot(self.current_x - x, self.current_y - y)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        # 过滤过远的最近点（防止路径跳变）
        if min_dist > 2.0:
            return -1
        return nearest_idx

    # 循环前视点搜索（适配机器狗短前视距离）
    def _find_lookahead_idx_cyclic(self, start_idx):
        if not self.global_path:
            return -1
        
        path_len = len(self.global_path)
        cumulative_dist = 0.0
        target_idx = start_idx
        
        # 从最近点开始循环搜索
        for i in range(start_idx, start_idx + path_len):
            current_i = i % path_len
            next_i = (i + 1) % path_len
            
            x1, y1 = self.global_path[current_i]
            x2, y2 = self.global_path[next_i]
            
            segment_dist = np.hypot(x2 - x1, y2 - y1)
            cumulative_dist += segment_dist
            
            # 达到前视距离立即返回
            if cumulative_dist >= self.Ld:
                target_idx = next_i
                break
        
        return target_idx

class TrackerNode:
    def __init__(self):
        self.controller = PurePursuitController()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.reached_pub = rospy.Publisher('/reached', Bool, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        
        # 避障延迟参数
        self.last_obstacle_time = rospy.Time(0)
        self.obstacle_clear_delay = 0.3  # 缩短延迟适配机器狗响应速度
        
        self.rate = rospy.Rate(30)  # 提高发布频率（30Hz适配机器狗控制）
        self.reached_msg = Bool()

    def path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.controller.set_path(pts)

    def scan_cb(self, msg):
        """激光雷达回调：适配机器狗的障碍物检测"""
        obstacle_detected = False
        valid_points = 0
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        
        # 遍历前方±20度激光点
        for i, r in enumerate(ranges):
            current_angle = angle_min + i * angle_increment
            if abs(current_angle) < self.controller.obstacle_angle_range:
                # 过滤无效数据和安全距离外的点
                if not np.isinf(r) and not np.isnan(r) and r < self.controller.obstacle_stop_distance:
                    valid_points += 1
        
        # 更新障碍物状态
        if valid_points >= self.controller.obstacle_min_points:
            obstacle_detected = True
            self.last_obstacle_time = rospy.Time.now()
        else:
            if (rospy.Time.now() - self.last_obstacle_time).to_sec() > self.obstacle_clear_delay:
                obstacle_detected = False
        
        self.controller.update_obstacle_status(obstacle_detected)

    def get_pose(self):
        """获取机器狗位姿（增加容错）"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'body_foot', rospy.Time(0), rospy.Duration(0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # 四元数转偏航角（优化计算精度）
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except Exception as e:
            rospy.logdebug(f"获取位姿失败: {e}")
            return None

    def run(self):
        """主循环：适配机器狗的控制频率和响应速度"""
        while not rospy.is_shutdown():
            pose = self.get_pose()
            if not pose:
                # 位姿获取失败时发布停止指令
                twist = Twist()
                self.cmd_pub.publish(twist)
                self.rate.sleep()
                continue
                
            x, y, yaw = pose
            self.controller.update_state(x, y, yaw)
            # 获取全向速度指令：前进v，横向vy，旋转w
            v, vy, w = self.controller.solve()
            
            # 发布全向速度指令
            twist = Twist()
            twist.linear.x = v      # 前进/后退速度
            twist.linear.y = vy     # 左右侧移速度（机器狗全向核心）
            twist.angular.z = w     # 旋转角速度
            self.cmd_pub.publish(twist)
            
            # 发布到达状态（调试用）
            self.reached_msg.data = self.controller.reached_goal
            self.reached_pub.publish(self.reached_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('dog_pure_pursuit_tracker')
    try:
        TrackerNode().run()
    except rospy.ROSInterruptException:
        # 异常时发布停止指令
        twist = Twist()
        rospy.Publisher('/cmd_vel', Twist, queue_size=1).publish(twist)
        pass


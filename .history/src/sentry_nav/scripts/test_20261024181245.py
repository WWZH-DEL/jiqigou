#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全局路径跟踪控制器（纯追踪，不执行避障）
直接跟踪 /global_path，不依赖 A* 局部规划绕障节点
=====================================================
架构：
  PurePursuitController — 核心控制器
  ├─ [定位]    解析前视目标点
  ├─ [调头]    严格判定 → 原地转向
  ├─ [追踪]    纯追踪 + 曲率自适应 + 低通滤波
"""
import rospy
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
from enum import Enum, auto
# ==============================================================================
# 工具函数
# ==============================================================================
def normalize_angle(a):
    """归一化到 [-π, π]"""
    return math.atan2(math.sin(a), math.cos(a))
class CtrlMode(Enum):
    NORMAL = auto()
    TURNING = auto()
# ==============================================================================
# 配置组（移除绕障、后退相关参数）
# ==============================================================================
class TrackCfg:
    def __init__(self):
        self.Ld = 0.8                # 前视距离 (m)
        self.max_v = 0.8             # 最大线速度 (m/s)
        self.max_w = 0.8             # 最大角速度 (rad/s)
        self.goal_tol = 0.1          # 终点容差 (m)
        self.turn_tol = 0.2          # 调头容差 (rad)
        self.curv_k = 0.5            # 曲率减速系数
        self.min_scale = 0.4         # 最低速比例
        self.loop_tol = 0.5          # 闭环判定：首尾距离 < 此值视为闭环 (m)
# ==============================================================================
# PurePursuitController
# ==============================================================================
class PurePursuitController:
    """
    纯追踪控制器（不执行障碍物检测或避障）
    使用方法:
        ctrl = PurePursuitController()
        ctrl.update_state(x, y, yaw)
        ctrl.set_path([(x1,y1), ...])
        v, w = ctrl.solve()
    """
    def __init__(self):
        self.cfg_t = TrackCfg()
        # 状态
        self.x = self.y = self.yaw = 0.0
        self.path = []
        self.mode = CtrlMode.NORMAL
        self.is_loop = False          # 是否为闭环路径
        # 平滑滤波
        self._last_v = self._last_w = 0.0
    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------
    def update_state(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
    def set_path(self, pts):
        """设置跟踪路径，自动判断是否闭环"""
        self.path = pts
        self.mode = CtrlMode.NORMAL
        # 自动检测闭环：首尾距离 < loop_tol 视为闭环
        if pts and len(pts) > 1:
            d = np.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
            self.is_loop = d < self.cfg_t.loop_tol
    def solve(self):
        """
        3阶段管线: 定位 → 调头? → 纯追踪
        不检测障碍物，也不执行避障或紧急刹车
        """
        # 1. 定位前视点
        tgt = self._resolve_target()
        if tgt is None:
            return 0.0, 0.0
        tx, ty, lx, ly, yer, is_endpoint = tgt
        # 2. 调头判断
        if self._need_turn(lx, yer):
            return self._do_turn(yer)
        # 3. 纯追踪正常行驶
        if lx < 0.01:
            return 0.0, 0.0
        v, w = self._pure_track(ly)

        # 接近终点时减速，确保精准停靠
        if is_endpoint:
            dist_to_end = math.hypot(tx - self.x, ty - self.y)
            # 在 Ld 范围内线性减速
            if dist_to_end < self.cfg_t.Ld:
                ratio = dist_to_end / self.cfg_t.Ld
                v = v * max(ratio, 0.15)  # 最低保留15%速度防止完全不动

        return v, w
    # ------------------------------------------------------------------
    # 1. 定位前视点
    # ------------------------------------------------------------------
    def _resolve_target(self):
        """返回 (tx, ty, local_x, local_y, yaw_err, is_endpoint) 或 None"""
        if not self.path:
            return None
        _, _, ni = self._nearest_idx()
        if ni < 0:
            return None
        ti = self._lookahead_idx(ni)
        if ti < 0:
            return None

        n = len(self.path)
        is_endpoint = False

        # 非闭环：剩余路径太短或前视点回绕 → 直接用终点
        if not self.is_loop:
            dist_to_end = 0.0
            for i in range(ni, n - 1):
                dist_to_end += np.hypot(self.path[i+1][0] - self.path[i][0],
                                         self.path[i+1][1] - self.path[i][1])
            if dist_to_end < self.cfg_t.Ld or ti <= ni:
                ti = n - 1
                is_endpoint = True

        tx, ty = self.path[ti]
        dx, dy = tx - self.x, ty - self.y
        lx = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        ly = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        ye = normalize_angle(math.atan2(dy, dx) - self.yaw)
        return tx, ty, lx, ly, ye, is_endpoint
    # ------------------------------------------------------------------
    # 2. 原地调头逻辑
    # ------------------------------------------------------------------
    def _need_turn(self, lx, ye):
        """前视点在后方(lx<0)且朝向偏差较大时原地调头"""
        return lx < 0 and abs(ye) > self.cfg_t.turn_tol
    def _do_turn(self, ye):
        self.mode = CtrlMode.TURNING
        if abs(ye) > self.cfg_t.turn_tol:
            w = np.clip(ye * 2.0, -self.cfg_t.max_w, self.cfg_t.max_w)
        else:
            self.mode = CtrlMode.NORMAL
            w = 0.0
        return 0.0, w
    # ------------------------------------------------------------------
    # 4. 纯追踪算法
    # ------------------------------------------------------------------
    def _pure_track(self, ly):
        """纯追踪 + 曲率自适应 + 低通滤波"""
        ld2 = max(self.cfg_t.Ld ** 2, 0.01)
        k = 2.0 * ly / ld2
        wr = np.clip(k * self.cfg_t.max_v, -self.cfg_t.max_w, self.cfg_t.max_w)
        curv = abs(k)
        sc = np.clip(1.0 - curv * self.cfg_t.curv_k, self.cfg_t.min_scale, 1.0)
        vr = self.cfg_t.max_v * sc
        v = np.clip(self._lp_v(vr), 0, self.cfg_t.max_v)
        w = np.clip(self._lp_w(wr), -self.cfg_t.max_w, self.cfg_t.max_w)
        return v, w
    # ------------------------------------------------------------------
    # 低通滤波平滑速度、角速度
    # ------------------------------------------------------------------
    def _lp_v(self, raw):
        self._last_v = self._last_v * 0.7 + raw * 0.3
        return self._last_v
    def _lp_w(self, raw):
        self._last_w = self._last_w * 0.7 + raw * 0.3
        return self._last_w
    # ------------------------------------------------------------------
    # 路径搜索工具函数
    # ------------------------------------------------------------------
    def _nearest_idx(self):
        """(x, y, idx) 最近路径点"""
        if not self.path:
            return 0.0, 0.0, -1
        md, bi = float('inf'), 0
        for i, (px, py) in enumerate(self.path):
            d = np.hypot(self.x - px, self.y - py)
            if d < md:
                md, bi = d, i
        return self.path[bi][0], self.path[bi][1], bi
    def _lookahead_idx(self, si):
        """沿路径前搜前视点（支持循环）"""
        if not self.path:
            return -1
        n = len(self.path)
        if n == 1:
            return 0
        dist, ti = 0.0, si
        for s in range(min(n, 100)):
            ci = (si + s) % n
            ni = (si + s + 1) % n
            seg = np.hypot(self.path[ni][0] - self.path[ci][0],
                           self.path[ni][1] - self.path[ci][1])
            dist += seg
            if dist >= self.cfg_t.Ld:
                ti = ni
                break
        return ti % n
# ==============================================================================
# ROS 节点
# ==============================================================================
class TrackerNode:
    def __init__(self):
        self.ctrl = PurePursuitController()
        self._buf = tf2_ros.Buffer()
        self._lst = tf2_ros.TransformListener(self._buf)
        self._sub_path = rospy.Subscriber('/global_path', Path, self._cb_path, queue_size=1)
        self._pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._pub_reached = rospy.Publisher('/reached', Bool, queue_size=1)
        self._rate = rospy.Rate(20)
        self._reached = False         # 内部状态
        self._last_reached = None     # 上次发布的状态（None=未发布过）
        self._goal_pos = None
        self._is_loop = False
        self._goal_tol = 0.05
        self._last_path_pts = None    # 用于路径去重

    def _cb_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        # 路径去重：比较首/中/尾采样点，忽略同一条路径的重复发布
        if self._last_path_pts is not None and len(pts) == len(self._last_path_pts):
            same = True
            for idx in [0, len(pts) // 3, 2 * len(pts) // 3, len(pts) - 1]:
                d = math.hypot(pts[idx][0] - self._last_path_pts[idx][0],
                               pts[idx][1] - self._last_path_pts[idx][1])
                if d > 0.15:
                    same = False
                    break
            if same:
                return  # 相同逻辑路径（仅窗口滑动），跳过
        self._last_path_pts = pts

        self.ctrl.set_path(pts)
        self._goal_pos = pts[-1]
        self._reached = False
        self._last_reached = None     # 重置发布状态
        self._is_loop = self.ctrl.is_loop
        if self._is_loop:
            rospy.loginfo("收到闭环路径，%d 个点（将循环绕圈）", len(pts))
        else:
            rospy.loginfo("收到路径，%d 个点", len(pts))
    def run(self):
        while not rospy.is_shutdown():
            pose = self._get_pose()
            if pose is None:
                self._rate.sleep()
                continue
            x, y, _ = pose
            self.ctrl.update_state(*pose)

            # 闭环模式：无限循环，不判定到达
            if self._is_loop:
                v, w = self.ctrl.solve()
            elif not self._reached and self._goal_pos is not None:
                gx, gy = self._goal_pos
                dist_to_goal = math.hypot(x - gx, y - gy)
                if dist_to_goal < self._goal_tol:
                    self._reached = True
                else:
                    v, w = self.ctrl.solve()
            else:
                v, w = self.ctrl.solve()

            # 到达后停车
            if self._reached:
                v, w = 0.0, 0.0

            # 只在状态变化时发布 /reached
            if self._reached != self._last_reached:
                self._pub_reached.publish(Bool(data=self._reached))
                self._last_reached = self._reached
                if self._reached:
                    d = math.hypot(x - self._goal_pos[0], y - self._goal_pos[1]) if self._goal_pos else 0
                    rospy.loginfo("终点到达！距目标 %.3f m", d)

            t = Twist()
            t.linear.x, t.angular.z = v, w
            self._pub_cmd.publish(t)
            self._rate.sleep()

    def _get_pose(self):
        try:
            t = self._buf.lookup_transform('map', 'body_foot',
                                           rospy.Time(0), rospy.Duration(0.2))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            return x, y, yaw
        except Exception:
            return None
if __name__ == '__main__':
    rospy.init_node('pure_pursuit_tracker')
    try:
        TrackerNode().run()
    except rospy.ROSInterruptException:
        pass


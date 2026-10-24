#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全局路径跟踪控制器（纯追踪 + 主动绕障 + 紧急刹车 + 小幅后退）
=====================================================
架构：
  PurePursuitController — 核心控制器
  ├─ [定位]    解析前视目标点
  ├─ [调头]    严格判定 → 原地转向
  ├─ [绕障系]  路径阻塞检测 + Gap Finding + 状态机（±80°范围）
  ├─ [追踪]    纯追踪 + 曲率自适应 + 低通滤波
  ├─ [紧急]    三级安全覆盖：刹停 / 减速 / 侧方限速 + 小幅后退
  └─ solve()   5阶段管线：定位→调头?→绕障?→追踪→安全出口
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from threading import Lock
from enum import Enum, auto

# ==============================================================================
# 工具函数
# ==============================================================================

def normalize_angle(a):
    """归一化到 [-π, π]"""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a, b):
    """计算两角度最小差值 [0, π]"""
    d = abs(a - b)
    return min(d, 2 * math.pi - d)


def valid_range(r):
    """距离值是否有效（非 inf / nan / 负值）"""
    return not (np.isinf(r) or np.isnan(r) or r <= 0.0)


class CtrlMode(Enum):
    NORMAL = auto()
    TURNING = auto()
    AVOIDANCE = auto()
    BACKUP = auto()  # 新增：后退模式


# ==============================================================================
# 配置组（取代 __init__ 中的散落参数）
# ==============================================================================

class TrackCfg:
    def __init__(self):
        self.Ld = 3                # 前视距离 (m)
        self.max_v = 1.5             # 最大线速度 (m/s)
        self.max_w = 1.0             # 最大角速度 (rad/s)
        self.goal_tol = 0.2          # 终点容差 (m)
        self.turn_tol = 0.2          # 调头容差 (rad)
        self.curv_k = 2.0            # 曲率减速系数
        self.min_scale = 0.3         # 最低速比例


class ObsCfg:
    def __init__(self):
        self.stop_dist = 1.3         # 阻塞判定 (m)
        self.min_pts = 5             # 阻塞最少点数
        self.scan_rng = math.radians(80)  # 修改：绕障检测范围±80°
        self.safe_dist = 0.5        # 最小安全 (m)
        self.side_clr = 0.7          # 侧方空隙 (m)
        self.infl_dist = 2.0         # 影响半径 (m)
        self.fwd_clr = 0.8           # 畅通判定 (m)
        self.max_v_ratio = 0.5       # 修改：降低绕障最大速度比例（原0.6）
        self.min_v = 0.15            # 最低速度 (m/s)
        self.exit_dist = 1.0         # 退出前进距离 (m)
        self.exit_frames = 5         # 连续畅通帧
        self.timeout = 15.0          # 绕障超时 (s)
        self.backup_dist = 0.3       # 新增：触发后退的距离阈值
        self.backup_v = -0.2         # 新增：后退速度（负为后退）
        self.backup_frames = 3       # 新增：后退持续帧数
        self.backup_ang = math.radians(10)  # 新增：后退时转向角度


class EmgCfg:
    def __init__(self):
        self.stop_d = 0.35           # 刹停距离 (m)
        self.slow_d = 0.7            # 减速距离 (m)
        self.side_d = 0.25           # 侧方安全 (m)
        self.fwd_ang = math.radians(45)   # 前向检测 (±)
        self.side_start = math.radians(60) # 侧方起始角


# ==============================================================================
# PurePursuitController
# ==============================================================================

class PurePursuitController:
    """
    纯追踪控制器。
    使用方法:
        ctrl = PurePursuitController()
        ctrl.update_state(x, y, yaw)
        ctrl.set_path([(x1,y1), ...])
        ctrl.update_scan(ranges, amin, amax, ainc)
        v, w = ctrl.solve()
    """

    def __init__(self):
        self.cfg_t = TrackCfg()
        self.cfg_o = ObsCfg()
        self.cfg_e = EmgCfg()

        # 状态
        self.x = self.y = self.yaw = 0.0
        self.path = []
        self.mode = CtrlMode.NORMAL

        # 激光（线程安全）
        self._s_lock = Lock()
        self._s_rng = None
        self._s_amin = self._s_amax = self._s_inc = 0.0

        # 绕障状态
        self._av_pose = None   # (x, y, yaw)
        self._av_dist = 0.0
        self._av_t0 = None
        self._av_clear = 0

        # 紧急状态
        self._emg_on = False
        self._emg_cnt = 0

        # 平滑
        self._last_v = self._last_w = 0.0

        # 新增：后退状态
        self._backup_cnt = 0

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def update_state(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def set_path(self, pts):
        self.path = pts
        self.mode = CtrlMode.NORMAL
        self._reset_av()
        self._backup_cnt = 0  # 重置后退计数

    def update_scan(self, rng, amin, amax, inc):
        with self._s_lock:
            self._s_rng = rng
            self._s_amin, self._s_amax, self._s_inc = amin, amax, inc

    def solve(self):
        """
        5 阶段管线:  定位 → 调头? → 绕障? → 追踪 → 紧急出口
        新增：后退逻辑优先级高于绕障
        """
        # 1. 定位
        tgt = self._resolve_target()
        if tgt is None:
            return 0.0, 0.0
        tx, ty, lx, ly, yer = tgt

        # 2. 调头
        if self._need_turn(lx, yer):
            v, w = self._do_turn(yer)
            return self._apply_emg(v, w)

        # 新增：先检测是否需要后退
        if self._need_backup():
            v, w = self._do_backup()
            return self._apply_emg(v, w)

        # 3. 绕障
        self._update_av()
        if self.mode == CtrlMode.AVOIDANCE:
            v, w = self._avoid(tx, ty)
            # 无可行路径时停车并持续检测
            if v == 0.0 and w == 0.0:
                self._av_clear = 0  # 重置畅通计数，持续检测
            return self._apply_emg(v, w)

        # 4. 纯追踪
        if lx < 0.01:
            return self._apply_emg(0.0, 0.0)
        v, w = self._pure_track(ly)
        return self._apply_emg(v, w)

    # ------------------------------------------------------------------
    # 新增：后退逻辑
    # ------------------------------------------------------------------
    def _need_backup(self):
        """判断是否需要后退：前方极近距离有障碍物且无可行绕障路径"""
        if self.mode == CtrlMode.BACKUP:
            return True  # 已进入后退模式则继续

        # 1. 检测前方极近距离障碍物
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return False
        
        fwd_min_dist = float('inf')
        for i in range(n):
            if abs(amin + i * inc) < math.radians(30):  # 正前方±30°
                r = rng[i]
                if valid_range(r) and r < fwd_min_dist:
                    fwd_min_dist = r

        # 2. 检测是否有可行绕障路径
        has_valid_gap = self._check_valid_gap()

        # 前方极近且无可行路径 → 触发后退
        return fwd_min_dist < self.cfg_o.backup_dist and not has_valid_gap

    def _check_valid_gap(self):
        """检查是否有可行的绕障间隙"""
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return False
        
        fi = [i for i in range(n) if abs(amin + i * inc) < self.cfg_o.scan_rng]
        if not fi:
            return False
        
        _, blk, _ = self._build_grid(rng, amin, inc, n)
        if not blk:
            return True
        
        # 找有效间隙
        gaps = self._find_gaps(fi, blk, amin, inc, 0.0)  # 目标角设为0（正前方）
        return len(gaps) > 0

    def _do_backup(self):
        """执行小幅后退"""
        self.mode = CtrlMode.BACKUP
        self._backup_cnt += 1

        # 后退指定帧数后退出
        if self._backup_cnt > self.cfg_o.backup_frames:
            self.mode = CtrlMode.AVOIDANCE
            self._backup_cnt = 0
            return 0.0, 0.0

        # 后退时小幅转向（避免卡死）
        w = self.cfg_o.backup_ang * np.sign(self.yaw)  # 沿当前偏航小幅转向
        w = np.clip(w, -self.cfg_t.max_w*0.5, self.cfg_t.max_w*0.5)
        return self.cfg_o.backup_v, w

    # ------------------------------------------------------------------
    # 1. 定位
    # ------------------------------------------------------------------

    def _resolve_target(self):
        """返回 (tx, ty, local_x, local_y, yaw_err) 或 None"""
        if not self.path:
            return None
        _, _, ni = self._nearest_idx()
        if ni < 0:
            return None
        ti = self._lookahead_idx(ni)
        if ti < 0:
            return None
        tx, ty = self.path[ti]
        dx, dy = tx - self.x, ty - self.y
        lx = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        ly = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        ye = normalize_angle(math.atan2(dy, dx) - self.yaw)
        return tx, ty, lx, ly, ye

    # ------------------------------------------------------------------
    # 2. 调头
    # ------------------------------------------------------------------

    def _need_turn(self, lx, ye):
        # 关闭调头：目标点在后时由纯追踪自然转弯跟踪
        return False

    def _do_turn(self, ye):
        self.mode = CtrlMode.TURNING
        if abs(ye) > self.cfg_t.turn_tol:
            w = np.clip(ye * 2.0, -self.cfg_t.max_w, self.cfg_t.max_w)
        else:
            self.mode = CtrlMode.NORMAL
            w = 0.0
        return 0.0, w

    # ------------------------------------------------------------------
    # 3. 绕障系统
    # ------------------------------------------------------------------

    def _path_blocked(self):
        """(bool, min_dist) 路径前进方向是否被阻挡"""
        if not self.path:
            return False, float('inf')
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return False, float('inf')

        _, _, ni = self._nearest_idx()
        ti = self._lookahead_idx(ni)
        if ti < 0:
            return False, float('inf')
        tx, ty = self.path[ti]
        ta = normalize_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)

        d_rng = math.radians(25)
        md = float('inf')
        bc = 0
        for i in range(n):
            a = amin + i * inc
            if angle_diff(a, ta) < d_rng:
                r = rng[i]
                if not valid_range(r):
                    continue
                if r < md:
                    md = r
                if r < self.cfg_o.stop_dist:
                    bc += 1
        return bc >= self.cfg_o.min_pts, md

    def _fwd_clear(self):
        """正前方 ±20° 内是否畅通"""
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return True
        for i in range(n):
            if abs(amin + i * inc) < math.radians(20):
                r = rng[i]
                if valid_range(r) and r < self.cfg_o.fwd_clr:
                    return False
        return True

    def _update_av(self):
        blocked, _ = self._path_blocked()
        # 进入
        if self.mode != CtrlMode.AVOIDANCE and blocked and self.mode != CtrlMode.BACKUP:
            self._enter_av()
            return
        # 退出
        if self.mode == CtrlMode.AVOIDANCE:
            self._av_progress()
            if self._av_can_exit():
                self._exit_av()

    def _enter_av(self):
        self.mode = CtrlMode.AVOIDANCE
        self._av_pose = (self.x, self.y, self.yaw)
        self._av_dist = 0.0
        self._av_t0 = rospy.Time.now()
        self._av_clear = 0
        rospy.loginfo("[避障] 启动")

    def _exit_av(self):
        self.mode = CtrlMode.NORMAL
        self._reset_av()
        rospy.loginfo("[避障] 退出")

    def _reset_av(self):
        self._av_pose = None
        self._av_dist = 0.0
        self._av_t0 = None
        self._av_clear = 0

    def _av_progress(self):
        if self._av_pose is not None:
            d = np.hypot(self.x - self._av_pose[0], self.y - self._av_pose[1])
            self._av_dist = max(self._av_dist, d)

    def _av_can_exit(self):
        fc = self._fwd_clear()
        nb, _ = self._path_blocked()
        de = self._av_dist > self.cfg_o.exit_dist
        if fc and not nb:
            self._av_clear += 1
        else:
            self._av_clear = 0
        timeout = False
        if self._av_t0 is not None:
            el = (rospy.Time.now() - self._av_t0).to_sec()
            if el > self.cfg_o.timeout:
                timeout = True
                rospy.logwarn("[避障] 超时 %.1fs", el)
        return (self._av_clear >= self.cfg_o.exit_frames and de) or timeout

    def _avoid(self, tx, ty):
        """Gap Finding 主动绕障（优化：优先原轨迹方向）"""
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return 0.0, 0.0  # 修改：无激光数据时停车，而非低速前进

        ta = normalize_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        d = np.hypot(tx - self.x, ty - self.y)
        if abs(ta) < math.radians(15) and d < self.cfg_t.Ld * 0.5:
            ta *= 0.5

        fi, blk, md = self._build_grid(rng, amin, inc, n)
        if not fi:
            return 0.0, 0.0  # 修改：无检测点时停车

        gaps = self._find_gaps(fi, blk, amin, inc, ta)
        if not gaps:
            rospy.logwarn("[避障] 无可行路径，停车等待")
            return 0.0, 0.0  # 修改：无间隙时停车，持续检测

        best_a = self._best_gap(gaps)
        return self._av_cmd(best_a, fi, blk, md, amin, inc)

    def _build_grid(self, rng, amin, inc, n):
        """构建前方障碍物栅格"""
        fi = [i for i in range(n) if abs(amin + i * inc) < self.cfg_o.scan_rng]
        if not fi:
            return [], [], []
        m = len(fi)
        blk = [False] * m
        md = [float('inf')] * m
        d_inc = max(math.degrees(inc), 0.01)
        for j, i in enumerate(fi):
            r = rng[i]
            if not valid_range(r):
                continue
            if r < self.cfg_o.infl_dist:
                md[j] = r
                if r < self.cfg_o.stop_dist * 1.2:
                    blk[j] = True
                    bw = max(1, int(math.degrees(self.cfg_o.side_clr / max(r, 0.3)) / d_inc))
                    for k in range(1, bw + 1):
                        if j - k >= 0:
                            blk[j - k] = True
                            md[j - k] = min(md[j - k], r)
                        if j + k < m:
                            blk[j + k] = True
                            md[j + k] = min(md[j + k], r)
        return fi, blk, md

    @staticmethod
    def _find_gaps(fi, blk, amin, inc, ta):
        """找连续自由间隙 → [(中心角, 宽度, 与目标角差)]"""
        m, gaps = len(fi), []
        ingap, start = False, 0
        for j in range(m):
            if not blk[j] and not ingap:
                ingap, start = True, j
            elif blk[j] and ingap:
                ingap = False
                sa = amin + fi[start] * inc
                ea = amin + fi[j - 1] * inc
                ca = (sa + ea) / 2.0
                gaps.append((ca, abs(ea - sa), angle_diff(ca, ta)))
        if ingap:
            sa = amin + fi[start] * inc
            ea = amin + fi[-1] * inc
            ca = (sa + ea) / 2.0
            gaps.append((ca, abs(ea - sa), angle_diff(ca, ta)))
        return gaps

    @staticmethod
    def _best_gap(gaps):
        """评分选最优间隙（优化：优先靠近原轨迹的间隙）"""
        best = gaps[0][0]
        bs = -1e9
        for ca, w, ad in gaps:
            # 修改评分公式：宽度权重降低，角度差权重提高，减少大幅转向
            s = w * 1.5 - ad * 3.0  # 原：w*2 - ad*1.5
            # 惩罚过大的转向角（超过60°）
            if abs(ca) > math.radians(60):
                s -= 5.0
            if s > bs:
                bs, best = s, ca
        return best

    def _av_cmd(self, ba, fi, blk, md, amin, inc):
        """绕障控制指令（优化：降低转向激进程度）"""
        # 修改：转向系数从3.0降至2.0，减少转向幅度
        w_r = math.atan(ba * 2.0) / (math.pi / 2.0) * self.cfg_t.max_w
        w = np.clip(w_r, -self.cfg_t.max_w*0.8, self.cfg_t.max_w*0.8)  # 限制最大转向

        fm = float('inf')
        for j, i in enumerate(fi):
            if abs(amin + i * inc) < math.radians(30) and md[j] < fm:
                fm = md[j]
        if fm == float('inf'):
            fm = self.cfg_o.infl_dist

        tf = max(0.25, 1.0 - abs(ba) / math.radians(80))  # 适配80°检测范围
        if fm < self.cfg_o.safe_dist:
            vr = self.cfg_o.min_v
        else:
            ratio = (fm - self.cfg_o.safe_dist) / max(0.01, self.cfg_o.infl_dist - self.cfg_o.safe_dist)
            vr = np.clip(ratio, self.cfg_o.min_v / self.cfg_t.max_v,
                         self.cfg_o.max_v_ratio) * self.cfg_t.max_v * tf
        return self._lp_v(vr), self._lp_w(w)

    # ------------------------------------------------------------------
    # 4. 纯追踪
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
    # 5. 紧急刹车（最高优先级）
    # ------------------------------------------------------------------

    def _emg_check(self):
        """三级检测 → (v_limit, w_limit, active)"""
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            self._emg_decay()
            return 0.0, 0.0, False

        fm, cc = float('inf'), 0
        for i in range(n):
            if abs(amin + i * inc) < self.cfg_e.fwd_ang:
                r = rng[i]
                if not valid_range(r):
                    continue
                if r < fm:
                    fm = r
                if r < self.cfg_e.stop_d:
                    cc += 1

        sm = float('inf')
        for i in range(n):
            a = abs(amin + i * inc)
            if self.cfg_e.side_start <= a <= self.cfg_o.scan_rng:
                r = rng[i]
                if valid_range(r) and r < sm:
                    sm = r

        # 级别1 — 刹停
        if fm < self.cfg_e.stop_d or cc >= 3:
            self._emg_on, self._emg_cnt = True, 10
            return 0.0, 0.0, True
        # 级别2 — 减速
        if fm < self.cfg_e.slow_d:
            self._emg_on, self._emg_cnt = True, 5
            r = (fm - self.cfg_e.stop_d) / max(0.01, self.cfg_e.slow_d - self.cfg_e.stop_d)
            return np.clip(r, 0.0, 0.3) * self.cfg_t.max_v, None, True
        # 级别3 — 侧方限速
        if sm < self.cfg_e.side_d:
            self._emg_on, self._emg_cnt = True, 3
            return self.cfg_o.min_v * 0.5, None, True

        self._emg_decay()
        return 0.0, 0.0, False

    def _emg_decay(self):
        if self._emg_on:
            self._emg_cnt -= 1
            if self._emg_cnt <= 0:
                self._emg_on, self._emg_cnt = False, 0

    def _apply_emg(self, v, w):
        ev, ew, act = self._emg_check()
        if act:
            if ew is not None:
                return ev, ew
            v = min(v, ev)
        return v, w

    # ------------------------------------------------------------------
    # 低通滤波
    # ------------------------------------------------------------------

    def _lp_v(self, raw):
        self._last_v = self._last_v * 0.7 + raw * 0.3
        return self._last_v

    def _lp_w(self, raw):
        self._last_w = self._last_w * 0.7 + raw * 0.3
        return self._last_w

    # ------------------------------------------------------------------
    # 路径搜索
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

    # ------------------------------------------------------------------
    # 激光快照
    # ------------------------------------------------------------------

    def _snap(self):
        """线程安全快照 (rng, amin, amax, inc, n)"""
        with self._s_lock:
            if self._s_rng is None:
                return None, 0.0, 0.0, 0.0, 0
            return (self._s_rng, self._s_amin, self._s_amax,
                    self._s_inc, len(self._s_rng))


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
        self._sub_scan = rospy.Subscriber('/scan', LaserScan, self._cb_scan, queue_size=1)
        self._rate = rospy.Rate(20)
        self._reached = Bool(data=False)

    def _cb_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.ctrl.set_path(pts)

    def _cb_scan(self, msg):
        """激光回调：将数据传给控制器，绕障状态由控制器内部管理"""
        self.ctrl.update_scan(msg.ranges, msg.angle_min,
                              msg.angle_max, msg.angle_increment)

    def run(self):
        while not rospy.is_shutdown():
            pose = self._get_pose()
            if pose is None:
                self._rate.sleep()
                continue
            self.ctrl.update_state(*pose)
            v, w = self.ctrl.solve()
            t = Twist()
            t.linear.x, t.angular.z = v, w
            self._pub_cmd.publish(t)
            self._pub_reached.publish(self._reached)
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


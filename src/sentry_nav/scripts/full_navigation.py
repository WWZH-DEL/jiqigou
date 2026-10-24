#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整导航管线：RVIZ 目标 -> A* 全局规划 -> 纯追踪跟踪 -> /cmd_vel
==================================================================
工作流程:
  1. 接收 /prior_map -> 障碍物膨胀预处理
  2. 接收 /move_base_simple/goal -> A* 全局规划 -> 发布 /global_path
  3. 20Hz 控制循环: 纯追踪跟踪 + Gap Finding 绕障 + 紧急刹车
  4. 到达目标 -> 停车 -> 发布 /reached

话题:
  订阅: /prior_map, /move_base_simple/goal, /scan
  发布: /global_path, /actual_path, /cmd_vel, /reached
TF:   map -> body_foot
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from heapq import heappush, heappop
import math
import tf2_ros
from threading import Lock
from enum import Enum, auto


# ==============================================================================
# 工具函数
# ==============================================================================

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a, b):
    d = abs(a - b)
    return min(d, 2 * math.pi - d)


def valid_range(r):
    return not (np.isinf(r) or np.isnan(r) or r <= 0.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class CtrlMode(Enum):
    NORMAL = auto()
    TURNING = auto()
    AVOIDANCE = auto()


# ==============================================================================
# A* 全局路径规划
# ==============================================================================

class AStarGlobalPlanner:
    def __init__(self):
        self.robot_radius = 0.6
        self.resolution = 0.1
        self.safe_dist_m = self.robot_radius + 0.15
        self.inflation_radius = self.safe_dist_m

        self.smooth_weight_data = 0.3
        self.smooth_weight_smooth = 0.2
        self.tolerance = 1e-6
        self.smooth_iterations = 200
        self.simplify_tolerance = 0.05
        self.interp_spacing = 0.03

        self.map_data = None
        self.inflated_map = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None

    def set_map(self, occ_grid_msg):
        self.map_data = np.array(occ_grid_msg.data).reshape(
            (occ_grid_msg.info.height, occ_grid_msg.info.width))
        self.map_width = occ_grid_msg.info.width
        self.map_height = occ_grid_msg.info.height
        self.map_origin = occ_grid_msg.info.origin
        self.resolution = occ_grid_msg.info.resolution

        self.inflated_map = self._inflate_obstacles(
            self.map_data,
            int(self.inflation_radius / self.resolution))

        obs_count = (self.map_data == 100).sum()
        rospy.loginfo("Map received (%d obstacles, inflation=%.2fm)",
                      obs_count, self.inflation_radius)
        return obs_count >= 0

    def has_map(self):
        return self.map_data is not None

    def _inflate_obstacles(self, grid, inflation_cells):
        inflated = np.copy(grid)
        obstacle_coords = np.argwhere(grid == 100)
        if len(obstacle_coords) == 0:
            return inflated
        rows, cols = grid.shape
        hw = inflation_cells
        for y, x in obstacle_coords:
            y_min = max(0, y - hw)
            y_max = min(rows, y + hw + 1)
            x_min = max(0, x - hw)
            x_max = min(cols, x + hw + 1)
            inflated[y_min:y_max, x_min:x_max] = 100
        return inflated

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin.position.x) / self.resolution)
        my = int((y - self.map_origin.position.y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = (mx + 0.5) * self.resolution + self.map_origin.position.x
        wy = (my + 0.5) * self.resolution + self.map_origin.position.y
        return wx, wy

    def plan(self, start_world, goal_world):
        if self.map_data is None:
            rospy.logwarn("Map not ready")
            return None

        sx, sy = self.world_to_map(*start_world)
        gx, gy = self.world_to_map(*goal_world)

        if not (0 <= sx < self.map_width and 0 <= sy < self.map_height):
            rospy.logerr("Start out of map bounds!")
            return None
        if not (0 <= gx < self.map_width and 0 <= gy < self.map_height):
            rospy.logerr("Goal out of map bounds!")
            return None

        path_indices = self._a_star((sx, sy), (gx, gy), self.inflated_map)
        if not path_indices:
            rospy.logwarn("A* found no path!")
            return None

        rospy.loginfo("Raw A* path: %d points", len(path_indices))

        raw = [self.map_to_world(ix, iy) for ix, iy in path_indices]
        raw = self._remove_dup(raw)
        simplified = self._simplify(raw)
        rospy.loginfo("  Simplified: %d points", len(simplified))
        smooth = self._smooth(simplified)
        final = self._interpolate(smooth)
        rospy.loginfo("  Final path: %d points", len(final))
        return final

    def _a_star(self, start, goal, cost_map):
        rows, cols = cost_map.shape
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        gx, gy = goal

        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, -1), (1, -1), (-1, 1)
        ]
        move_costs = [1.0, 1.0, 1.0, 1.0,
                      1.414, 1.414, 1.414, 1.414]

        while open_set:
            _, current = heappop(open_set)
            cx, cy = current

            if current == goal:
                return self._reconstruct(came_from, current)

            for (dx, dy), cost in zip(directions, move_costs):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < cols and 0 <= ny < rows):
                    continue
                if cost_map[ny, nx] == 100:
                    continue

                next_pos = (nx, ny)
                tg = g_score[current] + cost
                if next_pos not in g_score or tg < g_score[next_pos]:
                    came_from[next_pos] = current
                    g_score[next_pos] = tg
                    f = tg + math.hypot(nx - gx, ny - gy)
                    heappush(open_set, (f, next_pos))

        return []

    @staticmethod
    def _reconstruct(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _remove_dup(pts):
        if not pts:
            return pts
        cleaned = [pts[0]]
        for pt in pts[1:]:
            last = cleaned[-1]
            if abs(pt[0] - last[0]) > 1e-6 or abs(pt[1] - last[1]) > 1e-6:
                cleaned.append(pt)
        return cleaned

    def _simplify(self, pts):
        if len(pts) <= 2:
            return pts
        arr = np.array(pts)

        def perp_dist(p, s, e):
            if np.allclose(s, e):
                return np.linalg.norm(p - s)
            sv = e - s
            pv = p - s
            t = np.dot(pv, sv) / max(np.dot(sv, sv), 1e-12)
            t = max(0.0, min(1.0, t))
            return np.linalg.norm(p - (s + t * sv))

        def dp(points, tol):
            if len(points) <= 2:
                return points
            s, e = points[0], points[-1]
            md, mi = 0.0, 0
            for i in range(1, len(points) - 1):
                d = perp_dist(points[i], s, e)
                if d > md:
                    md, mi = d, i
            if md > tol:
                left = dp(points[:mi + 1], tol)
                right = dp(points[mi:], tol)
                return np.vstack((left[:-1], right))
            return np.array([s, e])

        return dp(arr, self.simplify_tolerance).tolist()

    def _smooth(self, pts):
        if len(pts) < 3:
            return pts
        path = np.array(pts, dtype=np.float64)
        smooth = np.copy(path)
        wd, ws = self.smooth_weight_data, self.smooth_weight_smooth

        for it in range(self.smooth_iterations):
            prev = np.copy(smooth)
            for i in range(1, len(path) - 1):
                smooth[i] += wd * (path[i] - smooth[i]) + \
                    ws * (smooth[i-1] + smooth[i+1] - 2.0 * smooth[i])
            if np.sum(np.abs(smooth - prev)) < self.tolerance:
                rospy.loginfo("  Smooth converged at iter %d", it + 1)
                break
        return smooth.tolist()

    def _interpolate(self, pts):
        if len(pts) < 2:
            return pts
        arr = np.array(pts)
        dists = np.zeros(len(arr))
        for i in range(1, len(arr)):
            dists[i] = dists[i-1] + np.linalg.norm(arr[i] - arr[i-1])
        total = dists[-1]
        n = max(2, int(total / self.interp_spacing))
        new_d = np.linspace(0, total, n)
        result = []
        for d in new_d:
            idx = np.searchsorted(dists, d)
            if idx == 0:
                result.append(arr[0].tolist())
            elif idx >= len(arr):
                result.append(arr[-1].tolist())
            else:
                t = (d - dists[idx-1]) / max(dists[idx] - dists[idx-1], 1e-12)
                pt = arr[idx-1] + t * (arr[idx] - arr[idx-1])
                result.append(pt.tolist())
        return result


# ==============================================================================
# 纯追踪跟踪控制器
# ==============================================================================

class TrackCfg:
    def __init__(self):
        self.Ld = 1.5
        self.max_v = 1.5
        self.max_w = 1.0
        self.goal_tol = 0.2
        self.turn_tol = 0.2
        self.curv_k = 2.0
        self.min_scale = 0.3


class ObsCfg:
    def __init__(self):
        self.stop_dist = 1.3
        self.min_pts = 5
        self.scan_rng = math.radians(100)
        self.safe_dist = 0.35
        self.side_clr = 0.5
        self.infl_dist = 2.0
        self.fwd_clr = 0.8
        self.max_v_ratio = 0.6
        self.min_v = 0.15
        self.exit_dist = 1.0
        self.exit_frames = 5
        self.timeout = 15.0


class EmgCfg:
    def __init__(self):
        self.stop_d = 0.35
        self.slow_d = 0.7
        self.side_d = 0.25
        self.fwd_ang = math.radians(45)
        self.side_start = math.radians(60)


class PurePursuitTracker:
    def __init__(self):
        self.cfg_t = TrackCfg()
        self.cfg_o = ObsCfg()
        self.cfg_e = EmgCfg()

        self.x = self.y = self.yaw = 0.0
        self.path = []
        self.goal_pos = None
        self.mode = CtrlMode.NORMAL

        self._s_lock = Lock()
        self._s_rng = None
        self._s_amin = self._s_amax = self._s_inc = 0.0

        self._av_pose = None
        self._av_dist = 0.0
        self._av_t0 = None
        self._av_clear = 0

        self._emg_on = False
        self._emg_cnt = 0

        self._last_v = self._last_w = 0.0
        self.goal_reached = False

    def update_state(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def set_path(self, pts, goal=None):
        self.path = pts
        self.goal_pos = goal if goal else (pts[-1] if pts else None)
        self.mode = CtrlMode.NORMAL
        self.goal_reached = False
        self._reset_av()

    def update_scan(self, rng, amin, amax, inc):
        with self._s_lock:
            self._s_rng = rng
            self._s_amin, self._s_amax, self._s_inc = amin, amax, inc

    def solve(self):
        if self._check_arrived():
            self.goal_reached = True
            return 0.0, 0.0

        tgt = self._resolve_target()
        if tgt is None:
            return 0.0, 0.0
        lx, ly, yaw_err = tgt

        self._update_av()
        if self.mode == CtrlMode.AVOIDANCE:
            v, w = self._avoid()
            return self._apply_emg(v, w)

        if lx < 0.01:
            return self._apply_emg(0.0, 0.0)
        v, w = self._pure_track(ly)
        return self._apply_emg(v, w)

    def _check_arrived(self):
        if self.goal_pos is None or not self.path:
            return False
        gx, gy = self.goal_pos
        d = math.hypot(self.x - gx, self.y - gy)
        return d < self.cfg_t.goal_tol

    def _resolve_target(self):
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
        return lx, ly, ye

    def _path_blocked(self):
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
        if self.mode != CtrlMode.AVOIDANCE and blocked:
            self._enter_av()
            return
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
        rospy.loginfo("[AV] Enter avoidance")

    def _exit_av(self):
        self.mode = CtrlMode.NORMAL
        self._reset_av()
        rospy.loginfo("[AV] Exit avoidance")

    def _reset_av(self):
        self._av_pose = None
        self._av_dist = 0.0
        self._av_t0 = None
        self._av_clear = 0

    def _av_progress(self):
        if self._av_pose is not None:
            d = math.hypot(self.x - self._av_pose[0],
                           self.y - self._av_pose[1])
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
            elapsed = (rospy.Time.now() - self._av_t0).to_sec()
            if elapsed > self.cfg_o.timeout:
                timeout = True
                rospy.logwarn("[AV] Timeout %.1fs", elapsed)
        return (self._av_clear >= self.cfg_o.exit_frames and de) or timeout

    def _avoid(self):
        rng, amin, _, inc, n = self._snap()
        if rng is None or n == 0:
            return self.cfg_t.max_v * 0.3, 0.0

        tx, ty = (self.goal_pos if self.goal_pos else
                  (self.path[-1] if self.path else (self.x, self.y)))
        ta = normalize_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        d = math.hypot(tx - self.x, ty - self.y)
        if abs(ta) < math.radians(15) and d < self.cfg_t.Ld * 0.5:
            ta *= 0.5

        fi, blk, md = self._build_grid(rng, amin, inc, n)
        if not fi:
            return self.cfg_t.max_v * 0.3, 0.0

        gaps = self._find_gaps(fi, blk, amin, inc, ta)
        if not gaps:
            return 0.0, self.cfg_t.max_w * 0.5

        best_a = self._best_gap(gaps)
        return self._av_cmd(best_a, fi, blk, md, amin, inc)

    def _build_grid(self, rng, amin, inc, n):
        fi = [i for i in range(n)
              if abs(amin + i * inc) < self.cfg_o.scan_rng]
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
                    bw = max(1, int(math.degrees(
                        self.cfg_o.side_clr / max(r, 0.3)) / d_inc))
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
        best, bs = gaps[0][0], -1e9
        for ca, w, ad in gaps:
            p = 3.0 if abs(ca) > math.radians(110) else 0.0
            s = w * 2.0 - ad * 1.5 - p
            if s > bs:
                bs, best = s, ca
        return best

    def _av_cmd(self, ba, fi, blk, md, amin, inc):
        w_r = math.atan(ba * 3.0) / (math.pi / 2.0) * self.cfg_t.max_w
        w = np.clip(w_r, -self.cfg_t.max_w, self.cfg_t.max_w)

        fm = float('inf')
        for j, i in enumerate(fi):
            if abs(amin + i * inc) < math.radians(30) and md[j] < fm:
                fm = md[j]
        if fm == float('inf'):
            fm = self.cfg_o.infl_dist

        tf = max(0.25, 1.0 - abs(ba) / math.radians(120))
        if fm < self.cfg_o.safe_dist:
            vr = self.cfg_o.min_v
        else:
            ratio = (fm - self.cfg_o.safe_dist) / max(
                0.01, self.cfg_o.infl_dist - self.cfg_o.safe_dist)
            vr = np.clip(ratio, self.cfg_o.min_v / self.cfg_t.max_v,
                         self.cfg_o.max_v_ratio) * self.cfg_t.max_v * tf
        return self._lp_v(vr), self._lp_w(w)

    def _pure_track(self, ly):
        ld2 = max(self.cfg_t.Ld ** 2, 0.01)
        k = 2.0 * ly / ld2
        wr = np.clip(k * self.cfg_t.max_v, -self.cfg_t.max_w, self.cfg_t.max_w)
        curv = abs(k)
        sc = np.clip(1.0 - curv * self.cfg_t.curv_k, self.cfg_t.min_scale, 1.0)
        vr = self.cfg_t.max_v * sc
        v = np.clip(self._lp_v(vr), 0.0, self.cfg_t.max_v)
        w = np.clip(self._lp_w(wr), -self.cfg_t.max_w, self.cfg_t.max_w)
        return v, w

    def _emg_check(self):
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

        if fm < self.cfg_e.stop_d or cc >= 3:
            self._emg_on, self._emg_cnt = True, 10
            return 0.0, 0.0, True
        if fm < self.cfg_e.slow_d:
            self._emg_on, self._emg_cnt = True, 5
            r = (fm - self.cfg_e.stop_d) / max(
                0.01, self.cfg_e.slow_d - self.cfg_e.stop_d)
            return np.clip(r, 0.0, 0.3) * self.cfg_t.max_v, None, True
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

    def _lp_v(self, raw):
        self._last_v = self._last_v * 0.7 + raw * 0.3
        return self._last_v

    def _lp_w(self, raw):
        self._last_w = self._last_w * 0.7 + raw * 0.3
        return self._last_w

    def _nearest_idx(self):
        if not self.path:
            return 0.0, 0.0, -1
        md, bi = float('inf'), 0
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(self.x - px, self.y - py)
            if d < md:
                md, bi = d, i
        return self.path[bi][0], self.path[bi][1], bi

    def _lookahead_idx(self, si):
        if not self.path:
            return -1
        n = len(self.path)
        if n == 1:
            return 0
        dist, ti = 0.0, si
        for s in range(min(n, 100)):
            ci = (si + s) % n
            ni = (si + s + 1) % n
            seg = math.hypot(self.path[ni][0] - self.path[ci][0],
                             self.path[ni][1] - self.path[ci][1])
            dist += seg
            if dist >= self.cfg_t.Ld:
                ti = ni
                break
        return ti % n

    def _snap(self):
        with self._s_lock:
            if self._s_rng is None:
                return None, 0.0, 0.0, 0.0, 0
            return (self._s_rng, self._s_amin, self._s_amax,
                    self._s_inc, len(self._s_rng))


# ==============================================================================
# 主节点
# ==============================================================================

class FullNavigationNode:
    def __init__(self):
        rospy.init_node('full_navigation', anonymous=False)

        self.planner = AStarGlobalPlanner()
        self.tracker = PurePursuitTracker()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._goal_pose = None
        self._last_goal_pos = None
        self._last_goal_time = rospy.Time(0)
        self._replan_cooldown = 1.0
        self._goal_arrival_threshold = 0.72

        self._actual_path = []

        rospy.Subscriber('/prior_map', OccupancyGrid,
                         self._cb_map, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                         self._cb_goal, queue_size=1)
        rospy.Subscriber('/scan', LaserScan,
                         self._cb_scan, queue_size=1)

        self._pub_global_path = rospy.Publisher(
            '/global_path', Path, queue_size=1)
        self._pub_actual_path = rospy.Publisher(
            '/actual_path', Path, queue_size=1)
        self._pub_cmd = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        self._pub_reached = rospy.Publisher(
            '/reached', Bool, queue_size=1)

        self._control_timer = rospy.Timer(
            rospy.Duration(0.05), self._control_loop)
        self._path_timer = rospy.Timer(
            rospy.Duration(0.1), self._publish_actual_path)

        rospy.loginfo("=" * 55)
        rospy.loginfo("Full Navigation Node Started")
        rospy.loginfo("  Sub: /prior_map /move_base_simple/goal /scan")
        rospy.loginfo("  Pub: /global_path /actual_path /cmd_vel /reached")
        rospy.loginfo("=" * 55)

    def _cb_map(self, msg):
        self.planner.set_map(msg)

    def _cb_goal(self, msg):
        now = rospy.Time.now()
        gx, gy = msg.pose.position.x, msg.pose.position.y

        if self._last_goal_pos is not None:
            dt = (now - self._last_goal_time).to_sec()
            dx = gx - self._last_goal_pos[0]
            dy = gy - self._last_goal_pos[1]
            same_dist = math.hypot(dx, dy)
            if dt < self._replan_cooldown and same_dist < self.planner.robot_radius:
                rospy.loginfo("Skip duplicate goal (%.2fm, %.2fs)",
                              same_dist, dt)
                return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            rospy.logerr("Cannot get robot pose, skip planning")
            return
        rx, ry, _ = robot_pose

        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < self._goal_arrival_threshold:
            rospy.loginfo("Already at goal (%.2fm)", dist_to_goal)
            return

        rospy.loginfo("New goal (%.2f, %.2f), dist=%.2fm, planning...",
                      gx, gy, dist_to_goal)

        path = self.planner.plan((rx, ry), (gx, gy))
        if path is None:
            rospy.logwarn("Global planning failed")
            return

        self._goal_pose = msg.pose
        self._last_goal_pos = (gx, gy)
        self._last_goal_time = now
        self._actual_path.clear()

        self._publish_global_path(path)
        self.tracker.set_path(path, goal=(gx, gy))
        rospy.loginfo("Path published (%d pts), tracking started", len(path))

    def _cb_scan(self, msg):
        self.tracker.update_scan(
            msg.ranges, msg.angle_min,
            msg.angle_max, msg.angle_increment)

    def _control_loop(self, event):
        pose = self._get_robot_pose()
        if pose is None:
            return

        rx, ry, ryaw = pose
        self.tracker.update_state(rx, ry, ryaw)

        v, w = self.tracker.solve()

        reached = Bool(data=self.tracker.goal_reached)
        if reached.data:
            v, w = 0.0, 0.0

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self._pub_cmd.publish(cmd)
        self._pub_reached.publish(reached)

    def _publish_actual_path(self, event):
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry = pose[0], pose[1]

        self._actual_path.append((rx, ry))
        if len(self._actual_path) > 2000:
            self._actual_path.pop(0)

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for px, py in self._actual_path:
            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = px
            p.pose.position.y = py
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)
        self._pub_actual_path.publish(path_msg)

    def _publish_global_path(self, path_pts):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for px, py in path_pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = px
            p.pose.position.y = py
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self._pub_global_path.publish(msg)

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'body_foot',
                rospy.Time(0), rospy.Duration(0.2))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = quat_to_yaw(t.transform.rotation)
            return x, y, yaw
        except Exception:
            return None


# ==============================================================================
# 启动
# ==============================================================================

if __name__ == '__main__':
    try:
        node = FullNavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


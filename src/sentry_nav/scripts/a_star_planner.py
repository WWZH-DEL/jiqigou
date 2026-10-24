#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from heapq import heappush, heappop
import math
import tf2_ros

class AStarPlanner:
    def __init__(self):
        rospy.init_node('a_star_planner_simple', anonymous=True)
        
        # ==================== 机器人参数 ====================
        self.robot_radius = 0.6                     # 差速车半径 (m)
        self.resolution = 0.1
        self.safe_dist_m = self.robot_radius + 0.15  # 安全距离 = 半径 + 余量
        self.inflation_radius = self.safe_dist_m     # 膨胀半径

        # ==================== 平滑参数 ====================
        self.smooth_weight_data = 0.3                # 数据项权重（保持路径形状）
        self.smooth_weight_smooth = 0.2              # 平滑项权重（使路径光滑）
        self.tolerance = 1e-6
        self.smooth_iterations = 200                 # 迭代次数，更多≈更光滑
        self.simplify_tolerance = 0.05               # 路径简化容差 (m)
        self.interp_spacing = 0.03                   # 插值点间距 (m)

        # ==================== 状态变量 ====================
        self.map_data = None
        self.inflated_map = None    # 膨胀后的地图，用于快速碰撞检测
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None
        self.goal_pose = None
        self.current_pose = None

        # ==================== 实际行驶轨迹 ====================
        self.actual_path_points = []
        self.max_actual_path_length = 2000

        # ==================== 重复规划防护 ====================
        self.goal_arrival_threshold = self.robot_radius * 1.2  # 到达判定阈值
        self._last_goal_pos = None          # 上一次目标位置
        self._last_goal_time = rospy.Time(0)  # 上一次目标时间戳
        self._replan_cooldown = 1.0          # 最小规划间隔（秒）

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅与发布（话题名保持不变）
        rospy.Subscriber('/prior_map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.actual_path_pub = rospy.Publisher('/actual_path', Path, queue_size=1)

        rospy.loginfo(f"✅ A* Planner 已启动 (半径={self.robot_radius}m, 安全距离={self.safe_dist_m}m)")

        # 定时发布实际轨迹（10Hz）
        rospy.Timer(rospy.Duration(0.1), self.publish_actual_path)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        self.resolution = msg.info.resolution
        
        # 预处理：膨胀障碍物地图，加速A*碰撞检测
        self.inflated_map = self.inflate_obstacles(
            self.map_data,
            int(self.inflation_radius / self.resolution)
        )
        rospy.loginfo(f"地图已接收 ({(self.map_data==100).sum()} 个障碍物, 膨胀半径={self.inflation_radius}m)")

    def inflate_obstacles(self, grid, inflation_cells):
        """
        对障碍物进行膨胀处理，生成碰撞检测用地图。
        差速车可看作圆形，将障碍物膨胀 robot_radius 后，
        路径点只需检查单点是否在膨胀区域即可，极大加速A*。
        """
        inflated = np.copy(grid)
        obstacle_mask = (grid == 100)
        obstacle_coords = np.argwhere(obstacle_mask)
        
        if len(obstacle_coords) == 0:
            return inflated
        
        rows, cols = grid.shape
        half_width = inflation_cells
        
        for y, x in obstacle_coords:
            y_min = max(0, y - half_width)
            y_max = min(rows, y + half_width + 1)
            x_min = max(0, x - half_width)
            x_max = min(cols, x + half_width + 1)
            inflated[y_min:y_max, x_min:x_max] = 100
        
        return inflated

    def goal_callback(self, msg):
        """
        接收 RVIZ 2D Nav Goal 目标点。
        包含重复规划防护逻辑：
          1) 相同目标点短时间内不重复规划
          2) 机器人已到达目标附近时忽略新目标
          3) 新目标与旧目标非常接近时跳过
        """
        now = rospy.Time.now()

        # ---- 防护1: 冷却时间检查 - 防止同一目标反复触发 ----
        if self._last_goal_pos is not None:
            dt = (now - self._last_goal_time).to_sec()
            dx = msg.pose.position.x - self._last_goal_pos[0]
            dy = msg.pose.position.y - self._last_goal_pos[1]
            same_goal_dist = math.hypot(dx, dy)

            if dt < self._replan_cooldown and same_goal_dist < self.robot_radius:
                rospy.loginfo(f"⏳ 目标重复且间隔过短 ({dt:.2f}s)，跳过")
                return

        # ---- 防护2: 检查机器人是否已在目标附近 ----
        try:
            trans = self.tf_buffer.lookup_transform('map', 'body_foot',
                                                     rospy.Time(0), rospy.Duration(1.0))
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            dx_goal = msg.pose.position.x - robot_x
            dy_goal = msg.pose.position.y - robot_y
            dist_to_goal = math.hypot(dx_goal, dy_goal)

            if dist_to_goal < self.goal_arrival_threshold:
                rospy.loginfo(f"📍 机器人已在目标附近 ({dist_to_goal:.2f}m)，无需规划")
                return

            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = now
            start.pose.position.x = trans.transform.translation.x
            start.pose.position.y = trans.transform.translation.y
            start.pose.position.z = trans.transform.translation.z
            start.pose.orientation = trans.transform.rotation
            self.current_pose = start.pose

        except Exception as e:
            rospy.logerr(f"无法从TF获取机器人位姿: {e}")
            return

        self.goal_pose = msg.pose
        self._last_goal_pos = (msg.pose.position.x, msg.pose.position.y)
        self._last_goal_time = now

        # 重置实际行驶轨迹，避免新旧路径重叠显示
        self.actual_path_points.clear()
        rospy.loginfo(f"🎯 新目标 ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})，开始规划...")
        self.plan_path()

    def plan_path(self):
        if self.map_data is None or self.current_pose is None or self.goal_pose is None:
            rospy.logwarn("缺少数据（地图、目标点或当前位置）")
            return

        start_x, start_y = self.world_to_map(
            self.current_pose.position.x, self.current_pose.position.y)
        goal_x, goal_y = self.world_to_map(
            self.goal_pose.position.x, self.goal_pose.position.y)

        if not (0 <= start_x < self.map_width and 0 <= start_y < self.map_height):
            rospy.logerr("起点超出地图边界！")
            return
        if not (0 <= goal_x < self.map_width and 0 <= goal_y < self.map_height):
            rospy.logerr("目标点超出地图边界！")
            return

        # A* 搜索（使用膨胀地图快速检测碰撞）
        path_indices = self.a_star_search(
            (start_x, start_y), (goal_x, goal_y), self.inflated_map)

        if not path_indices:
            rospy.logwarn("未找到可行路径！")
            return

        rospy.loginfo(f"原始路径 {len(path_indices)} 个点，正在优化...")

        # 转换为世界坐标
        raw_path = [self.map_to_world(ix, iy) for ix, iy in path_indices]

        # Step 0: 去除连续重复点（格点转坐标可能产生相同位置）
        raw_path = self.remove_duplicate_points(raw_path)

        # Step 1: 路径简化（去除冗余共线点）
        simplified = self.simplify_path(raw_path)
        rospy.loginfo(f"简化后 {len(simplified)} 个点")

        # Step 2: 路径平滑（梯度下降）
        smooth_path = self.smooth_path(simplified)

        # Step 3: 插值加密（使路径更细腻光滑）
        final_path = self.interpolate_path(smooth_path)
        rospy.loginfo(f"最终路径 {len(final_path)} 个点")

        self.publish_path(final_path)
        rospy.loginfo("✅ 优化路径已发布！")

    def remove_duplicate_points(self, path_points):
        """去除路径中连续重复的点，避免路径重叠"""
        if not path_points:
            return path_points
        cleaned = [path_points[0]]
        for pt in path_points[1:]:
            last = cleaned[-1]
            if abs(pt[0] - last[0]) > 1e-6 or abs(pt[1] - last[1]) > 1e-6:
                cleaned.append(pt)
        return cleaned

    def simplify_path(self, path_points):
        """
        使用 Douglas-Peucker 算法简化路径，去除冗余共线点。
        减少后续平滑的计算量，同时保留路径的关键形状特征。
        """
        if len(path_points) <= 2:
            return path_points

        pts = np.array(path_points)

        def perpendicular_distance(point, line_start, line_end):
            """计算点到线段的垂直距离"""
            if np.allclose(line_start, line_end):
                return np.linalg.norm(point - line_start)
            line_vec = line_end - line_start
            point_vec = point - line_start
            line_len = np.dot(line_vec, line_vec)
            t = np.dot(point_vec, line_vec) / line_len
            t = max(0.0, min(1.0, t))
            projection = line_start + t * line_vec
            return np.linalg.norm(point - projection)

        def douglas_peucker_recursive(points, tolerance):
            if len(points) <= 2:
                return points

            start = points[0]
            end = points[-1]
            max_dist = 0.0
            max_idx = 0

            for i in range(1, len(points) - 1):
                dist = perpendicular_distance(points[i], start, end)
                if dist > max_dist:
                    max_dist = dist
                    max_idx = i

            if max_dist > tolerance:
                left = douglas_peucker_recursive(points[:max_idx + 1], tolerance)
                right = douglas_peucker_recursive(points[max_idx:], tolerance)
                return np.vstack((left[:-1], right))
            else:
                return np.array([start, end])

        simplified = douglas_peucker_recursive(pts, self.simplify_tolerance)
        return simplified.tolist()

    def interpolate_path(self, path_points):
        """
        对路径进行等间距插值加密，生成细腻平滑的路径点序列。
        差速车需要密集的路径点来实现更平顺的跟踪。
        """
        if len(path_points) < 2:
            return path_points

        pts = np.array(path_points)

        # 计算累积弧长
        dists = np.zeros(len(pts))
        for i in range(1, len(pts)):
            dists[i] = dists[i-1] + np.linalg.norm(pts[i] - pts[i-1])

        total_len = dists[-1]
        num_points = max(2, int(total_len / self.interp_spacing))

        # 等弧长采样
        new_dists = np.linspace(0, total_len, num_points)

        # 分段线性插值
        new_path = []
        for d in new_dists:
            idx = np.searchsorted(dists, d)
            if idx == 0:
                new_path.append(pts[0].tolist())
            elif idx >= len(pts):
                new_path.append(pts[-1].tolist())
            else:
                t = (d - dists[idx-1]) / (dists[idx] - dists[idx-1])
                point = pts[idx-1] + t * (pts[idx] - pts[idx-1])
                new_path.append(point.tolist())

        return new_path

    def publish_actual_path(self, event):
        """定时从TF获取机器人位置，记录轨迹并发布"""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'body_foot',
                                                     rospy.Time(0), rospy.Duration(0.1))
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            self.actual_path_points.append((x, y))

            if len(self.actual_path_points) > self.max_actual_path_length:
                self.actual_path_points.pop(0)

            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"

            for px, py in self.actual_path_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.actual_path_pub.publish(path_msg)

        except Exception:
            pass

    def a_star_search(self, start, goal, cost_map):
        """
        A* 搜索算法。
        使用预膨胀的障碍物地图，碰撞检测仅需一次查表，极大加速搜索。
        支持8方向移动，对角线代价为√2，使路径更自然。
        """
        rows, cols = cost_map.shape
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        # 8方向移动向量与对应代价
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),       # 四方向
            (1, 1), (-1, -1), (1, -1), (-1, 1)       # 对角线
        ]
        move_costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

        while open_set:
            current_cost, current = heappop(open_set)
            cx, cy = current

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for (dx, dy), cost in zip(directions, move_costs):
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < cols and 0 <= ny < rows):
                    continue

                # 快速碰撞检测：预膨胀地图，一次查表即可
                if cost_map[ny, nx] == 100:
                    continue

                next_pos = (nx, ny)
                tentative_g = g_score[current] + cost

                if next_pos not in g_score or tentative_g < g_score[next_pos]:
                    came_from[next_pos] = current
                    g_score[next_pos] = tentative_g
                    f_score = tentative_g + self.heuristic(next_pos, goal)
                    heappush(open_set, (f_score, next_pos))

        return []

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    def smooth_path(self, path):
        """
        基于梯度下降的路径平滑算法。
        在"贴近原始路径"和"路径光滑"之间取得平衡。
        增加迭代次数，使差速车行驶更平顺。
        """
        if len(path) < 3:
            return path

        path = np.array(path, dtype=np.float64)
        smooth = np.copy(path)

        for iteration in range(self.smooth_iterations):
            prev_smooth = np.copy(smooth)

            for i in range(1, len(path) - 1):
                # 数据项：吸引到原始路径，防止过度平滑偏离
                term_data = self.smooth_weight_data * (path[i] - smooth[i])
                # 平滑项：拉向相邻点的中心（使路径光滑）
                term_smooth = self.smooth_weight_smooth * (
                    smooth[i-1] + smooth[i+1] - 2.0 * smooth[i])
                smooth[i] += term_data + term_smooth

            # 检查是否收敛
            diff = np.sum(np.abs(smooth - prev_smooth))
            if diff < self.tolerance:
                rospy.loginfo(f"平滑在第 {iteration+1} 次迭代收敛")
                break

        return smooth.tolist()

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin.position.x) / self.resolution)
        my = int((y - self.map_origin.position.y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = (mx + 0.5) * self.resolution + self.map_origin.position.x
        wy = (my + 0.5) * self.resolution + self.map_origin.position.y
        return (wx, wy)

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for px, py in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


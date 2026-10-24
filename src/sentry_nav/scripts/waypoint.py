#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import yaml
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


YAML_FILE_PATH = "src/sentry_nav/config/waypoint.yaml"  # 你的YAML文件绝对路径
LOOP_PATROL = True    # 是否循环巡检：True=循环，False=单次
STAY_TIME = 0        # 到达点后停留时间（秒），0=不停留
MAP_FRAME = "map"      # 地图坐标系，固定为map


def load_waypoints_from_yaml(yaml_path):
    """加载YAML中的导航点（位置x/y + 四元数ox/oy/oz/ow）"""
    if not os.path.exists(yaml_path):
        rospy.logerr(f"YAML文件不存在！路径：{yaml_path}")
        return None
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        # 校验YAML格式
        if "waypoints" not in data or not isinstance(data["waypoints"], list):
            rospy.logerr("YAML格式错误！必须包含`waypoints`列表")
            return None
        # 解析导航点，过滤无效点
        patrol_points = []
        for idx, wp in enumerate(data["waypoints"]):
            required_keys = ["x", "y", "ox", "oy", "oz", "ow"]
            if all(k in wp for k in required_keys):
                patrol_points.append({
                    "x": wp["x"], "y": wp["y"],
                    "ox": wp["ox"], "oy": wp["oy"], "oz": wp["oz"], "ow": wp["ow"]
                })
                rospy.loginfo(f"解析第{idx+1}个点：x={wp['x']:.2f}, y={wp['y']:.2f}")
            else:
                rospy.logwarn(f"第{idx+1}个点格式错误（缺少字段），已跳过")
        # 校验有效点数量
        if len(patrol_points) == 0:
            rospy.logerr("YAML中无有效导航点！")
            return None
        rospy.loginfo(f"成功加载{len(patrol_points)}个有效导航点")
        return patrol_points
    except Exception as e:
        rospy.logerr(f"读取YAML失败：{str(e)}")
        return None

def create_movebase_goal(x, y, ox, oy, oz, ow):
    """构造MoveBase导航目标：直接复用原始四元数，无任何转换"""
    goal = MoveBaseGoal()
    # 设置消息头
    goal.target_pose.header.frame_id = MAP_FRAME
    goal.target_pose.header.stamp = rospy.Time.now()
    # 设置位置（2D导航z固定为0）
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    # 设置朝向（直接赋值原始四元数，核心！）
    goal.target_pose.pose.orientation.x = ox
    goal.target_pose.pose.orientation.y = oy
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    return goal

def main():
    # 初始化ROS节点
    rospy.init_node('yaml_waypoint_patrol_node', anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("===== 导航点巡检节点启动 =====")
    rospy.loginfo(f"配置：循环巡检={LOOP_PATROL} | 点停留时间={STAY_TIME}s")

    # 第一步：加载YAML导航点
    patrol_points = load_waypoints_from_yaml(YAML_FILE_PATH)
    if patrol_points is None:
        rospy.logerr("===== 加载导航点失败，节点退出 =====")
        return

    # 第二步：连接MoveBase Action服务（超时10秒）
    rospy.loginfo("等待MoveBase服务启动...")
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    if not move_base_client.wait_for_server(rospy.Duration(10)):
        rospy.logerr("❌ MoveBase服务连接失败！请检查MoveBase是否启动")
        rospy.logerr("===== 节点退出 =====")
        return
    rospy.loginfo("✅ MoveBase服务连接成功，开始按序巡检！")

    # 第三步：按序发布导航点（核心逻辑）
    while not rospy.is_shutdown():
        for idx, wp in enumerate(patrol_points):
            # 解析当前点的位置和四元数
            x, y = wp["x"], wp["y"]
            ox, oy, oz, ow = wp["ox"], wp["oy"], wp["oz"], wp["ow"]
            # 构造并发布导航目标
            goal = create_movebase_goal(x, y, ox, oy, oz, ow)
            rospy.loginfo(f"\n===== 发布第{idx+1}/{len(patrol_points)}个导航点 =====")
            rospy.loginfo(f"位置：x={x:.2f}, y={y:.2f}")
            rospy.loginfo(f"四元数：oz={oz:.4f}, ow={ow:.4f}")
            move_base_client.send_goal(goal)
            # 阻塞等待导航完成（直到到达/失败/手动取消）
            move_base_client.wait_for_result()
            # 获取导航结果并判定
            current_status = move_base_client.get_state()
            if current_status == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✅ 第{idx+1}个点导航成功！停留{STAY_TIME}s")
                rospy.sleep(STAY_TIME)
            else:
                rospy.logwarn(f"⚠️  第{idx+1}个点导航失败！状态码：{current_status}")
                rospy.logwarn(f"⚠️  跳过当前点，继续下一个")
        
        # 单次巡检：执行完所有点后退出
        if not LOOP_PATROL:
            rospy.loginfo("\n===== 所有导航点巡检完成，节点正常退出 =====")
            break
        # 循环巡检：执行完一轮后，等待1秒重新开始
        else:
            rospy.loginfo("\n===== 一轮巡检完成，1秒后开始下一轮 =====")
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("===== 节点被手动中断（Ctrl+C） =====")
    except Exception as e:
        rospy.logerr(f"===== 节点运行出错：{str(e)} =====")


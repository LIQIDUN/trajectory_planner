#!/usr/bin/env python3
import rospy
import heapq
import math
import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from std_msgs.msg import Header

# ==========================================
# 1. A* 核心算法类
# ==========================================
class AStar:
    def __init__(self, width, height, resolution, obstacles):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.obstacles = obstacles # set of (x, y) tuples in grid coords
        
        # 8-邻域移动方向 (dx, dy, cost)
        self.motions = [
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

    def heuristic(self, node, goal):
        # 欧几里得距离作为启发函数
        return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

    def search(self, start, goal):
        # start, goal 都是 (grid_x, grid_y)
        
        # 优先队列 (cost, current_node)
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        came_from = {}       # 记录路径: child -> parent
        cost_so_far = {}     # 记录代价: node -> cost
        
        came_from[start] = None
        cost_so_far[start] = 0
        
        current = None
        
        while open_list:
            # 取出代价最小的节点
            _, current = heapq.heappop(open_list)
            
            if current == goal:
                break
            
            for dx, dy, move_cost in self.motions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 1. 越界检查
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                    continue
                
                # 2. 障碍物检查
                if neighbor in self.obstacles:
                    continue
                
                new_cost = cost_so_far[current] + move_cost
                
                # 3. 如果发现了更短的路径，或者该节点未访问过
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current
        
        return self.reconstruct_path(came_from, start, goal) if current == goal else []

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None: # 防御性编程
                return []
        path.append(start)
        path.reverse()
        return path

# ==========================================
# 2. ROS 节点封装
# ==========================================
class PathPlannerNode:
    def __init__(self):
        rospy.init_node('astar_node', anonymous=True)
        
        # --- 参数设置 ---
        self.map_width = 20   # 20个格子宽
        self.map_height = 20  # 20个格子高
        self.resolution = 0.5 # 每个格子 0.5米
        self.origin_x = -5.0  # 地图原点世界坐标
        self.origin_y = -5.0
        
        # --- 创建虚拟障碍物 (这里手动写死一些障碍物) ---
        # 在地图中间画一堵墙
        self.obstacles = set()
        for i in range(5, 15):
            self.obstacles.add((10, i)) # 竖线
            self.obstacles.add((i, 10)) # 横线
            
        self.astar = AStar(self.map_width, self.map_height, self.resolution, self.obstacles)

        # --- Publishers ---
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        
        # --- Subscribers ---
        # 使用 RViz 的 "2D Nav Goal" 工具作为终点
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        # 使用 RViz 的 "2D Pose Estimate" 工具作为起点 (可选，默认起点为 0,0)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_cb)
        
        self.start_grid = (2, 2) # 默认起点索引
        
        # 启动后先发布一次地图
        self.publish_map()
        rospy.loginfo("A* Planner Ready! Use '2D Nav Goal' in RViz to verify.")

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x + self.resolution / 2.0
        wy = gy * self.resolution + self.origin_y + self.resolution / 2.0
        return wx, wy

    def start_cb(self, msg):
        gx, gy = self.world_to_grid(msg.pose.pose.position.x, msg.pose.pose.position.y)
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            self.start_grid = (gx, gy)
            rospy.loginfo(f"Start Point Set to Grid: {self.start_grid}")
            # 重新发布地图以刷新起点显示（如果需要可视化起点）
        else:
            rospy.logwarn("Start point out of map bounds!")

    def goal_cb(self, msg):
        # 1. 获取目标点世界坐标
        goal_wx = msg.pose.position.x
        goal_wy = msg.pose.position.y
        
        # 2. 转为栅格坐标
        gx, gy = self.world_to_grid(goal_wx, goal_wy)
        
        if not (0 <= gx < self.map_width and 0 <= gy < self.map_height):
            rospy.logwarn("Goal out of map bounds!")
            return
            
        goal_grid = (gx, gy)
        rospy.loginfo(f"Planning from {self.start_grid} to {goal_grid}...")
        
        # 3. 运行 A*
        path_grids = self.astar.search(self.start_grid, goal_grid)
        
        if not path_grids:
            rospy.logwarn("No path found!")
            return
            
        rospy.loginfo(f"Path found! Length: {len(path_grids)}")
        
        # 4. 发布路径用于显示
        self.publish_path(path_grids)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        # 填充数据: -1:未知, 0:空闲, 100:障碍
        # data 是一个一维数组，行优先
        data = [0] * (self.map_width * self.map_height)
        
        for (obs_x, obs_y) in self.obstacles:
            idx = obs_y * self.map_width + obs_x
            if 0 <= idx < len(data):
                data[idx] = 100
                
        msg.data = data
        self.map_pub.publish(msg)

    def publish_path(self, path_grids):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for (gx, gy) in path_grids:
            wx, wy = self.grid_to_world(gx, gy)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0 # 2D 规划，高度设为0
            # 姿态默认 (0,0,0,1)
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        node = PathPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import heapq
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

# ==========================================
# 1. 3D A* 核心算法类
# ==========================================
class AStar3D:
    def __init__(self, x_size, y_size, z_size, resolution, obstacles):
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.resolution = resolution
        self.obstacles = obstacles # set of (gx, gy, gz)
        
        # 生成 26 个邻居移动方向 (x, y, z)
        self.motions = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    # 代价是距离 (1 或 1.414 或 1.732)
                    cost = math.sqrt(dx**2 + dy**2 + dz**2)
                    self.motions.append(((dx, dy, dz), cost))

    def heuristic(self, node, goal):
        # 3D 欧几里得距离
        return math.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2 + (node[2]-goal[2])**2)

    def search(self, start, goal):
        # start, goal 都是 (gx, gy, gz)
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        came_from = {}
        cost_so_far = {}
        
        came_from[start] = None
        cost_so_far[start] = 0
        
        current = None
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            if current == goal:
                break
            
            for move, move_cost in self.motions:
                dx, dy, dz = move
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                
                # 1. 越界检查 (3D)
                if not (0 <= neighbor[0] < self.x_size and 
                        0 <= neighbor[1] < self.y_size and 
                        0 <= neighbor[2] < self.z_size):
                    continue
                
                # 2. 障碍物检查
                if neighbor in self.obstacles:
                    continue
                
                new_cost = cost_so_far[current] + move_cost
                
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
            if current is None:
                return []
        path.append(start)
        path.reverse()
        return path

# ==========================================
# 2. ROS 节点封装
# ==========================================
class PathPlannerNode3D:
    def __init__(self):
        rospy.init_node('astar_node_3d', anonymous=True)
        
        # --- 地图参数 ---
        self.resolution = 0.2  # 精度更高：0.2米一个格子
        self.map_x_min = -5.0
        self.map_x_max = 5.0
        self.map_y_min = -5.0
        self.map_y_max = 5.0
        self.map_z_min = 0.0
        self.map_z_max = 3.0   # 最高飞3米
        
        # 计算网格尺寸
        self.gx_size = int((self.map_x_max - self.map_x_min) / self.resolution)
        self.gy_size = int((self.map_y_max - self.map_y_min) / self.resolution)
        self.gz_size = int((self.map_z_max - self.map_z_min) / self.resolution)
        
        rospy.loginfo(f"Map Size: {self.gx_size}x{self.gy_size}x{self.gz_size}")

        # --- 生成 3D 障碍物 ---
        self.obstacles = set()
        self.generate_obstacles()
        
        self.astar = AStar3D(self.gx_size, self.gy_size, self.gz_size, self.resolution, self.obstacles)

        # --- Publishers ---
        self.path_pub = rospy.Publisher('/planned_path_3d', Path, queue_size=1)
        self.obs_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=1, latch=True)
        
        # --- Subscribers ---
        # RViz 的 "2D Nav Goal" 只能给 x,y。我们手动给 z 赋值
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        
        # 默认起点：(0, 0, 1.0) 1米高度
        self.start_grid = self.world_to_grid(0, 0, 1.0)
        
        # 发布一次障碍物显示
        self.publish_obstacles()
        rospy.loginfo("3D A* Ready! Use '2D Nav Goal' in RViz. Goal height is fixed to 1.5m for testing.")

    def generate_obstacles(self):
        # 场景：在 x=3.0 处造一堵墙，但中间留个窗户
        wall_x_world = 3.0
        wall_gx = int((wall_x_world - self.map_x_min) / self.resolution)
        
        for gy in range(self.gy_size):
            for gz in range(self.gz_size):
                # 窗户范围：y 在 [-1, 1] 之间，z 在 [1.0, 2.0] 之间
                wy, wz = self.grid_to_world_val(gy, gz)
                
                if -1.0 < wy < 1.0 and 1.0 < wz < 2.0:
                    continue # 这是窗户，不放障碍
                
                self.obstacles.add((wall_gx, gy, gz))

    def world_to_grid(self, wx, wy, wz):
        gx = int((wx - self.map_x_min) / self.resolution)
        gy = int((wy - self.map_y_min) / self.resolution)
        gz = int((wz - self.map_z_min) / self.resolution)
        return gx, gy, gz

    def grid_to_world_val(self, gy, gz):
        # 辅助函数：只算 y 和 z 的世界坐标
        wy = gy * self.resolution + self.map_y_min + self.resolution/2
        wz = gz * self.resolution + self.map_z_min + self.resolution/2
        return wy, wz

    def grid_to_world(self, gx, gy, gz):
        wx = gx * self.resolution + self.map_x_min + self.resolution/2
        wy = gy * self.resolution + self.map_y_min + self.resolution/2
        wz = gz * self.resolution + self.map_z_min + self.resolution/2
        return wx, wy, wz

    def goal_cb(self, msg):
        # 1. 获取目标点 (RViz 只给 2D，我们强制设置高度)
        goal_wx = msg.pose.position.x
        goal_wy = msg.pose.position.y
        goal_wz = 1.5 # <--- 强制目标高度 1.5米 (为了穿过窗户)
        
        rospy.loginfo(f"New Goal Received: ({goal_wx:.2f}, {goal_wy:.2f}, {goal_wz:.2f})")
        
        gx, gy, gz = self.world_to_grid(goal_wx, goal_wy, goal_wz)
        
        if not (0 <= gx < self.gx_size and 0 <= gy < self.gy_size and 0 <= gz < self.gz_size):
            rospy.logwarn("Goal out of bounds!")
            return

        goal_grid = (gx, gy, gz)
        
        # 2. 运行 A*
        path_grids = self.astar.search(self.start_grid, goal_grid)
        
        if not path_grids:
            rospy.logwarn("No 3D path found!")
            return
            
        rospy.loginfo(f"Path found! Length: {len(path_grids)}")
        self.publish_path(path_grids)

    def publish_path(self, path_grids):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for (gx, gy, gz) in path_grids:
            wx, wy, wz = self.grid_to_world(gx, gy, gz)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = wz
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def publish_obstacles(self):
        # 使用 MarkerArray 显示 3D 障碍物 (一个个小方块)
        marker_array = MarkerArray()
        
        # 为了性能，我们把所有方块合并成一个 CUBE_LIST
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5 # 半透明
        
        for (gx, gy, gz) in self.obstacles:
            wx, wy, wz = self.grid_to_world(gx, gy, gz)
            p = Point()
            p.x = wx
            p.y = wy
            p.z = wz
            marker.points.append(p)
            
        marker_array.markers.append(marker)
        self.obs_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        node = PathPlannerNode3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

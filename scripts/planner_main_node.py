#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

# ==========================================
# 关键步骤：导入同目录下的模块
# ==========================================
from astar_algorithm import AStar3D
from min_snap_algorithm import MinSnapOptimizer

class PathPlannerNode3D:
    def __init__(self):
        rospy.init_node('astar_node_3d', anonymous=True)
        
        # --- 参数 ---
        self.resolution = 0.2
        self.map_x_min = -5.0; self.map_x_max = 5.0
        self.map_y_min = -5.0; self.map_y_max = 5.0
        self.map_z_min = 0.0;  self.map_z_max = 3.0
        
        self.gx_size = int((self.map_x_max - self.map_x_min) / self.resolution)
        self.gy_size = int((self.map_y_max - self.map_y_min) / self.resolution)
        self.gz_size = int((self.map_z_max - self.map_z_min) / self.resolution)
        
        # --- 实例化障碍物 ---
        self.obstacles = set()
        self.generate_obstacles()
        
        # --- 实例化算法模块 ---
        self.astar = AStar3D(self.gx_size, self.gy_size, self.gz_size, self.resolution, self.obstacles)
        self.optimizer = MinSnapOptimizer()

        # --- Publishers ---
        self.path_pub = rospy.Publisher('/planned_path_3d', Path, queue_size=1)
        self.smooth_path_pub = rospy.Publisher('/smoothed_path_3d', Path, queue_size=1)
        self.obs_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=1, latch=True)
        
        # --- Subscriber ---
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.start_grid = self.world_to_grid(0, 0, 1.0)
        
        # 初始显示
        self.publish_obstacles()
        rospy.loginfo("Modular 3D Planner Ready! Click '2D Nav Goal' in RViz.")

    def generate_obstacles(self):
        # 创建一个带窗户的墙
        wall_x = 2.0
        wall_gx = int((wall_x - self.map_x_min) / self.resolution)
        for gy in range(self.gy_size):
            for gz in range(self.gz_size):
                wy, wz = self.grid_to_world_val(gy, gz)
                if -1.0 < wy < 1.0 and 1.0 < wz < 2.0: continue
                self.obstacles.add((wall_gx, gy, gz))

    def world_to_grid(self, wx, wy, wz):
        gx = int((wx - self.map_x_min) / self.resolution)
        gy = int((wy - self.map_y_min) / self.resolution)
        gz = int((wz - self.map_z_min) / self.resolution)
        return gx, gy, gz

    def grid_to_world(self, gx, gy, gz):
        wx = gx * self.resolution + self.map_x_min + self.resolution/2
        wy = gy * self.resolution + self.map_y_min + self.resolution/2
        wz = gz * self.resolution + self.map_z_min + self.resolution/2
        return wx, wy, wz
        
    def grid_to_world_val(self, gy, gz):
        wy = gy * self.resolution + self.map_y_min + self.resolution/2
        wz = gz * self.resolution + self.map_z_min + self.resolution/2
        return wy, wz

    def goal_cb(self, msg):
        goal_wx = msg.pose.position.x
        goal_wy = msg.pose.position.y
        goal_wz = 1.5 
        
        gx, gy, gz = self.world_to_grid(goal_wx, goal_wy, goal_wz)
        if not (0 <= gx < self.gx_size and 0 <= gy < self.gy_size and 0 <= gz < self.gz_size): return
        
        # 1. 调用 A* 模块
        path_grids = self.astar.search(self.start_grid, (gx, gy, gz))
        if not path_grids:
            rospy.logwarn("No path found!")
            return
            
        rospy.loginfo(f"A* Path Found: {len(path_grids)} points. Optimizing...")
        
        # 转换坐标
        waypoints = []
        for p in path_grids:
            waypoints.append(self.grid_to_world(*p))
            
        # 2. 调用 MinSnap 模块
        smooth_path_points = self.optimizer.optimize(waypoints)
        
        # 3. 发布结果
        self.publish_path(self.path_pub, waypoints)
        self.publish_path(self.smooth_path_pub, smooth_path_points)

    def publish_path(self, publisher, points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for p in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        publisher.publish(path_msg)

    def publish_obstacles(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "obstacles"; marker.id = 0; marker.type = Marker.CUBE_LIST; marker.action = Marker.ADD
        marker.scale.x = self.resolution; marker.scale.y = self.resolution; marker.scale.z = self.resolution
        marker.color.r = 1.0; marker.color.a = 0.5
        for (gx, gy, gz) in self.obstacles:
            wx, wy, wz = self.grid_to_world(gx, gy, gz)
            p = Point(); p.x = wx; p.y = wy; p.z = wz
            marker.points.append(p)
        marker_array.markers.append(marker)
        self.obs_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        node = PathPlannerNode3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
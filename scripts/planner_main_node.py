#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

# ==========================================
# 关键步骤：导入同目录下的模块
# ==========================================
from astar_algorithm import AStar3D
from min_snap_algorithm import MinSnapOptimizer
from overactuated_driver.msg import TrajectoryCommand

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
        
        # ★ 控制器指令发布器
        self.traj_cmd_pub = rospy.Publisher('/uav/trajectory_command', TrajectoryCommand, queue_size=10)
        
        # --- 轨迹执行状态 ---
        self.traj_timer = None
        self.traj_start_time = None
        self.current_traj_x = None
        self.current_traj_y = None
        self.current_traj_z = None
        self.current_durations = None
        self.is_running = False  # 是否正在执行轨迹
        self.trajectory_data = None # 暂存轨迹数据
        self.target_mode = 29

        # --- Subscriber ---
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.start_grid = self.world_to_grid(0, 0, 0)
        rospy.Subscriber('/uav/custom_mode', Int32, self.mode_cb)

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
        smooth_path_points, traj_x, traj_y, traj_z, durations = self.optimizer.optimize(waypoints)
        
        # 3. 发布结果
        self.publish_path(self.path_pub, waypoints)
        self.publish_path(self.smooth_path_pub, smooth_path_points)
        # 更新起点为终点，方便连续点击规划
        self.start_grid = (gx, gy, gz)
        # 3. 
        if traj_x is not None:
            # 仅仅保存数据，不启动定时器
            self.trajectory_data = {
                'x': traj_x, 'y': traj_y, 'z': traj_z, 'durations': durations
            }
            self.is_running = False 
            rospy.loginfo("轨迹已保存。请切换 ArduPilot 到 Mode 29 开始飞行。")

            if self.traj_timer is not None:
                self.traj_timer.shutdown()
                self.traj_timer = None
            # 开启 100Hz 的定时器向几何控制器发指令
            # self.traj_timer = rospy.Timer(rospy.Duration(0.01), self.execute_trajectory_cb)
            # rospy.loginfo("Trajectory optimization success! Executing at 100Hz...")
    
    def mode_cb(self, msg):
        current_mode = msg.data
        
        # 触发条件：有保存的轨迹 + 模式切到了 29 + 当前没在运行
        if current_mode == self.target_mode and self.trajectory_data is not None:
            if not self.is_running:
                rospy.loginfo("检测到 Mode 29！开始发布轨迹数据...")

                # 【新增】必须在这里把数据提取出来，否则定时器回调里全为 None！
                self.current_traj_x = self.trajectory_data['x']
                self.current_traj_y = self.trajectory_data['y']
                self.current_traj_z = self.trajectory_data['z']
                self.current_durations = self.trajectory_data['durations']
                
                self.traj_start_time = rospy.Time.now()
                self.is_running = True
                
                # 启动定时器发布指令
                if self.traj_timer is not None:
                    self.traj_timer.shutdown()
                self.traj_timer = rospy.Timer(rospy.Duration(0.01), self.execute_trajectory_cb)
        
        # 退出条件：如果切离了模式 29，停止发布
        elif current_mode != self.target_mode and self.is_running:
            rospy.logwarn("模式已切换，停止轨迹发布。")
            self.is_running = False
            if self.traj_timer:
                self.traj_timer.shutdown()     
                self.traj_timer = None   
            


    def execute_trajectory_cb(self, event):
        if not self.is_running or self.trajectory_data is None or self.current_durations is None: return
            
        elapsed_time = (rospy.Time.now() - self.traj_start_time).to_sec()
        
        curr_t = elapsed_time
        seg_idx = -1
        for i, T in enumerate(self.current_durations):
            if curr_t <= T:
                seg_idx = i
                break
            curr_t -= T 
            
        if seg_idx == -1:
            rospy.loginfo("Trajectory Execution Finished!")
            self.traj_timer.shutdown()
            self.current_durations = None
            return
            
        # 提取当前段系数与相对时间
        cx, cy, cz = self.current_traj_x[seg_idx], self.current_traj_y[seg_idx], self.current_traj_z[seg_idx]
        t = curr_t
       # 2. 定义求导闭包函数，减少重复代码
        def eval_p(c, t): return c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
        def eval_v(c, t): return c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
        def eval_a(c, t): return 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3
        def eval_j(c, t): return 6*c[3] + 24*c[4]*t + 60*c[5]*t**2
        def eval_s(c, t): return 24*c[4] + 120*c[5]*t

        # 3. 计算 ENU 坐标系下的各阶导数
        # World X (East), World Y (North), World Z (Up)
        p_enu = [eval_p(cx, t), eval_p(cy, t), eval_p(cz, t)]
        v_enu = [eval_v(cx, t), eval_v(cy, t), eval_v(cz, t)]
        a_enu = [eval_a(cx, t), eval_a(cy, t), eval_a(cz, t)]
        j_enu = [eval_j(cx, t), eval_j(cy, t), eval_j(cz, t)]
        s_enu = [eval_s(cx, t), eval_s(cy, t), eval_s(cz, t)]

        cmd = TrajectoryCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world_ned" # 建议修改 frame_id 提醒下游控制器

        # 4. 坐标映射: ENU -> NED
        # NED.x = North = ENU.y
        # NED.y = East  = ENU.x
        # NED.z = Down  = -ENU.z
        
        cmd.pos.x, cmd.pos.y, cmd.pos.z = p_enu[1], p_enu[0], -p_enu[2]
        cmd.vel.x, cmd.vel.y, cmd.vel.z = v_enu[1], v_enu[0], -v_enu[2]
        cmd.acc.x, cmd.acc.y, cmd.acc.z = a_enu[1], a_enu[0], -a_enu[2]
        cmd.jerk.x, cmd.jerk.y, cmd.jerk.z = j_enu[1], j_enu[0], -j_enu[2]
        cmd.snap.x, cmd.snap.y, cmd.snap.z = s_enu[1], s_enu[0], -s_enu[2]

        # 5. 角度运动 (Head)
        # 根据你的要求，head 指向 x=1, y=0, z=0 (通常对应 NED 中的正北)
        cmd.head.x = 1.0
        cmd.head.y = 0.0
        cmd.head.z = 0.0
        
        # 也可以设置为速度方向的 Yaw (NED 下)
        # cmd.head.z = math.atan2(cmd.vel.y, cmd.vel.x) 

        cmd.head_rate = Vector3(0, 0, 0)
        cmd.head_acc = Vector3(0, 0, 0)
        
        self.traj_cmd_pub.publish(cmd)


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
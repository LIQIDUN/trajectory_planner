import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class MapManager:
    def generate_obstacles(self):
            # 创建一个带窗户的墙
            # ENU坐标系下，墙在 x=2.0 处，y 从 -5 到 5，z 从 0 到 3
            # wall_x = 2.0
            # wall_gx = int((wall_x - self.map_x_min) / self.resolution)
            # for gy in range(self.gy_size):
            #     for gz in range(self.gz_size):
            #         wy, wz = self.grid_to_world_val(gy, gz)
            #         if -1.0 < wy < 1.0 and 1.0 < wz < 2.0: continue
            #         self.obstacles.add((wall_gx, gy, gz))
            wall_y = -12.0
            wall_gy = int((wall_y - self.map_y_min) / self.resolution)
            for gx in range(self.gx_size):
                for gz in range(self.gz_size):
                    wx, wz = MapManager.grid_to_world_val(self, gx, gz)
                    if -2.0 < wx < 2.0 and 0.0 < wz < 3.0: self.obstacles.add((gx, wall_gy, gz))
                    

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
    
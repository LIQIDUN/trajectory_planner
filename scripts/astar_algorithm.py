#!/usr/bin/env python3
import heapq
import math

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
                neighbor = (current[0] + move[0], current[1] + move[1], current[2] + move[2])
                
                # 越界检查
                if not (0 <= neighbor[0] < self.x_size and 
                        0 <= neighbor[1] < self.y_size and 
                        0 <= neighbor[2] < self.z_size):
                    continue
                
                # 障碍物检查
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
                return [] # 路径断裂保护
        path.append(start)
        path.reverse()
        return path
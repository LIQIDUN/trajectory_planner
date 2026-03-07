#!/usr/bin/env python3
import numpy as np
import rospy
import math

class MinSnapOptimizer:
    def __init__(self):
        # 平均速度 (m/s)
        self.avg_vel = 0.1 

    def optimize(self, waypoints_raw):
        """
        waypoints_raw: A* 输出的原始稠密路径 [[x,y,z], ...]
        return: 平滑后的密集轨迹点
        """
        if len(waypoints_raw) < 2:
            return waypoints_raw, None, None, None, None

        # --- 步骤 1: 关键！路径稀疏化 ---
        # 只保留拐点，去掉共线的中间点，解决数值不稳定问题
        waypoints = self.simplify_waypoints(waypoints_raw)
        
        rospy.loginfo(f"Path Optimized: Raw {len(waypoints_raw)} -> Simplified {len(waypoints)} waypoints")

        # 如果简化后只剩起点终点，直接返回直线插值
        if len(waypoints) < 2:
            return waypoints, None, None, None, None

        # --- 步骤 2: 时间分配 ---
        n_segments = len(waypoints) - 1
        durations = []
        for i in range(n_segments):
            dist = np.linalg.norm(np.array(waypoints[i+1]) - np.array(waypoints[i]))
            # 这里的 max(t, 0.5) 很关键，每段至少给 0.5秒，保证矩阵数值稳定
            t = dist / self.avg_vel
            durations.append(max(t, 0.5)) 
        
        # --- 步骤 3: Minimum Jerk (5阶) 闭式求解 ---
        # 针对 X, Y, Z 三轴分别求解
        traj_x = self.solve_axis(waypoints, durations, 0)
        traj_y = self.solve_axis(waypoints, durations, 1)
        traj_z = self.solve_axis(waypoints, durations, 2)
        
        # 检查是否求解失败
        if traj_x is None or traj_y is None or traj_z is None:
            rospy.logerr("MinSnap Solver Failed! Returning raw path.")
            return waypoints, None, None, None, None

        # --- 步骤 4: 采样生成显示点 ---
        dense_path = []
        for i in range(n_segments):
            T = durations[i]
            coeffs_x = traj_x[i]
            coeffs_y = traj_y[i]
            coeffs_z = traj_z[i]
            
            # 采样密度：每 0.05秒 一个点
            num_points = int(T / 0.05)
            for t in np.linspace(0, T, num_points):
                px = self.poly_eval(coeffs_x, t)
                py = self.poly_eval(coeffs_y, t)
                pz = self.poly_eval(coeffs_z, t)
                dense_path.append((px, py, pz))
                
        dense_path.append(waypoints[-1])
        return dense_path, traj_x, traj_y, traj_z, durations

    def simplify_waypoints(self, path):
        """
        去除共线点，只保留拐点。
        原理：计算相邻三点的向量叉积，如果接近0则共线。
        """
        if len(path) < 3:
            return path
            
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            p_prev = np.array(path[i-1])
            p_curr = np.array(path[i])
            p_next = np.array(path[i+1])
            
            vec1 = p_curr - p_prev
            vec2 = p_next - p_curr
            
            # 归一化向量
            dist1 = np.linalg.norm(vec1)
            dist2 = np.linalg.norm(vec2)
            
            if dist1 < 0.01 or dist2 < 0.01:
                continue # 点太近，忽略
                
            vec1 /= dist1
            vec2 /= dist2
            
            # 检查共线 (方向向量的夹角)
            # 如果点积接近 1，说明方向相同，共线，可以去掉 p_curr
            if np.dot(vec1, vec2) < 0.999: 
                simplified.append(path[i]) # 不共线，保留
        
        simplified.append(path[-1])
        return simplified

    def solve_axis(self, waypoints, durations, axis):
        n_seg = len(durations)
        n_coef = 6 # Minimum Jerk (5阶多项式) -> p, v, a, j, s
        
        # 矩阵维度: n_seg * n_coef
        dim = n_seg * n_coef
        A = np.zeros((dim, dim))
        b = np.zeros(dim)
        
        row = 0
        
        # ==========================
        # 1. 强约束：必须经过每一个 Waypoint
        # ==========================
        for i in range(n_seg):
            # 每一段的 Start (t=0) 必须等于 Waypoint[i]
            # p_i(0) = c0 = W_i
            A[row, i*n_coef + 0] = 1 
            b[row] = waypoints[i][axis]
            row += 1
            
            # 每一段的 End (t=T) 必须等于 Waypoint[i+1]
            # p_i(T) = ... = W_{i+1}
            T = durations[i]
            for j in range(n_coef):
                A[row, i*n_coef + j] = T**j
            b[row] = waypoints[i+1][axis]
            row += 1

        # ==========================
        # 2. 连续性约束 (PVAJ 连续)
        # ==========================
        # 对每一个连接点 (Seg_i 和 Seg_{i+1})
        for i in range(n_seg - 1):
            T = durations[i]
            
            # 速度连续 v_i(T) - v_{i+1}(0) = 0
            # v_i(T): c1 + 2c2T + 3c3T^2 ...
            # v_{i+1}(0): c1 (of next segment)
            
            # Vel
            for j in range(1, n_coef): A[row, i*n_coef + j] = j * T**(j-1)
            A[row, (i+1)*n_coef + 1] = -1
            row += 1
            
            # Acc
            for j in range(2, n_coef): A[row, i*n_coef + j] = j*(j-1) * T**(j-2)
            A[row, (i+1)*n_coef + 2] = -2
            row += 1
            
            # Jerk
            for j in range(3, n_coef): A[row, i*n_coef + j] = j*(j-1)*(j-2) * T**(j-3)
            A[row, (i+1)*n_coef + 3] = -6
            row += 1
            
            # Snap (Optional for Min Jerk, but helps smoothness)
            for j in range(4, n_coef): A[row, i*n_coef + j] = j*(j-1)*(j-2)*(j-3) * T**(j-4)
            A[row, (i+1)*n_coef + 4] = -24
            row += 1

        # ==========================
        # 3. 起点和终点 边界条件 (静止)
        # ==========================
        # Start (Seg 0, t=0): Vel=0, Acc=0
        A[row, 1] = 1; row += 1 # v0 = 0
        A[row, 2] = 2; row += 1 # a0 = 0
        
        # End (Seg N-1, t=T_last): Vel=0, Acc=0
        T_last = durations[-1]
        last_idx = (n_seg-1)*n_coef
        
        # Vel End
        for j in range(1, n_coef): A[row, last_idx + j] = j * T_last**(j-1)
        row += 1
        # Acc End
        for j in range(2, n_coef): A[row, last_idx + j] = j*(j-1) * T_last**(j-2)
        row += 1

        # 检查行数是否匹配 (调试用)
        # expected_rows = 2*n_seg + 4*(n_seg-1) + 4
        # if row != dim:
        #     print(f"Matrix Dimension Mismatch! Row={row}, Dim={dim}")

        # 求解
        try:
            # 使用伪逆求解 (比 solve 更鲁棒，能处理由于自由度导致的轻微奇异)
            x = np.linalg.pinv(A).dot(b)
        except np.linalg.LinAlgError:
            return None
            
        coeffs = []
        for i in range(n_seg):
            coeffs.append(x[i*n_coef : (i+1)*n_coef])
        return coeffs

    def poly_eval(self, coeffs, t):
        val = 0
        for i, c in enumerate(coeffs):
            val += c * (t**i)
        return val
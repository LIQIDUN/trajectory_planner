import numpy as np

def point_to_line_distance_3d(point, line_start, line_end):
    """计算3D空间中点到线段的距离"""
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.linalg.norm(line_vec)
    
    if line_len == 0.0: # 起点和终点重合
        return np.linalg.norm(point_vec)
        
    # 利用叉乘计算点到直线的垂直距离 (平行四边形面积 / 底边长)
    distance = np.linalg.norm(np.cross(line_vec, point_vec)) / line_len
    return distance

def rdp_simplify(waypoints, epsilon=0.5):
    """
    Ramer-Douglas-Peucker 抽稀算法
    :param waypoints: list of [x, y, z]
    :param epsilon: 允许的最大误差距离，值越大删的点越多（例如0.5个栅格大小）
    :return: 抽稀后的 waypoints
    """
    if len(waypoints) < 3:
        return waypoints

    pts = np.array(waypoints)
    start_pt = pts[0]
    end_pt = pts[-1]

    # 找到距离首尾连线最远的点
    max_dist = 0.0
    index_of_max = 0
    for i in range(1, len(pts) - 1):
        dist = point_to_line_distance_3d(pts[i], start_pt, end_pt)
        if dist > max_dist:
            max_dist = dist
            index_of_max = i

    # 如果最远点的距离大于阈值，则在该点处切断，递归处理左右两段
    if max_dist > epsilon:
        left_part = rdp_simplify(waypoints[:index_of_max + 1], epsilon)
        right_part = rdp_simplify(waypoints[index_of_max:], epsilon)
        # 合并结果，注意去掉重复的中间点
        return left_part[:-1] + right_part
    else:
        # 如果所有点都在阈值范围内，中间的点都可以删掉，只保留首尾
        return [waypoints[0], waypoints[-1]]

# ================= 使用示例 =================
# 假设你的 A* 路径
# raw_waypoints = [[0,0,0], [1,1,0], [2,1,0], [3,2,0], [4,2,0], [5,3,0]...]
# simplified_path = rdp_simplify(raw_waypoints, epsilon=0.8)
# print(f"原路径点数: {len(raw_waypoints)}, 抽稀后: {len(simplified_path)}")
import os
import yaml
import rospkg
import rospy      

# 使用 rospkg 从 overactuated_driver/config 读 YAML
def load_4d_waypoints(yaml_path_param):
    rospack = rospkg.RosPack()
    try:
        # 找到 overactuated_driver 包的路径
        pkg_path = rospack.get_path('overactuated_driver')
        default_yaml_path = os.path.join(pkg_path, 'config', 'traj_params.yaml')
    except rospkg.ResourceNotFound:
        rospy.logerr("Rospack couldn't find the package 'overactuated_driver'!")
        return []

    yaml_path = rospy.get_param('~waypoint_file', default_yaml_path)

    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
            
        raw_waypoints = config_data.get('waypoints', [])
        
        target_waypoints_world =[]
        for p in raw_waypoints:
            # 兼容 3 维和 4 维：如果用户忘了写 pitch，默认为 0.0
            pitch = float(p[3]) if len(p) >= 4 else 0.0
            target_waypoints_world.append((float(p[0]), float(p[1]), float(p[2]), pitch))
        
        rospy.loginfo(f"Successfully loaded {len(target_waypoints_world)} 4D waypoints from {yaml_path}")
        return target_waypoints_world
    
    except Exception as e:
        rospy.logerr(f"Failed to load waypoints from {yaml_path}. Error: {e}")
        return []
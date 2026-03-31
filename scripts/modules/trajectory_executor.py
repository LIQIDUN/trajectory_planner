#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from modules.py_read_yaml import load_4d_waypoints
from modules.map_manager import MapManager
from overactuated_driver.msg import TrajectoryCommand

class TrajectoryExecutor:
    
    # 定义求导闭包函数，减少重复代码
    def eval_p(c, t): return c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
    def eval_v(c, t): return c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
    def eval_a(c, t): return 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3
    def eval_j(c, t): return 6*c[3] + 24*c[4]*t + 60*c[5]*t**2
    def eval_s(c, t): return 24*c[4] + 120*c[5]*t

        
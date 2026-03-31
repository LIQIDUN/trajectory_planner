#!/bin/bash

# --- 1. 环境准备 ---
mkdir -p ~/.gazebo
cat <<EOF > ~/.gazebo/gui.ini
[geometry]
x=100
y=100
width=1024
height=768
EOF

SOURCE_ROS="source /opt/ros/noetic/setup.bash && source ~/ros/ros_quadtilt/devel/setup.bash"
GAZEBO_MODELS="export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/liqidun/github/ardupilot_gazebo/models:/home/liqidun/github/ArduPilot_QuadTilt/Tools/autotest/models"

echo "正在清理旧进程并启动仿真..."

# --- 2. 强力清理旧环境 (防止 run_id 冲突的关键) ---
pkill -9 -f "ros"
pkill -9 -f "gazebo"
pkill -9 -f "arducopter"

# --- 3. 分窗口启动 ---

# [新增] 窗口 0：专门跑 roscore。有了它，后面的 launch 就不抢地盘了。
terminator -u --geometry=500x200+0+0 -T "ROS Master" -x bash -c "$SOURCE_ROS && roscore; exec bash" &
sleep 2 # 等待 Master 彻底启动

# 1. 启动 SITL (加了 sleep 确保 Master 已好)
terminator -u --geometry=800x400+0+250 -T "ArduPilot SITL" -x bash -c \
"cd /home/liqidun/github/ArduPilot_QuadTilt/Tools/autotest/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map; exec bash" &

sleep 2

# 2. 启动 Gazebo
terminator -u --geometry=600x300+850+0 -T "Gazebo" -x bash -c \
"$GAZEBO_MODELS && cd /home/liqidun/github/ardupilot_gazebo/worlds && gazebo --verbose quadtilt_indoor.world; exec bash" &

sleep 2

# 3. 启动 QGC
terminator -u --geometry=500x300+0+700 -T "QGC" -x bash -c \
"cd /home/liqidun/appimage && ./QGroundControl.AppImage; exec bash" &

# --- ROS 节点部分：全部增加 --wait 参数 ---

# 4. 启动 ROS 控制节点 (加上 --wait)
terminator -u --geometry=600x250+850+400 -T "ROS CTRL" -x bash -c \
"$SOURCE_ROS && roslaunch overactuated_driver run_ctrl_nodes.launch --wait; exec bash" &

# 5. 启动 ROS 轨迹规划 (加上 --wait)
terminator -u --geometry=600x250+1460+0 -T "ROS TRAJ PLANNER" -x bash -c \
"$SOURCE_ROS && roslaunch trajectory_planner traj_node.launch --wait; exec bash" &

# 6. 启动 ROS 显示 (加上 --wait)
terminator -u --geometry=600x250+1460+350 -T "ROS VISUAL" -x bash -c \
"$SOURCE_ROS && roslaunch trajectory_planner display_sitl.launch --wait; exec bash" &

# 7. 启动 RViz
RVIZ_CONFIG="/home/liqidun/ros/ros_quadtilt/src/trajectory_planner/test/cfg.rviz"

terminator -u --geometry=600x250+1460+700 -T "RViz" -x bash -c \
"$SOURCE_ROS && rviz -d $RVIZ_CONFIG; exec bash" &

echo "所有组件已启动，请检查各窗口是否有红色报错。"
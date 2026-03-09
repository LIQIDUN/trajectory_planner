#!/usr/bin/env python3
import rospy
import tf
from gazebo_msgs.msg import ModelStates

class GazeboTFPublisher:
    def __init__(self):
        rospy.init_node('gazebo_tf_broadcaster', anonymous=True)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # ！！！注意：这里填你飞机在 Gazebo 里的真实模型名称！！！
        # 常见的名字有 "quadtilt", "iris", "plane" 等
        self.model_name = "quadtilt" 
        
        # 订阅 Gazebo 的全局状态话题
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.loginfo(f"Waiting for Gazebo model '{self.model_name}' states...")

    def callback(self, msg):
        try:
            # 在所有的 Gazebo 模型中找到我们的飞机
            idx = msg.name.index(self.model_name)
            pose = msg.pose[idx]
            
            # 提取位置和姿态
            pos = (pose.position.x, pose.position.y, pose.position.z)
            ori = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            
            # 直接使用电脑当前时间发布 TF，绝不会抖动！
            self.tf_broadcaster.sendTransform(
                pos, ori, rospy.Time.now(), "base_link", "map"
            )
        except ValueError:
            # 如果还没加载出来，就忽略
            pass

if __name__ == '__main__':
    try:
        GazeboTFPublisher()
    except rospy.ROSInterruptException:
        pass
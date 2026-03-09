#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class RVizTFPublisher:
    def __init__(self):
        rospy.init_node('rviz_tf_broadcaster', anonymous=True)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 直接订阅 MAVROS 已经发布好的完美数据！
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.loginfo("Subscribed to /mavros/local_position/pose, ready to publish TF!")

    def pose_callback(self, msg):
        # 提取位姿
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        ori = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        
        # 【核心魔法】：强行使用 rospy.Time.now() 发布 TF！
        # 这会抹平 MAVROS 传过来的任何时间延迟差，保证 RViz 里的飞机绝对不会抖动！
        self.tf_broadcaster.sendTransform(pos, ori, rospy.Time.now(), "base_link", "map")

if __name__ == '__main__':
    try:
        RVizTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
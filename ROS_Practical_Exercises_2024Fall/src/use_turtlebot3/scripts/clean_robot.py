#!/usr/bin/env python  
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def clean_room(msg):
    front_distance = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
    # 如果前方障碍物过近，转向
    if front_distance < 0.5:  # 如果距离小于 0.5 米
        angular_z = 1  # 向右转
        linear_x = 0
    else:
        linear_x = 0.2  # 前进
        angular_z = 0
    twist_talker(linear_x, angular_z)

def twist_talker(linear_x, angular_z):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # 声明一个Publisher对象表示该节点正在使用String消息类型发布到chatter话题
    myTwist = Twist()
    myTwist.linear.x = linear_x
    myTwist.angular.z = angular_z 
    pub.publish(myTwist) # 发布消息
    

if __name__ == '__main__':
    rospy.init_node('clean_robot')
    rospy.Subscriber('/scan' ,LaserScan, clean_room)
    rospy.spin()
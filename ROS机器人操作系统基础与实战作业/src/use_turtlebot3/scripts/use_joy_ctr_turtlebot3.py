#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    try:
        twist_talker(data.axes[1], data.axes[0])
    except rospy.ROSInterruptException:
        pass


def twist_talker(linear_x, angular_z):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # 声明一个Publisher对象表示该节点正在使用String消息类型发布到chatter话题
    myTwist = Twist()
    myTwist.linear.x = linear_x * 2
    myTwist.angular.z = angular_z * 2
    pub.publish(myTwist) # 发布消息

def joy_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.ss
    rospy.init_node('use_joy_ctr_turtlebot', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() # 当有新的消息到达订阅的topic时，rospy.spin()会自动调用相应的回调函数

if __name__ == '__main__':
    joy_listener()
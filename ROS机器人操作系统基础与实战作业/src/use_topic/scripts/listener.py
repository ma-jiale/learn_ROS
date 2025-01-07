#!/usr/bin/env python
import rospy
from use_topic.msg import MyInfo

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", MyInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() # 当有新的消息到达订阅的topic时，rospy.spin()会自动调用相应的回调函数

if __name__ == '__main__':
    listener()
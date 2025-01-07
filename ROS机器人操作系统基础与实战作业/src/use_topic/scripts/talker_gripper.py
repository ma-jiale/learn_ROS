#!/usr/bin/env python # 确保脚本作为Python脚本执行
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('set_gripper', Int32, queue_size=10) # 声明一个Publisher对象表示该节点正在使用String消息类型发布到chatter话题
    rospy.init_node('talker', anonymous=True) # 把该节点的名称告诉了rospy
    rate = rospy.Rate(10) # 10hz 

    # 相当标准的节点运行的rospy结构
    while not rospy.is_shutdown(): # 查看is_shutdown()以检查程序是否应该退出
        # 创建自定义消息
        my_info = Int32()
        my_info.data = 1
        rospy.loginfo(my_info) # 打印消息到屏幕上；把消息写入节点的日志文件中；写入rosout
        pub.publish(my_info) # 发布消息
        rate.sleep() # 暂停程序，确保每秒循环10次

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
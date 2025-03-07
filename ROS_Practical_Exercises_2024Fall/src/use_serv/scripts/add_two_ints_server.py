#!/usr/bin/env python

from __future__ import print_function

from use_serv.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints) # 创建一个服务对象声明服务
    print("Ready to add two ints.")
    rospy.spin() # 防止代码在服务关闭之前退出，当有要求来时自动用回调函数handle_add_two_ints

if __name__ == "__main__":
    add_two_ints_server()
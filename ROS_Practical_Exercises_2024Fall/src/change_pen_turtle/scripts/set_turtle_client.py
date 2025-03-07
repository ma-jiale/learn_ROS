#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from change_pen_turtle.srv import SetPen

def set_turtle_client(r, g, b, width):
     # 对于客户端来说不需要调用init_node()
    rospy.wait_for_service('/turtle1/set_pen') # 在/turtle1/set_pen服务可用之前一直阻塞
    try:
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen) # 为服务的调用创建了一个句柄（handle）
        resp1 = set_pen(r, g, b ,width, 0)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [r g b width]"%sys.argv[0]

if __name__ == "__main__":
   
    if len(sys.argv) == 5:
        r = int(sys.argv[1])
        g = int(sys.argv[2])
        b = int(sys.argv[3])
        width = int(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    set_turtle_client(r, g, b, width)
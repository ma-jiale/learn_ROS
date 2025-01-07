#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan


def handle_laserscan_pose(msg):
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "base_scan"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.2
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    rospy.loginfo("finish sending!")
    rospy.sleep(0.1)
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_laserscan_broadcaster')
    rospy.Subscriber('/scan' ,LaserScan, handle_laserscan_pose)
    rospy.spin()
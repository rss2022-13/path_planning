#!/usr/bin/env python2

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped

def listen(msg):
    pose = msg.pose
    x = pose.position.x
    y = pose.position.y
    o = pose.orientation
    theta = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    print x, y, theta

if __name__ == "__main__":
    rospy.init_node("goalpose_sub")
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, listen, queue_size=1)
    rospy.spin()

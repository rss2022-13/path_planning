#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 1 # Tune later
        self.speed            = 1 # Tune later
        self.wrap             = 0 # Unsure of what this is for
        self.wheelbase_length = 0.5 # Measure later
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pursuit, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def find_closest_point(self, odom):
        """ Finds the closest point on the followed trajectory based on localization information
        """
        # Current position based on odometry
        pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])

    def pursuit(self, msg):
        """ Publishes drive instructions based on current PF pose information
        """


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rosy.spin()

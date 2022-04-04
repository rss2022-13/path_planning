#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        # Map traits
        self.map = None
        self.map_Transform = None
        self.map_set = False
        self.map_resolution = None

        # Goal Traits
        self.goal = None


    def map_cb(self, msg):
        '''
        Populates our map fields in the class with the map object, and sets the 
        map_set param to true so that other functions can run
        '''
        # data is indexed by (v,u) now that it is reshaped
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution

        # rot_mat is a 4x4 homo Transformation once we add the position elements
        rot_mat = quaternion_matrix([msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w])
        rot_mat[0,3] = msg.info.origin.position.x
        rot_mat[1,3] = msg.info.origin.position.y
        rot_mat[2,3] = 0
        self.map_Transform = rot_mat
        self.map_set = True

        return None


    def xy_to_uv(self, coord):
        '''
        Converts from (x,y) -> (u,v)
        '''
        if not self.map_set:
            return

        xy_homo = np.array([coord[0],coord[1],0,1]).T

        uv_homo = np.matmul(np.linalg.inv(self.map_Transform), xy_homo)

        return uv_homo[0,0]/self.map_resolution , uv_homo[1,0]/self.map_resolution

    def uv_to_xy(self, coord):
        '''
        Converts from (u,v) -> (x,y)

        [[r, r, r, x]   [[u*res]    [[x]
         [r, r, r, y] X  [v*res]  =  [y]
         [r, r, r, 0]    [0*res]     [0]
         [0, 0, 0, 1]]   [1]]        [1]]
        '''
        if not self.map_set:
            return

        uv_homo = np.array([coord[0],coord[1],0,1]).T * self.map_resolution
        uv_homo[3,0] = 1

        xy_homo = np.matmul(self.map_Transform, uv_homo)

        return xy_homo[0,0], xy_homo[1,0]


    def odom_cb(self, msg):
        pass ## REMOVE AND FILL IN ##


    def goal_cb(self, msg):
        '''
        Stores the goal location as a 4x4 Transformation matrix
        '''
        self.goal = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.goal[0,3] = msg.pose.position.x
        self.goal[1,3] = msg.pose.position.y
        self.goal[2,3] = 0

        return None

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

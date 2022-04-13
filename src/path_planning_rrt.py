#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
import datetime
from utils import LineTrajectory
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from scipy.signal import convolve2d
from Queue import PriorityQueue

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_cb, queue_size=10)

        self.map = None
        self.map_set = False
        self.map_transform = None
        self.map_resolution = None
        self.initial_pose = None
        self.initial_theta = None
        self.goal_pose = None
        self.goal_theta = None
        self.bounds = None 
        self.mask = np.ones((15,15))

        self.turning_radius = 1

    def map_cb(self, msg):
        rospy.loginfo("setting map")

        # data is indexed by (u,v) now that it is reshaped and transposed
        map_ = np.array(msg.data).reshape((msg.info.height, msg.info.width)).T
        map_[np.nonzero(map_)] = 1
        self.map = convolve2d(map_, self.mask, mode="same")
        self.map = self.map.astype(int)
        min_x, min_y = msg.info.origin.position.x, msg.info.origin.position.y
        max_x, max_y = min_x + msg.info.width, min_y + msg.info.height
        self.bounds = [min_x, min_y, max_x, max_y]

        self.map_resolution = msg.info.resolution

        rot_mat = quaternion_matrix([msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w])
        rot_mat[0,3] = msg.info.origin.position.x
        rot_mat[1,3] = msg.info.origin.position.y
        rot_mat[2,3] = 0
        self.map_transform = rot_mat
        self.map_set = True

        rospy.loginfo("Map set")

    def odom_cb(self, msg):
        # USE FOR ON ROBOT TESTING
        # self.initial_pose = msg.PoseWithCovariance.pose
        pass

    def goal_cb(self, msg):
        rospy.loginfo("GOAL POSITION RECIEVED")
        self.goal_pose = msg.pose
        if self.initial_pose and self.map_set:
            self.plan_path(self.initial_pose, self.goal_pose)
        return

    def start_cb(self, msg):
        self.initial_pose = msg.pose
        rospy.loginfo("INITIALIZATION SET")
        return

    def plan_path(self, start_pose, end_pose):
        plan = RRT(start_pose, end_pose, self.map, self.bounds, self.map_resolution, self.map_transform)
        rospy.loginfo("INITIALIZED and PLANNING PATH")
        trajectory = plan.plan()

        # publish trajectory
        self.traj_pub.publish(trajectory.toPoseArray())

        # visualize trajectory Markers
        trajectory.publish_viz()

class RRT(object):
    class Node(object):
        """Class for nodes in RRT"""
        def __init__(self, p):
            self.point = np.array(p)
            self.parent = None

    def __init__(self, start, goal, obstacles, bounds, map_resolution, map_transform, 
                 max_extend_length=10.0, path_resolution=0.5, 
                 goal_sample_rate=0.2, max_iter=10000):
        self.map_resolution = map_resolution
        self.map_transformation = map_transform
        self.start = self.Node(self.xy_to_uv([start.pose.position.x, start.pose.position.y]))
        self.goal = self.Node(self.xy_to_uv([goal.position.x, goal.position.y]))
        self.bounds = bounds
        self.max_extend_length = max_extend_length
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacles = obstacles
        self.node_list = None
        self.trajectory = LineTrajectory("/planned_trajectory")

    def xy_to_uv(self, coord):
        '''
        Converts from (x,y) -> (u,v)
        '''
        xy_homo = np.array([[coord[0],coord[1],0,1]]).T

        uv_homo = np.matmul(np.linalg.inv(self.map_transformation), xy_homo)
        # rospy.loginfo(uv_homo.shape)

        return [int(np.round(uv_homo[0,0]/self.map_resolution)) , int(np.round(uv_homo[1,0]/self.map_resolution))]
    
    def uv_to_xy(self, coord):
        '''
        Converts from (u,v) -> (x,y)
        [[r, r, r, x]   [[u*res]    [[x]
         [r, r, r, y] X  [v*res]  =  [y]
         [r, r, r, 0]    [0*res]     [0]
         [0, 0, 0, 1]]   [1]]        [1]]
        '''
        uv_homo = np.array([[coord[0],coord[1],0,1]]).T * self.map_resolution
        uv_homo[3,0] = 1

        xy_homo = np.matmul(self.map_transformation, uv_homo)

        return xy_homo[0,0], xy_homo[1,0]

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        rospy.loginfo("IN THE PLAN HELPER FUNCTION")
        self.node_list = [self.start]
        for i in range(self.max_iter):
            # Create a random node inside bounded environment
            rnd = self.get_random_node()
            # Find nearest node            
            nearest_node = self.get_nearest_node(self.node_list, rnd)
            # Get new node by connecting rnd and nearest_node
            new_node = self.steer(nearest_node, rnd, self.max_extend_length)
            # If the path between new_node and the nearest node is not in collision, add it to the node_list
            if not self.collision(new_node, nearest_node, self.obstacles):
                self.node_list.append(new_node)
    
            # If the new_node is close to the goal, connect it
            # directly to the goal and return the final path
            if self.dist_to_goal(self.node_list[-1].point) <= self.max_extend_length:
                final_node = self.steer(self.node_list[-1], self.goal, self.max_extend_length)
                if not self.collision(final_node, self.node_list[-1], self.obstacles):
                    return self.final_path(len(self.node_list) - 1)
        return None  # cannot find path

    def steer(self, from_node, to_node, max_extend_length=np.inf):
        """Connects from_node to a new_node in the direction of to_node
        with maximum distance max_extend_length
        """
        new_node = self.Node(to_node.point)
        d = from_node.point - to_node.point
        dist = np.linalg.norm(d)
        if dist > max_extend_length:
            # rescale the path to the maximum extend_length
            new_node.point  = from_node.point - d / dist * max_extend_length
        new_node.parent = from_node
        return new_node

    def dist_to_goal(self, p):
        """Distance from p to goal"""
        return np.linalg.norm(p - self.goal.point)

    def get_random_node(self):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > self.goal_sample_rate:
            # Sample random point inside boundaries
            rnd = self.Node(np.random.rand(2)*[(self.bounds[2]-self.bounds[0]), (self.bounds[3]-self.bounds[1])]+ [self.bounds[0], self.bounds[1]])
        else:  
            # Select goal point
            rnd = self.Node(self.goal.point)
        return rnd
    
    def get_nearest_node(self, node_list, node):
        """Find the nearest node in node_list to node"""
        dlist = [np.sum(np.square((node.point - n.point))) for n in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]
    
    def collision(self, node1, node2, obstacles):
        """Check whether the path connecting node1 and node2 
        is in collision with anyting from the obstacle_list
        """
        p1 = node2.point
        p2 = node1.point

        # Get the eqn of a line from p1 to p2
        slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = np.full((3,1), p2[1] - slope*p2[0])

        # Divide interval from x2 to x1 into 100 pieces
        interval = np.linspace(p1[0], p2[0], 3).reshape((1,3)).T 

        # Check if any of these pieces return non-zero values when multiplied by the line eqn
        y = slope*interval + b
        xy = np.append(interval, y, axis=1)

        for c in xy:
            if self.obstacles[int(c[0]), int(c[1])] != 0:
                return True
        return False
    
    def final_path(self, goal_ind):
        """Compute the final path from the goal node to the start node
        Formats path as LineTrajectory to be returned
        """
        s = Point()
        node = self.node_list[goal_ind]
        s.x, s.y = self.uv_to_xy((node.point[0], node.point[1]))
        path = [s]
        rospy.loginfo("ENTERED FINAL PATH")

        while node.parent:
            node = node.parent
            s2 = Point()
            s2.x, s2.y = self.uv_to_xy((node.point[0], node.point[1])) 
            path.append(s2)

        for e in reversed(path):
            self.trajectory.addPoint(e)

        return self.trajectory

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

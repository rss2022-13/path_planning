#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from scipy.signal import convolve2d
from Queue import PriorityQueue
# import rospkg
import time, os
from utils import LineTrajectory
import datetime

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
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_cb, queue_size=10)
        self.clicked_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        # Map traits
        self.map = None
        self.map_Transform = None
        self.map_set = False
        self.map_resolution = None

        # For eroding the map with scipy
        self.mask = np.ones((15,15))
        self.stride = 1

        # odom traits
        self.turning_radius = 1

        # Start Traits
        self.start = None
        self.start__theta = None

        # clicked traits
        self.clicked = None
        
        # Goal Traits
        self.goal = None
        self.goal_theta = None

        #publish trajectory
        # self.plan_path(self.start, self.goal, self.map)


    def map_cb(self, msg):
        '''
        Populates our map fields in the class with the map object, and sets the 
        map_set param to true so that other functions can run
        '''
        #rospy.loginfo("setting map")
        # data is indexed by (u,v) now that it is reshaped and transposed
        map = np.array(msg.data).reshape((msg.info.height, msg.info.width)).T

        # make all nonzero values 1 (including -1 vals)
        map[np.nonzero(map)] = 1

        # Now the map is dilated(sorta) by a square of size 7
        self.map = convolve2d(map, self.mask, mode="same")
        self.map = self.map.astype(int)

        # NOTE: Can uncomment the stride portion to downsample the map in order to increase
        # speed. Will reduce accuracy though
        self.map = self.map[0:self.map.shape[0]:self.stride, 0:self.map.shape[1]:self.stride]

        self.map_resolution = msg.info.resolution

        # rot_mat is a 4x4 homo Transformation once we add the position elements
        self.map_Transform = quaternion_matrix([msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w])
        self.map_Transform[0,3] = msg.info.origin.position.x
        self.map_Transform[1,3] = msg.info.origin.position.y
        self.map_Transform[2,3] = 0
        self.map_set = True

        return None


    def xy_to_uv(self, coord):
        '''
        Converts from (x,y) -> (u,v)
        '''
        if not self.map_set:
            #rospy.loginfo("Map not set yet")
            return

        xy_homo = np.array([[coord[0],coord[1],0,1]]).T

        uv_homo = np.matmul(np.linalg.inv(self.map_Transform), xy_homo)
        # rospy.loginfo(uv_homo.shape)

        return int(np.round(uv_homo[0,0]/self.map_resolution)) , int(np.round(uv_homo[1,0]/self.map_resolution))

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

        uv_homo = np.array([[coord[0],coord[1],0,1]]).T * self.map_resolution
        uv_homo[3,0] = 1

        xy_homo = np.matmul(self.map_Transform, uv_homo)

        return xy_homo[0,0], xy_homo[1,0]


    def odom_cb(self, msg):
        if self.start is None:
            self.start = msg.pose


    def goal_cb(self, msg):
        '''
        Stores the goal location as a 4x4 Transformation matrix
        '''
        # self.goal = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # self.goal[0,3] = msg.pose.position.x
        # self.goal[1,3] = msg.pose.position.y
        # self.goal[2,3] = 0
        # self.goal_theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.goal = int(np.round(msg.pose.position.x)), int(np.round(msg.pose.position.y))
        if self.start is not None and self.map_set:
            self.trajectory.clear()
            self.plan_path()

        return None

    def start_cb(self, msg):
        '''
        Stores the start location as a 4x4 Transformation matrix
        '''
        # self.start = quaternion_matrix([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # self.start[0,3] = msg.pose.pose.position.x
        # self.start[1,3] = msg.pose.pose.position.y
        # self.start[2,3] = 0
        # self.start_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.start = int(np.round(msg.pose.pose.position.x)), int(np.round(msg.pose.pose.position.y))
        if self.goal is not None and self.map_set:
            self.trajectory.clear()
            self.plan_path()

        return None

    def clicked_cb(self, msg):
        '''
        Stores the clicked location as a 4x4 Transformation matrix
        '''
        #self.clicked = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        #self.clicked[0,3] = msg.point.position.x
        #self.clicked[1,3] = msg.point.position.y
        #self.clicked[2,3] = 0
        self.clicked = int(np.round(msg.point.x)), int(np.round(msg.point.y)), int(np.round(msg.point.z))

        return None

    def heuristic(self, node1, node2):
        # Dubins curve heuristic (used in plan_path)
        # start = node1+[0] #TODO: Change [0] to an angle that matches the path better
        # end = node2+[self.goal_theta]
        # return dubins.path_length(dubins.shortest_path(start, end, self.turning_radius))

        # Euclidian Dist
        return ((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)**.5
        
    def plan_path(self):
        ## CODE FOR PATH PLANNING ##
        print "PLANNING PATH!"
        start_time = time.time()
        # A* search
        #rospy.loginfo("Planning Path")
        frontier = PriorityQueue()
        seen = set()
        start_u, start_v = self.xy_to_uv(self.start)
        start_u = int(start_u/self.stride)
        start_v = int(start_v/self.stride)
        end_u, end_v = self.xy_to_uv(self.goal)
        end_u = int(end_u/self.stride)
        end_v = int(end_v/self.stride)
        frontier.put((0,0,(start_u, start_v)))

        came_from = dict()
        cost_so_far = dict()

        # came_from[(start_u, start_v)] = None
        cost_so_far[(start_u, start_v)] = 0

        while not frontier.empty():
            current = frontier.get()[2]

            seen.add(current)

            if current == (end_u, end_v):
                break

            u, v = current[0], current[1]

            neighbors = {(u+1, v), (u-1, v), (u, v+1), (u, v-1), (u-1,v-1), (u-1, v+1), (u+1, v-1), (u+1, v+1)}
            for next in neighbors:
                # ignore if seen
                if next in seen:
                    continue
                # Ignore this path if obstacle
                try:
                    if self.map[next[0], next[1]] != 0:
                        continue
                except(e):
                    # if out of bounds
                    #print "error: ", e
                    continue
                
                new_cost = cost_so_far[(u,v)] + self.heuristic((u,v), next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next,(end_u/self.stride, end_v/self.stride))
                    frontier.put((priority, datetime.datetime.now(), next))
                    came_from[next] = (u,v)
                    
        # set up trajectory based on A* search results
        s = Point()
        curr = (end_u, end_v)
        s.x, s.y = self.goal[0], self.goal[1]
        pts = [s]
        while curr in came_from.keys():
            next = came_from[curr]
            s2 = Point()
            s2.x, s2.y = self.uv_to_xy((next[0]*self.stride, next[1]*self.stride))
            pts.append(s2)
            curr = next
        
        for e in reversed(pts):
            self.trajectory.addPoint(e)

        print "Time:", time.time() - start_time
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

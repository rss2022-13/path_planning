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
        """ Finds the closest point on the followed trajectory based on localization information.
            Inputs
            --------
            odom: Odometry msg, describes vehicle's position in the world

            Outputs
            --------
            closest: Tuple of the closest point, closest distance, and the index value of the trajectory 
                    segment (starting from 0)
        """
        # Current position based on odometry
        cur_pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])

        # Trajectory points
        traj_pts = np.array(self.trajectory.points)
        
        # Relative Trajectory Segment Vectors
        traj_deltas = np.diff(traj_pts, axis=0)
        traj_norms = np.linalg.norm(traj_deltas, axis=1)

        # Relative Vectors from Current Pose to Trajectory Points
        traj2pos_deltas = np.subtract(cur_pos, traj_pts[:-1])

        # Finding closest point for each segment
            
        dot_product = np.sum(np.multiply(traj2pos_deltas, traj_deltas), axis=1)
            # Scale it to the norms of the trajectory segments
        scaled_dot_product = np.divide(dot_product, traj_norms)
            # Cap the result to either a 0 or a 1, to stay within segment boundaries
        capped = np.maximum(np.minimum(scaled_dot_product, 1), 0)
            # Multiply this value to the segment deltas and add it to the traj_pts to get closest points
            # on each segment
        closest_pts_list = np.add(traj_pts[:-1], np.multiply(traj_deltas, capped.reshape(len(capped), 1)))
        closest_dist_list = np.linalg.norm(np.subtract(cur_pos, closest_pts), axis=1)
        # index of the actual closesst point
        c_ind = np.argmin(closest_dist_list)

        # Return a tuple of the closest distance, closest point, and the index value
        closest = (closest_pts_list[c_ind][:], closest_dist_list[c_ind], c_ind)
        return closest

    def find_goal_point(self, odom)
        """ Find the goal point on the current trajectory
        """
        
        cur_pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        closest = self.find_closest_point(odom)

        traj_pts = np.array(self.trajectory.points)

        closest_pt = closest[0]
        closest_dist = closest[1]
        closest_ind = closest[2]


        if (closest_dist >= self.lookahead):
            return closest_pt
        else:
            # Iterator for goal point search
            i = 1

            while i+closest_ind < len(traj_pts):
                # Trajectory Segment (starting at the closest point)
                if i == 1:
                    start_pt = closest_pt
                else:
                    start_pt = traj_pts[closest_ind+i-1]
                    
                end_pt = traj_pts[closest_ind+i]
                seg_delta = np.subtract(end_pt, start_pt)

                robot_to_start = np.subtract(start_pt, cur_pos)
                # Guaranteed intersection with lookahead circle if the closest distance is 
                # less than lookahead distance
                a = np.dot(seg_delta,seg_delta)
                b = 2*np.dot(seg_delta, robot_to_start)
                c = np.dot(robot_to_start, robot_to_start) - self.lookahead**2

                # Since we are starting at the closest point
                # and we are assuming it is within the circle
                # the correct t is positive while the other is negative
                t1 = (-b+np.sqrt(b**2 - 4*a*c))/(2*a)
                t2 = (-b-np.sqrt(b**2 - 4*a*c))/(2*a)
            
                t = max(t1, t2)
                if t < 0:
                    print "Something went very wrong"
                elif t < 1:
                    return np.add(start_pt, np.multiply(seg_delta, t))
                else:
                    i += 1
            return None
            
            

    def pursuit(self, msg):
        """ Publishes drive instructions based on current PF pose information
        """
        

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rosy.spin()

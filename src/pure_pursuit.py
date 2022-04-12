#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = "/odom" #rospy.get_param("~odom_topic")
        self.lookahead        = 2 # Tune later
        self.speed            = 1 # Tune later
        self.wheelbase_length = 0.325 # From model robot, change later
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pursuit, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print ("Receiving new trajectory:" + str(len(msg.poses)) + "points")
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
        if len(self.trajectory.points) == 0:
            return None
        # Current position based on odometry
        cur_pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])

        # Trajectory points
        traj_pts = np.array(self.trajectory.points)

        # Relative Trajectory Segment Vectors
        traj_deltas = np.diff(traj_pts, axis=0)
        traj_norm_squared = np.sum(np.multiply(traj_deltas, traj_deltas), axis=1)

        # Relative Vectors from Current Pose to Trajectory Points
        traj2pos_deltas = np.subtract(cur_pos, traj_pts[:-1])

        # Finding closest point for each segment
        #rospy.loginfo("Traj 2 Pos: [%.2f, %.2f]", traj2pos_deltas[0][0], traj2pos_deltas[0][1])     
        #rospy.loginfo("Traj Delta: [%.2f, %.2f]", traj_deltas[10][0], traj_deltas[10][1])     

        dot_product = np.sum(np.multiply(traj2pos_deltas, traj_deltas), axis=1)
        #rospy.loginfo("Dot Product: %.2f", dot_product[0])     

            # Scale it to the norms of the trajectory segments
        scaled_dot_product = np.divide(dot_product, traj_norm_squared)
        #rospy.loginfo("Traj Norm: %.2f", traj_norms[10])     
            # Cap the result to either a 0 or a 1, to stay within segment boundaries
        capped = np.maximum(np.minimum(scaled_dot_product, 1), 0)
            # Multiply this value to the segment deltas and add it to the traj_pts to get closest points
            # on each segment
        closest_pts_list = np.add(traj_pts[:-1], np.multiply(traj_deltas, capped.reshape(len(capped), 1)))
        difference = np.subtract(cur_pos, closest_pts_list)
        closest_dist_list = np.sqrt(np.sum(np.multiply(difference,difference), axis=1))
        # index of the actual closesst point
        if len(closest_dist_list) > 0:
            c_ind = closest_dist_list.argmin()
        else:
            rospy.loginfo("Something went wrong again, the list is empty")
            rospy.loginfo(len(closest_pts_list))
            rospy.loginfo(len(traj_deltas))
            return None

        #rospy.loginfo("Closest Point: (%f.2, %f.2)", closest_pts_list[c_ind][0], closest_pts_list[c_ind][1])
        #rospy.loginfo("Closest Distance: %f.2", closest_dist_list[c_ind])
        #rospy.loginfo("Current Position: (%.2f, %.2f)", cur_pos[0], cur_pos[1])

        # Return a tuple of the closest point, and the index value
        closest = (closest_pts_list[c_ind], c_ind, closest_dist_list[c_ind])
        return closest

    def find_goal_point(self, odom):
        """ Find the goal point on the current trajectory
        """
        cur_pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        closest = self.find_closest_point(odom)
        if closest is None:
            return None

        traj_pts = np.array(self.trajectory.points)

        closest_pt = closest[0]
        closest_ind = closest[1]
        closest_dist = closest[2]

        if closest_dist > self.lookahead:
            return (closest_pt, False)
        else:
            # Find the first point that is farther than the lookahead distance
            first_out = closest_ind
            for i in range(closest_ind+1, len(traj_pts)):
                pt = traj_pts[i]
                squared_dist = np.dot(cur_pos-pt, cur_pos-pt)
                #rospy.loginfo("Point: [%.2f, %.2f]", pt[0], pt[1])
                #rospy.loginfo("Squared Dist: %.2f", squared_dist)
                if (squared_dist > self.lookahead**2):
                    first_out = i
                    break
            out_pt = traj_pts[first_out][:]
            in_pt = traj_pts[first_out-1][:]
            
            vec_v = out_pt - in_pt;
            vec_w = in_pt - cur_pos;
            
            c = np.dot(vec_w, vec_w) - self.lookahead**2
            b = 2*np.dot(vec_v, vec_w)
            a = np.dot(vec_v, vec_v)
            
            t = 0
            if (b**2 - 4*a*c) > 0:
                t1 = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)
                t2 = (-b - np.sqrt(b**2 - 4*a*c))/(2*a)
                
                t = max(t1, t2)
                    
                #rospy.loginfo("This is the expected case")
                #rospy.loginfo("t1 = %.2f, t2 = %.2f", t1, t2)

                return (in_pt + t*vec_v, True)

            elif abs(b**2 - 4*a*c) < 1e-6 and b < 0:
                t = -b/(2*a)
                #rospy.loginfo("This is not expected, but should still work")
                #rospy.loginfo("t = %.2f", t)
                return (in_pt + t*vec_v, True)

            else:
                rospy.loginfo("wtf, negative b^2-4*a*c")
                return None
            
            

    def pursuit(self, msg):
        """ Publishes drive instructions based on current PF pose information
        """

        # Find the goal point
        goal = self.find_goal_point(msg)
        """
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = msg.header.frame_id
        ack_msg.drive.steering_angle = 0
        ack_msg.drive.speed = 0
        self.drive_pub.publish(ack_msg)
        """
        if goal is not None:
            goal_point = goal[0]
            goal_point = np.append(goal_point, [0])
            
            # Find current position and orientation
            cur_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0])
            #cur_theta = np.arccos(msg.pose.pose.orientation.w)*2
            cur_theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
            
            # Determine the perpendicular distance to the goal point from the car position
            orientation_vec = np.array([np.cos(cur_theta), np.sin(cur_theta), 0])
            diff_vec = np.subtract(goal_point, cur_pos)
            cross = np.cross(orientation_vec, diff_vec)
            
            rospy.loginfo("Orientation: %.2f, %.2f", orientation_vec[0], orientation_vec[1])
            rospy.loginfo("angle: %.2f", cur_theta)
            rospy.loginfo("Goal Point: %.2f, %.2f", goal[0][0], goal[0][1])
            rospy.loginfo("Current Position: %.2f, %.2f", cur_pos[0], cur_pos[1])
            rospy.loginfo("Diff_vec: [%.2f, %.2f]", diff_vec[0], diff_vec[1])
            perp_dist = cross[2] # Allowed to be negative to distinguish between turing left or right
            rospy.loginfo("Cross: [%.2f, %.2f, %.2f]", cross[0], cross[1], cross[2])
            #rospy.loginfo("Perpendicular Distance: %.2f", perp_dist)
       
            
            if goal[1]:
                arc_curvature = (2*perp_dist)/(self.lookahead**2)
                #rospy.loginfo("Curvature: %.2f", arc_curvature)
                steering_angle = np.arctan(self.wheelbase_length*arc_curvature) # Getting steering angle from curvature
            else:
                if abs(perp_dist) > 1e-3:
                    rospy.loginfo("Edge Case")
                    steering_angle = np.sign(perp_dist)*1
                else:
                    steering_angle = 0
            rospy.loginfo("Steering: %.2f", steering_angle)
            rospy.loginfo("----------------------------")
            
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.header.frame_id = msg.header.frame_id
            ack_msg.drive.steering_angle = steering_angle
            ack_msg.drive.speed = self.speed

            self.drive_pub.publish(ack_msg)
            #rospy.loginfo("Is this working")
        else:
            # rospy.loginfo("Not working, empty trajectory")
            pass
        


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()

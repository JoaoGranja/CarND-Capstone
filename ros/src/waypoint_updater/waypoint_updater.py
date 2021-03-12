#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import cKDTree
from std_msgs.msg import Int32

import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2D  = None
        self.tree = None
        self.pose = None
        self.traffic_idx = None
        
        self.main_loop()
        
    def main_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if None not in (self.pose, self.tree, self.waypoints_2D ):
                rospy.logdebug("Publish final waypoints")
                self.publish_final_waypoints()
            rate.sleep()
            
    def publish_final_waypoints(self):
        closest_idx = self.get_closest_index()
        lane = Lane()
        lane.header = self.base_waypoints.header
        #if (self.traffic_idx is not None) and (self.traffic_idx > closest_idx and self.traffic_idx < closest_idx + LOOKAHEAD_WPS):
        #    lane.waypoints = self.process_traffic_waypoint(closest_idx)
        #else:
        #    lane.waypoints = self.base_waypoints.waypoints[closest_idx : closest_idx + LOOKAHEAD_WPS]
        lane.waypoints = self.base_waypoints.waypoints[closest_idx : closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        
    def process_traffic_waypoint(self, closest_idx):
        # This function will change the velocity of the waypoints which are between closest_idx and traffic_idx
        current_base_list = self.base_waypoints.waypoints[closest_idx : self.traffic_idx]
        wp_list = current_base_list.copy()
        
        dist_traffic_line =  distance(current_base_list, closest_idx, self.traffic_idx - 2)
        current_wp_len = closest_idx - self.traffic_idx + 1
        for idx, wp in enumerate(current_base_list):
            # The ideia is to decelerate linearly until reach 0 on waypoint and 2 positions before
            wp_vel = get_waypoint_velocity(wp)
            
            calc_vel = ((current_wp_len - idx) / current_wp_len )* wp_vel 
            rospy.logdebug("WP index is {0} Calculated velocity is {1}, waypoint velocity is {2}", idx, calc_vel, wp_vel)
            if calc_vel < 0.1:
                calc_vel = 0
            update_vel = min(wp_vel, calc_vel)
            set_waypoint_velocity(wp_list, idx, update_vel)
        
        return wp_list.append(self.base_waypoints.waypoints[self.traffic_idx : closest_index + LOOKAHEAD_WPS])
            
    def pose_cb(self, msg):
        self.pose = msg
        rospy.logdebug("Car position callback")
        
    def get_closest_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        #find the closest waypoint of the current car position
        closest_idx = self.tree.query([x,y])[1]
        
        #Check if this point is behind or after the current car position
        closest_wp = np.array(self.waypoints_2D[closest_idx])
        if closest_idx == 0:
            ahead_closest_wp = np.array(self.waypoints_2D[closest_idx+1])
            dot = np.dot((ahead_closest_wp - closest_wp), (closest_wp - np.array([x,y])))
        else:
            behind_closest_wp = np.array(self.waypoints_2D[closest_idx-1])
            dot = np.dot((closest_wp - behind_closest_wp), (closest_wp - np.array([x,y])))
            
        if dot > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2D)
            
        return closest_idx

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        self.waypoints_2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.tree = cKDTree(self.waypoints_2D)
        rospy.logdebug("Base waypoints callback")
        
    def traffic_cb(self, msg):
        rospy.logdebug("Traffic waypoint {0}", msg)
        self.traffic_idx = msg

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

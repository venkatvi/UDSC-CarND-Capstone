#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
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
MAX_DECL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # Enable once the dbw node is fixed
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.pose = None
        self.stopline_wp_idx = -1

        self.run()
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints: 
                self.publish_waypoints()
                
            rate.sleep()
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y 
        closest_idx = self.waypoint_tree.query([x,y], 1)[1] # the first element measures the distance 

        # check if closest idx is ahead or behind vehicle 
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect - cl_vect)
        if val > 0 :
            closest_idx = (closest_idx + 1) %len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
    def generate_lane(self):
        lane = Lane()
        closest_waypoint_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_waypoint_idx + LOOKAHEAD_WPS 
        base_waypoints        = self.base_waypoints.waypoints[closest_waypoint_idx:farthest_idx]

        self.stopline_wp_idx = -1 
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_waypoint_idx)
        lane.header = self.base_waypoints.header
        return lane
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_idx = max(self.stopline_wp_idx - closest_idx - 10, 0) # 2 waypoints back from the line 
                        
        for idx, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            # Get total distance between consecutive waypoints from idx to stop_idx
            dist = self.distance(waypoints, idx, stop_idx)
            
            velocity = math.sqrt(2 * MAX_DECL * dist)
            if velocity < 1.0:
                velocity = 0.0
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            temp.append(p)
        return temp
    def pose_cb(self, msg):
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        pass

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data
        rospy.loginfo('traffic_cb: {}'.format(self.stopline_wp_idx))
        pass
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
        if wp2 > len(waypoints):
            wp2 = len(waypoints)
        
        # Compute distance between waypoint i,j for i=1...n-1, j = 2...n where wp1 = 1, wp2 = n 
        # Sum up all distances 
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
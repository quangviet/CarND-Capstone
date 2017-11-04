#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
last_closest_wp = -1
ONE_MPH = 0.44704
MAX_SPEED = 40*ONE_MPH # mph

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.light_wp = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        # Get closest waypoint
        closest_wp = self.get_closest_waypoint(self.pose)
        #rospy.logerr('closest_wp: %d :: light_wp: %d', closest_wp, self.light_wp)
        if closest_wp > -1:
            # publish LOOKAHEAD_WPS waypoints
            message = Lane()
            for i in range(LOOKAHEAD_WPS):
                # Get waypoints from base_waypoints
                waypoint = self.waypoints[closest_wp+i]
                # Update linear velocity base on light_wp
                wp_speed = 0.0
                if self.light_wp > -1 and self.light_wp < closest_wp+LOOKAHEAD_WPS/2:
                    wp_speed = MAX_SPEED*(self.light_wp-(closest_wp+i))/100
                else:
                    wp_speed = MAX_SPEED
                waypoint.twist.twist.linear.x = wp_speed
                message.waypoints.append(waypoint)
                if wp_speed == 0.0:
                    break # break for loop
            self.final_waypoints_pub.publish(message)

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_wp = msg.data
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
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance2(self, pos1, pos2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(pos1.pose.position, pos2.pose.position)

    def angular(self, next_pos, current_pos):
        theta = math.atan2(next_pos.pose.position.y - current_pos.pose.position.y,
                            next_pos.pose.position.x - current_pos.pose.position.x)
        theta -= current_pos.pose.orientation.z
        return theta

    def get_closest_waypoint(self, pose):
        global last_closest_wp
        closest_len = 100 #large number
        closest_wp = -1
        if (self.waypoints):
            for i in range(len(self.waypoints)):
                if last_closest_wp > -1: # Already have last closest waypoint
                    if i < last_closest_wp or i > last_closest_wp+LOOKAHEAD_WPS: # Just find in LOOKAHEAD_WPS
                        continue
                dist = self.distance2(self.waypoints[i].pose, pose)
                if (dist < closest_len):
                    theta = self.angular(self.waypoints[i].pose, pose)
                    if theta < math.pi/4.0:
                        closest_len = dist
                        closest_wp = i
        last_closest_wp = closest_wp
        return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

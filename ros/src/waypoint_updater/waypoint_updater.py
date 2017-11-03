#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
last_closest_waypoint = -1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pos = None
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pos = msg
        # Get closest waypoint
        if self.base_waypoints is not None:
            closest_waypoint = self.get_closest_waypoint(self.base_waypoints, self.current_pos)
            #rospy.logerr('closest_waypoint: %d', closest_waypoint)
            if closest_waypoint > -1:
                # publish LOOKAHEAD_WPS waypoints
                message = Lane()
                for i in range(LOOKAHEAD_WPS):
                    waypoint = self.base_waypoints[closest_waypoint+i]
                    message.waypoints.append(waypoint)
                self.final_waypoints_pub.publish(message)

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.base_waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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

    def get_closest_waypoint(self, waypoints, pos):
        global last_closest_waypoint
        closestLen = 100 #large number
        closestWaypoint = -1
        for i in range(len(waypoints)):
            if last_closest_waypoint > -1: # Already have last closest waypoint
                if i < last_closest_waypoint or i > last_closest_waypoint+LOOKAHEAD_WPS: # Just find in LOOKAHEAD_WPS
                    continue
            dist = self.distance2(waypoints[i].pose, pos)
            if (dist < closestLen):
                theta = self.angular(waypoints[i].pose, pos)
                if theta < math.pi/4.0:
                    closestLen = dist
                    closestWaypoint = i
        last_closest_waypoint = closestWaypoint
        return closestWaypoint


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

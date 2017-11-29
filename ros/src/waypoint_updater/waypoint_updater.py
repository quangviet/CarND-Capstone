#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy

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
MAX_SPEED = 40*1000/60/60 # mps
last_closest_wp = -1
MAX_DECEL = 1.0

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
        car_position = self.get_closest_waypoint(self.pose)

        if car_position > -1:
            # publish LOOKAHEAD_WPS waypoints
            message = Lane()
            len_wps = len(self.waypoints)
            for i in range(LOOKAHEAD_WPS):
                wp_id = car_position+i
                if self.light_wp > -1 and wp_id > self.light_wp-3:
                    break # Stop car in front of the traffic light 3 wps
                if wp_id >= len_wps:
                    break # Stop car when cross the end of road
                waypoint = self.waypoints[wp_id]
                message.waypoints.append(waypoint)

            # Update velocity for wp
            if len(message.waypoints) > 0:
                if self.light_wp > -1:
                    message.waypoints = self.decelerate(message.waypoints)
                else:
                    message.waypoints = self.accelerate(message.waypoints)

            self.final_waypoints_pub.publish(message)

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def angular(self, next_pos, current_pos):
        theta = math.atan2(next_pos.pose.position.y - current_pos.pose.position.y,
                            next_pos.pose.position.x - current_pos.pose.position.x)
        sin_yaw = 2*current_pos.pose.orientation.w*current_pos.pose.orientation.z
        cos_yaw = 1.0 - 2*current_pos.pose.orientation.z**2
        yaw = math.atan2(sin_yaw, cos_yaw)
        theta -= yaw

        if theta > math.pi:
            theta -= 2*math.pi
        if theta < -math.pi:
            theta += 2*math.pi

        return theta

    def get_closest_waypoint(self, pose):
        global last_closest_wp
        closest_len = 100 #large number
        closest_wp = -1
        if (self.waypoints):
            for i in range(len(self.waypoints)):
                if last_closest_wp > -1:
                    '''
                    Already have last closest waypoint
                    Just find ahead in LOOKAHEAD_WPS waypoints
                    Ignore other waypoints
                    '''
                    if i < last_closest_wp or i > last_closest_wp+LOOKAHEAD_WPS:
                        continue
                dist = self.distance(self.waypoints[i].pose.pose.position, pose.pose.position)
                if (dist < closest_len):
                    theta = self.angular(self.waypoints[i].pose, pose)
                    if abs(theta) < math.pi/4.0:
                        closest_len = dist
                        closest_wp = i
        last_closest_wp = closest_wp
        return closest_wp

    def accelerate(self, waypoints):
        first = waypoints[0]
        if first.twist.twist.linear.x < 1.0:
            first.twist.twist.linear.x = 1.0
        vel = first.twist.twist.linear.x
        for wp in waypoints[1:]:
            dist = self.distance(wp.pose.pose.position, first.pose.pose.position)
            vel += math.sqrt(2 * MAX_DECEL * dist)
            if vel > MAX_SPEED:
                vel = MAX_SPEED
                break # Don't need to increase velocity after get MAX_SPEED
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

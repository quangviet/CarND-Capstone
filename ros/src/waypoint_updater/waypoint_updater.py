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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	#rospy.Subscriber('/traffic_waypoint',

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	self.yaw = 0
	self.x = 0
	self.y = 0
	self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
	#rospy.logerr("Pose: " + str(msg))
	siny = 2 * msg.pose.orientation.w * msg.pose.orientation.z
	cosy = 1 - 2 * msg.pose.orientation.z * msg.pose.orientation.z
	yaw = math.atan2(siny, cosy);
	#rospy.logerr("Yaw: " + str(yaw))
	self.yaw = yaw
	self.x = msg.pose.position.x
	self.y = msg.pose.position.y

	if self.waypoints is None:
	    return

	min_distance = 1e+10
	nearest_waypoint = 0

	for i in range(len(self.waypoints)):
	    x = self.waypoints[i].pose.pose.position.x
	    y = self.waypoints[i].pose.pose.position.y
	    d = (x-self.x)**2 + (y-self.y)**2
	    if d < min_distance:
		min_distance = d
		nearest_waypoint = i

	#for i in range(10):
	#    rospy.logerr(str(i-5) + " "  + str(self.waypoints[nearest_waypoint+i-5].pose.pose.position))

	#rospy.logerr("x = " + str(self.x) + " y = " + str(self.y) + " wp = " + str(self.waypoints[nearest_wayp].pose.pose.position))

	theta = math.atan2(self.waypoints[nearest_waypoint].pose.pose.position.y - self.y, self.waypoints[nearest_waypoint].pose.pose.position.x - self.x)
	if abs(theta - yaw) > (math.pi/2):
	    nearest_waypoint = (nearest_waypoint + 1) % len(self.waypoints)

	wp = nearest_waypoint
	final_waypoints = []
	for i in range(LOOKAHEAD_WPS):
	    final_waypoints.append(self.waypoints[wp])
	    wp = (wp + 1) % len(self.waypoints)

	self.publish(final_waypoints)
	#rospy.logerr("Publish " + str(len(final_waypoints)) + " waypoints with first wp is " + str(nearest_waypoint))

    def waypoints_cb(self, waypoints):
	self.waypoints = waypoints.waypoints

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

    def publish(self, final_waypoints):
	lane = Lane()
	lane.header.frame_id = '/final_waypoints'
	lane.header.stamp = rospy.Time(0)
	lane.waypoints = final_waypoints
	self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

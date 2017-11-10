#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import time

DEBUG_MODE = True

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200 # wps
DANGEROUS_ZONE = 40 # m
last_closest_wp = -1
stop_line_ways = []

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier('light_classification/model.h5')
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        '''
        10902
        '''
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        '''
        318 : 784 : 2095 : 2625 : 6322 : 7036 : 8565 : 8565
        '''
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # Collect data for training
        '''
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        cv2.namedWindow('cv_image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('cv_image', 320, 240)
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)

        time_str = time.strftime("%Y%m%d-%H%M%S")
        output_name = "light_classification/training_data/cv_image_" + time_str + ".png"
        if light_wp == -1:
            output_name = "light_classification/training_data/unknown/cv_image_" + time_str + ".png"
        cv2.imwrite(output_name, cv_image)
        '''

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            #light_wp = light_wp if state == TrafficLight.RED else -1
            if state != TrafficLight.RED:
                light_wp = -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

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
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
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
                dist = self.distance2(self.waypoints[i].pose, pose)
                if (dist < closest_len):
                    theta = self.angular(self.waypoints[i].pose, pose)
                    if abs(theta) < math.pi/4.0:
                        closest_len = dist
                        closest_wp = i
        last_closest_wp = closest_wp
        return closest_wp

    def get_closest_light_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_len = 100 #large number
        closest_wp = -1
        if (self.waypoints):
            for i in range(len(self.waypoints)):
                dist = self.distance2(self.waypoints[i].pose, pose)
                if (dist < closest_len):
                    closest_len = dist
                    closest_wp = i
        return closest_wp

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        global stop_line_ways
        light_wp = -1
        car_position = -1

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose)

        #TODO find the closest visible traffic light (if one exists)
        if car_position > -1:
            # Make stop_line_waypoint list
            if len(stop_line_ways) == 0: # Just need update 1 time
                # List of positions that correspond to the line to stop in front of for a given intersection
                '''
                Stop Line Way 292 : 753 : 2047 : 2580 : 6294 : 7008 : 8540 : 9733
                Traffic Light 318 : 784 : 2095 : 2625 : 6322 : 7036 : 8565 : 8565
                '''
                stop_line_positions = self.config['stop_line_positions']
                for stop_line_position in stop_line_positions:
                    pose = PoseStamped()
                    pose.pose.position.x = stop_line_position[0];
                    pose.pose.position.y = stop_line_position[1];
                    stop_line_way = self.get_closest_light_waypoint(pose)
                    stop_line_ways.append(stop_line_way)

            # Find dist_stop_line
            dist_stop_line = DANGEROUS_ZONE
            for stop_line_way in stop_line_ways:
                if stop_line_way > car_position:
                    dist = self.distance(self.waypoints, car_position, stop_line_way)
                    if dist < dist_stop_line:
                        dist_stop_line = dist
                        light_wp = stop_line_way

            '''
            if light_wp > -1:
                if dist_stop_line > DANGEROUS_ZONE-4:
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                    time_str = time.strftime("%Y%m%d-%H%M%S")
                    output_name = "light_classification/training_data/cv_image_" + time_str + ".png"
                    cv2.imwrite(output_name, cv_image)
            '''

        if light_wp > -1:
            predict_state = TrafficLight.UNKNOWN
            #predict_state = self.get_light_state()
            debug_state = TrafficLight.UNKNOWN
            if DEBUG_MODE == True:
                # Find dist_vis_light
                # 20 is distance from stop_line to vis_light
                dist_vis_light = DANGEROUS_ZONE+20
                for light in self.lights:
                    vis_light_way = self.get_closest_light_waypoint(light.pose)
                    if vis_light_way > car_position:
                        dist = self.distance(self.waypoints, car_position, vis_light_way)
                        if dist < dist_vis_light:
                            dist_vis_light = dist
                            debug_state = light.state
                state = debug_state
            else:
                state = predict_state
            rospy.logerr('car_wp: %d :: light_wp: %d :: dist_stop_line: %f :: debug_state: %d :: predict_state: %d', car_position, light_wp, dist_stop_line, debug_state, predict_state)
            return light_wp, TrafficLight.RED
        else:
            rospy.logerr('car_wp: %d', car_position)

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

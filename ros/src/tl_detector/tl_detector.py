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
from scipy.spatial import cKDTree
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.tree = None
        self.waypoints_2D = None
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
        self.use_classifier = rospy.get_param('~use_classifier')
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        if self.use_classifier:
            self.light_classifier = TLClassifier()
        #self.listener = tf.TransformListener()
        
        #rate = rospy.Rate(3000) # sleep 100 ms
        #rate.sleep()
        
        rospy.spin()
            
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.tree = cKDTree(self.waypoints_2D)
        rospy.logdebug("TL Detector - Base waypoints callback")

    def traffic_cb(self, msg):
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

        #rospy.logwarn("TL Detector - Closest light waypoint {} and state {}".format(light_wp, state))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
            #rospy.logwarn("TL Detector - light state changes")
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #rospy.logwarn("TL Detector - Publish next Red traffic waypoint {}".format(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #rospy.logwarn("TL Detector - Publish lst Red traffic waypoint  {}".format(self.last_wp))
        self.state_count += 1

    
    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        
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


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            #rospy.logwarn("TL Detector - no camera image")
            return False

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
        closest_stop_line_idx = None
        closest_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.tree):
            car_waypoint_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            rospy.logdebug("TL Detector - x: {} y: {}".format(self.pose.pose.position.x, self.pose.pose.position.y))
            rospy.logdebug("TL Detector - car_waypoint_idx {}".format(car_waypoint_idx))

            
            # find the closest visible traffic light (if one exists)
            closest_stop_line_idx = None
            for i, light in enumerate(self.lights):
                stop_line_pos = stop_line_positions[i]
                rospy.logdebug("TL Detector - stop_line_pos {}".format(stop_line_pos))
                stop_line_idx = self.get_closest_waypoint(stop_line_pos[0], stop_line_pos[1])
                rospy.logdebug("TL Detector - stop_line_idx {}".format(stop_line_idx))
                
                if stop_line_idx >= car_waypoint_idx and ( closest_stop_line_idx is None or stop_line_idx < closest_stop_line_idx):
                    closest_stop_line_idx = stop_line_idx
                    closest_light = light

        if closest_light:
            if self.use_classifier:
                state = self.get_light_state(closest_light)
                rospy.logwarn("TL Detector - state classified is {} and traffic light is {}".format(state, light.state))
            else:
                state = light.state
            return closest_stop_line_idx, state 
                          
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

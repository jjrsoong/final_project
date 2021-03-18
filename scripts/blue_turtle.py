#!/usr/bin/env python3

"""
This green turtle algorithm is based on the paper "A Sensor-Based Exploration Algorithm for Autonomous Map Generation on Mobile Robot Using Kinect" by 
"""

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from ghostturtle import Ghostturtle
import math
import numpy as np
import cv2
import cv_bridge
# name constants
PACTURTLE = "pacturtle"
RED = "redghost"
BLUE = "blueghost"
GREEN = "greenghost"
YELLOW = "yellowghost"

# Modes
EXPLORE = "explore"
HUNT = "hunt"


class Blueturtle(Ghostturtle):
    """Class for the behaviour of Blueturtle

    Args:
        Ghostturtle (Class)
    """
    def __init__(self):
        # Calls Ghostturtle's init function with the given arguments
        super().__init__(BLUE, self.laser_scan_callback,
                         self.image_callback, self.action_loop)

        # Explore vs hunting mode toggle
        self.mode = EXPLORE

        # Set up cv2
        self.bridge = cv_bridge.CvBridge()

        self.run()

    def laser_scan_callback(self, data: LaserScan):
        # do nothing
        return

    def image_callback(self, data: Image):
        """Given an image, checks if Pacturtle is in sight. If so changes mode to Hunting and charges at it

        Args:
            data (Image): Image data
        """
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        image = self.image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 255 108 10 rgba for orange
        lower_color = np.array([10, 125, 78])
        upper_color = np.array([22, 255, 255])
        
        # Masking only valid colors
        mask = cv2.inRange(hsv, lower_color, upper_color)
        
        # Shape info
        h, w, d = image.shape

        # using moments() function, determine the center of the pixels
        M = cv2.moments(mask)

        if M['m00'] > 0:
            # determine the center of the pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # visualize a yellow circle in our debugging window to indicate
            # the center point of the yellow pixels
            cv2.circle(image, (cx, cy), 20, (255, 255, 0), -1)
            # proportional control to have the robot follow the pixels
            err = w/2 - cx
            k_p = 1.0 / 1000.0
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = k_p * err
            self.cmd_vel_pub.publish(twist)
            self.mode = HUNT
        else: 
            if self.mode == HUNT:
                self.mode = EXPLORE
        return

    def move_forward(self):
        """Function to move turtle forward while avoiding obstacles 
        """
        # Move forward while avoiding obstacles
        twist = Twist()
        twist.linear.x = 0.3

        # Compute min distance of closest obstacle
        left_min = min(self.current_laser_data.ranges[280:])
        right_min = min(self.current_laser_data.ranges[:80])
        dist = min(left_min, right_min)
        if dist < 0.4:
            # Too close
            twist.linear.x = -0.1
            twist.angular.z = 0.3 if np.random.random() > 0.5 else -0.3
        else:
            # Continue moving forward
            twist.linear.x = 0.3
            twist.angular.z = np.random.normal()
            rospy.sleep(1)
        self.cmd_vel_pub.publish(twist)

    def action_loop(self):
        if self.mode == EXPLORE:
            if len(self.current_laser_data.ranges) != 360:
                return
            else:
                self.move_forward()

        elif self.mode == HUNT:  # Hunting mode
            # Charge straight at pacturtle
            return
        else:
            return



if __name__ == "__main__":
    Blueturtle()

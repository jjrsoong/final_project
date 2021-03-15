#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Ghostturtle(object): 
    def __init__(self, name, laser_scan_callback=None, image_callback=None, action_loop=None):
        self.name = name 
        rospy.init_node(name)


        self.current_laser_data = LaserScan()
        self._laser_scan_callback = laser_scan_callback
        rospy.Subscriber(f"{name}/scan", LaserScan, self.laser_scan_received)

        # ROS subscribe to robot's RGB camera data stream
        self.image = Image()
        self._image_callback = image_callback
        self.image_sub = rospy.Subscriber(
            f"{name}/camera/rgb/image_raw", Image, self.image_received)
            

        # Movement publisher
        self.cmd_vel_pub = rospy.Publisher(f"{name}/cmd_vel", Twist, queue_size=1)

        self._action_loop = action_loop

    def laser_scan_received(self, data:LaserScan):
        self.current_laser_data = data
        if self._laser_scan_callback != None: 
            self._laser_scan_callback(data)
    
    def image_received(self, data:Image):
        self.image = data 
        if self._image_callback != None: 
            self._image_callback(data)

    def run(self):
        r = rospy.Rate(10) # 2hz
        while not rospy.is_shutdown():
            if self._action_loop != None:
                self._action_loop()
            r.sleep()



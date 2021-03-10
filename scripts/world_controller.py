#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# name constants 
PACTURTLE = "pacturtle"
RED = "redghost"
BLUE = "blueghost"
GREEN = "greenghost"
YELLOW = "yellowghost"

class WorldController(object): 
    def __init__(self):

        # initialize this node 
        rospy.init_node('final_project_world_controller')

        # save initial 
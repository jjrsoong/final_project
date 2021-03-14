#!/usr/bin/env python3

import rospy
import math
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

        rospy.sleep(3)

        # initialize this node
        rospy.init_node('final_project_world_controller')

        # save initial positions
        self.init_pos = {}

        self.init_pos[PACTURTLE] = Point(x=2, y=5, z=0)
        self.init_pos[RED] = Point(x=3.5, y=8.5, z=0)
        self.init_pos[BLUE] = Point(x=4.5, y=8.5, z=0)
        self.init_pos[GREEN] = Point(x=3.5, y=9.5, z=0)
        self.init_pos[YELLOW] = Point(x=4.5, y=9.5, z=0)

        # Spawn point is 3 units to the right for ghosts
        self.spawn_point = {}
        for key, value in self.init_pos.items():
            if key == PACTURTLE:
                continue
            p = value
            p.x += 3
            self.spawn_point[key] = p

        # Keep track of state to avoid sending duplicate messages
        self.reset_world_in_progress = False

        # Subscribe to model states
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.model_states_received)

        # Model state pub
        self.model_states = ModelState()
        self.model_states_pub = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=10)


        rospy.sleep(1)

        self.reset_world()

        self.run()

    def ghosts_is_in_init_pos(self):
        # Checks if the ghosts are in the initialized positions 
        error = 0.01
        for turtle in self.init_pos.keys():
            index = self.model_states.name.index(turtle)
            turtle_pose = self.model_states.pose[index]
            if get_point_dist(turtle_pose.position, self.init_pos[turtle]) > error:     
                return False
            
        return True 

    def sequentially_spawn_ghosts(self): 
        for (turtle, position) in self.spawn_point.items():
            p = Pose(position=position)
            t = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            m_name = turtle

            model_state = ModelState(model_name=m_name, pose=p, twist=t)
            self.model_states_pub.publish(model_state)
            break # For now just spawn 1
        

    def reset_world(self):
        if self.reset_world_in_progress and not self.ghosts_is_in_init_pos():
            return
        else:
            self.reset_world_in_progress = True

        print("resetting world")
        for (turtle, position) in self.init_pos.items():
            p = Pose(position=position)
            t = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            m_name = turtle

            model_state = ModelState(model_name=m_name, pose=p, twist=t)
            self.model_states_pub.publish(model_state)

        if self.ghosts_is_in_init_pos(): 
            self.reset_world_in_progress = False
            self.sequentially_spawn_ghosts()
        


    def pacturtle_ghost_contact(self, location_mapping):
        x_delta_range = 0.5
        y_delta_range = 0.5 
        pacturtle_x = location_mapping[PACTURTLE].position.x
        pacturtle_y = location_mapping[PACTURTLE].position.y

        for turtle, pose in location_mapping.items():
            if turtle == PACTURTLE:
                continue
            position = pose.position
            x = position.x 
            y = position.y
            if abs(x - pacturtle_x) <= x_delta_range and abs(y - pacturtle_y) <= y_delta_range:
                print("contact", x, pacturtle_x, y, pacturtle_y)
                return True
        return False


    def model_states_received(self, data: ModelState):
        self.model_states = data
        location_mapping = {}
        for turtle in self.init_pos.keys():
            index = data.name.index(turtle)
            location_mapping[turtle] = data.pose[index]
        if (self.pacturtle_ghost_contact(location_mapping)):
            self.reset_world()
        return

    def run(self):
        rospy.spin()

def get_point_dist(point1: Point, point2: Point):
    x1, y1, z1 = point1.x, point1.y, point1.z
    x2, y2, z2 = point2.x, point2.y, point2.z
    d = math.sqrt(math.pow(x2 - x1, 2) +
                math.pow(y2 - y1, 2) +
                math.pow(z2 - z1, 2)* 1.0) 
    return d

if __name__ == "__main__":
    node = WorldController()

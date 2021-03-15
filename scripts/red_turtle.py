#!/usr/bin/env python3

import rospy
import math
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header
from numpy import genfromtxt
from queue import Queue

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

RED = "redghost"
PACTURTLE = "pacturtle"
MAP_WIDTH = 9
MAP_HEIGHT = 11
PAC_CELL = 0
RED_CELL = 0

class RedTurtle(object):
    def __init__(self):

        # rospy.sleep(1)

        # initialize this node
        rospy.init_node('final_project_red_turtle')

        self.unpack_adjMatrix()
        self.create_adjLists()
        self.init_bfs_entities()

        # save initial positions
        self.init_pos = {}
        self.init_pos[PACTURTLE] = Point(x=2, y=5, z=0)
        self.init_pos[RED] = Point(x=3.5, y=8.5, z=0)

        self.PAC_CELL = 0
        self.RED_CELL = 0
        # Ghost is initialized facing right (up = 0 degree)
        self.PAST_DIRECTION = 90

        # Subscribe to model states
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.turtle_hunter)

        # Command Vel pub
        self.command_pub = rospy.Publisher("/redghost/cmd_vel", Twist, queue_size=10)


        rospy.sleep(1)

        self.run()

    # Unpack the adjaceny matrix text file into an array
    # Note the generated matrix has one too many values -- the newline
    # char is treated as a -1. Will ignore this when working with adj matrix
    # (we do not use negative values in our matrix)
    def unpack_adjMatrix(self):
        self.matrix = genfromtxt("../map/Pacturtle_v2/map.txt", dtype=int, comments="#",
        delimiter=",", autostrip = True, unpack=False)

    # Uses unpacked adjaceny matrix to create a list of adjancent cells for each cell.
    # Packaged into a single dictionary
    def create_adjLists(self):
        self.adjLists = {}
        for node in range(0, 99):
            adjacent = []
            for pos_adj in range(0, 99):
                # Checks adjacen matrix to see if pos_adj is a valid adjancent cell
                if self.matrix[node][pos_adj] == 1:
                    adjacent.append(pos_adj)
            self.adjLists[node] = adjacent

    # BFS, source: https://www.youtube.com/watch?v=PQhMkmhYZjQ
    def init_bfs_entities(self):
        self.parentList = {}
        for source in range(0, 99):
            # Create and initialize required var to "empty" position
            visited = {}
            dist = {}
            parent = {}
            traversal = []
            queue = Queue()
            for node in self.adjLists.keys():
                visited[node] = False
                parent[node] = None
                dist[node] = -1

            # Begin BFS
            visited[source] = True
            dist[source] = 0
            queue.put(source)

            while not queue.empty():
                u = queue.get()
                traversal.append(u)

                for v in self.adjLists[u]:
                    if not visited[v]:
                        visited[v] = True
                        parent[v] = u
                        dist[v] = dist[u] + 1
                        queue.put(v)
            self.parentList[source] = parent

    # BFS is complete at this point
    # Use BFS entities to determine shortest path
    def shortest_path(self, source, end):
        path = []
        parent = self.parentList[source]
        while end is not None:
            path.append(end)
            end = parent[end]
        path.reverse()
        self.shortestPath = path

    def move(self):
        target_cell = 0
        for next_cell in self.shortestPath:
            if self.RED_CELL != next_cell:
                target_cell = next_cell
                break
        # Determine direction of target cell relative to current cell
        direction = ''
        # Below 2 are conditions for moving along the x axis
        if target_cell == self.RED_CELL - 1:
            direction = 270
        elif target_cell == self.RED_CELL + 1:
            direction = 90
        # Below 2 are conditions of moving along the y axis
        elif target_cell == self.RED_CELL + MAP_WIDTH:
            direction = 0
        else:
            direction = 180

        angle = direction - self.PAST_DIRECTION
        self.turn_around(angle)

        # self.turn_around(90)
        forward = True
        # Range, in meters, required before bot recalculates shortest path
        error = 0.1
        command = Twist()
        command.linear.x = 0.5
        # If moving along the x axis (left or right), look at x coordinate to determine
        # midpoint. Else, use y coordinate to detemrine proximity to cell midpoint
        if direction == 90 or direction == 270:
            midpoint = self.find_midpoint(target_cell, "x")
            # print('mid x ' + str(midpoint) + ' x pos ' + str(self.RED_POS.x))
            # print(redturtle_data)
            if midpoint - error < self.RED_POS.x and midpoint + error > self.RED_POS.x:
                forward = False
                print('stop forward x')
        # Robot is moving along the y axis
        else:
            midpoint = self.find_midpoint(target_cell, "y")
            print('mid y ' + str(midpoint) + ' y pos ' + str(self.RED_POS.y))
            if midpoint - error < self.RED_POS.y and midpoint + error > self.RED_POS.y:
                forward = False
                print('stop forward y')
        if not forward:
            command.linear.x = 0
        self.command_pub.publish(command)

    # Determines the coordinate value of the midpoint of a cell for the specificed axis
    # Add 0.5 to get the midpoint of the cell
    def find_midpoint(self, cell, axis):
        if axis=="x":
            return cell%MAP_WIDTH + 0.5
        else:
            return math.floor(cell/MAP_WIDTH) + 0.5

    #Helper function for rotating bot. Recycled from q_learning_project
    def turn_around(self, angle):
        print('turning')
        relative_angle = (angle * math.pi) / 180
        angular_speed = 0.5
        current_angle = 0
        firstTime = rospy.Time.now().to_sec()
        while current_angle < relative_angle:
            twister = Twist()
            twister.angular.z = angular_speed
            self.command_pub.publish(twister)

            curTime = rospy.Time.now().to_sec() - firstTime
            current_angle = angular_speed*(curTime)
        twister.angular.z = 0
        print('end')
        self.command_pub.publish(twister)

    # Helper function that translates a point into a cell location
    # on the map (map designed so that each cell is 1m x 1m)
    def determine_cell(self, position : Point):
        column = math.floor(position.x)
        row = math.floor(position.y)
        cell = MAP_WIDTH * row + column
        return cell

    def turtle_hunter(self, data: ModelState):
        # Extract Location of Red Turtle and Pacturtle
        pacturtle_data = Pose()
        pacturtle_cell = 0
        redturtle_data = Pose()
        redturtle_cell = 0
        for turtle in self.init_pos.keys():
            index = data.name.index(turtle)
            if turtle == PACTURTLE:
                pacturtle_data = data.pose[index]
                pacturtle_cell = self.determine_cell(pacturtle_data.position)
            else:
                redturtle_data = data.pose[index]
                redturtle_cell = self.determine_cell(redturtle_data.position)
                self.RED_POS = redturtle_data.position
        # One of the bots has changed positions, recalculate shortest path
        if not redturtle_cell == self.RED_CELL or not pacturtle_cell == self.PAC_CELL:
            self.RED_CELL = redturtle_cell
            self.PAC_CELL = pacturtle_cell
            print(self.RED_CELL)
            print(self.PAC_CELL)
            self.shortest_path(self.RED_CELL, self.PAC_CELL)
            print(self.shortestPath)

            self.move()
        return

    def pacturtle_ghost_contact(self, location_mapping):
        # Checks if the pacturtle is close enough to the ghostturtle
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
        # Callback for we receive model_states
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
    node = RedTurtle()

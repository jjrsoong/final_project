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

YELLOW = "yellowghost"
PACTURTLE = "pacturtle"
MAP_WIDTH = 9
MAP_HEIGHT = 11
INTEREST_PT = 0
YELLOW_CELL = 0

class YellowTurtle(object):
    def __init__(self):

        # rospy.sleep(1)

        # initialize this node
        rospy.init_node('final_project_yellow_turtle')

        self.unpack_adjMatrix()
        self.create_adjLists()
        self.init_bfs_entities()

        # save initial positions
        self.init_pos = {}
        self.init_pos[PACTURTLE] = Point(x=2, y=5, z=0)
        self.init_pos[YELLOW] = Point(x=8.5, y=10.5, z=0)

        self.INTEREST_PT = 33
        self.YELLOW_CELL = 0
        # Initially stationary (not moving forward)
        self.forward = False
        # Initially has not spotted Pacturtle with camera
        self.sighted = False

        # Subscribe to model states
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.turtle_hunter)

        # Command Vel pub
        self.command_pub = rospy.Publisher("/yellowghost/cmd_vel", Twist, queue_size=10)

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

    def get_yaw_from_pose(self, p):
        """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

        yaw = (euler_from_quaternion([
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w])
                [2])

        return yaw

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

    # Helper function that update INTEREST_PT once yellow turtle has reached
    # current point of interest
    # Although interest points are current hardcoded, future work could encompass
    # many points of interest dynamic
    def checkpoint(self):
        if self.YELLOW_CELL == 33:
            self.INTEREST_PT = 48
        elif self.YELLOW_CELL == 48:
            self.INTEREST_PT = 78

    def move(self):
        # Check if turtle is at point of interest. If so and pacturtle has not been spotted,
        # move to next point of interest
        self.checkpoint()
        # Find the next cell to traverse to
        # Required step since, depending on exact coordinates, shortestPath
        # may or many not include cell the bot is currently in (if it does, need to
        # ignore current cell and take the next one)
        target_cell = 0
        for next_cell in self.shortestPath:
            if self.YELLOW_CELL != next_cell:
                target_cell = next_cell
                break
        # Determine direction of target cell relative to current cell
        # Below 2 are conditions for moving along the x axis
        # If target cell is to the left
        if target_cell == self.YELLOW_CELL - 1:
            target_yaw = 3.1415
        # If target cell is to the right
        elif target_cell == self.YELLOW_CELL + 1:
            target_yaw = 0
        # Below 2 are conditions of moving along the y axis
        # If target cell is to the top
        elif target_cell == self.YELLOW_CELL + MAP_WIDTH:
            target_yaw = 1.5708
        # If target cell is to the bottom
        else:
            target_yaw = -1.5708

        yaw_error = 0.1
        current_yaw = self.get_yaw_from_pose(self.POSE)
        # Check for edge cases when yaw goes from positive to negative (ie. around 0,
        # transition from 3.14 to -3.14). If so, use additional rules to check for error margin
        # or negative numbers
        turning = False
        if target_yaw == 3.1415 or target_yaw == 0:
            if current_yaw > 0:
                if current_yaw > target_yaw + yaw_error or current_yaw < target_yaw - yaw_error:
                    turning = True
            else:
                if current_yaw > -1*(target_yaw) + yaw_error or current_yaw < -1*(target_yaw) - yaw_error:
                    turning = True
        else:
            if current_yaw > target_yaw + yaw_error or current_yaw < target_yaw - yaw_error:
                turning = True
            else:
                turning = False
        # Still need to turn, don't move forward yet
        if turning:
            self.forward = False
            command = Twist()

            # Check for cases where moving clockwise is better than the default
            # counter clockwise
            if target_yaw == 0 and current_yaw > 0:
                command.angular.z = -0.7
            elif target_yaw == 3.1415 and current_yaw < 0:
                command.angular.z = -0.7
            elif target_yaw == -1.5708 and current_yaw < 1.5708 and current_yaw > -1.5708:
                command.angular.z = -0.7
            # Default if counter clockwise
            else:
                command.angular.z = 0.5
            # print(str(current_yaw) + '      ' + str(target_yaw))
            self.command_pub.publish(command)
        # Turning has completed, can begin moving forward
        else:
            self.forward = True
            # Margin of error allowed before bot recalculates shortest path
            error = 0.1
            command = Twist()
            x_midpoint = self.find_midpoint(target_cell, "x")
            y_midpoint = self.find_midpoint(target_cell, "y")

            # Robot is moving along the x axis, check if it has reached midpoint of next cell
            if target_yaw == 0 or target_yaw == 3.1415:
                if x_midpoint - error < self.YELLOW_POS.x and x_midpoint + error > self.YELLOW_POS.x:
                    self.forward = False
                    print('stop forward x ' + str(self.YELLOW_POS.x))
            # Robot is moving along the y axis, check if it has reached midpoint of next cell
            else:
                if y_midpoint - error < self.YELLOW_POS.y and y_midpoint + error > self.YELLOW_POS.y:
                    self.forward = False
                    print('stop forward y ' + str(self.YELLOW_POS.y))

            if not self.forward:
                print('stop moving forward')
                command.linear.x = 0
                self.command_pub.publish(command)
                rospy.sleep(1)
            else:
                command.linear.x = 0.3
                self.command_pub.publish(command)

    # Determines the coordinate value of the midpoint of a cell for the specificed axis
    # Add 0.5 to get the midpoint of the cell
    def find_midpoint(self, cell, axis):
        if axis=="x":
            return cell%MAP_WIDTH + 0.5
        else:
            return math.floor(cell/MAP_WIDTH) + 0.5

    # Helper function that translates a point into a cell location
    # on the map (map designed so that each cell is 1m x 1m)
    def determine_cell(self, position : Point):
        column = math.floor(position.x)
        row = math.floor(position.y)
        cell = MAP_WIDTH * row + column
        return cell

    # Callback function that extracts ModelState data and calls move() command
    def turtle_hunter(self, data: ModelState):
        # Extract Location of Yellow Turtle and Pacturtle
        pacturtle_data = Pose()
        pacturtle_cell = 0
        yellowturtle_data = Pose()
        yellowturtle_cell = 0
        for turtle in self.init_pos.keys():
            index = data.name.index(turtle)
            if turtle == YELLOW:
                yellowturtle_data = data.pose[index]
                yellowturtle_cell = self.determine_cell(yellowturtle_data.position)
                self.YELLOW_POS = yellowturtle_data.position
                self.POSE = yellowturtle_data
        if not self.sighted:
            # Bot at midpoint, calculate new shortest path from current cell
            if not self.forward:
                self.YELLOW_CELL = yellowturtle_cell
                self.shortest_path(self.YELLOW_CELL, self.INTEREST_PT)
                self.move()
            # Still moving forward, do not bother it until reaches midpoint of target cell
            else:
                self.move()
        # TODO - added in camera tracking and hutning code here
        # else:
        return

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = YellowTurtle()

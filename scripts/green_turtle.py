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
# name constants
PACTURTLE = "pacturtle"
RED = "redghost"
BLUE = "blueghost"
GREEN = "greenghost"
YELLOW = "yellowghost"

# Modes
EXPLORE = "explore"
HUNT = "hunt"
MOVE_TO_BRANCH = "forward"


class Greenturtle(Ghostturtle):
    def __init__(self):
        super().__init__(GREEN, self.laser_scan_callback,
                         self.image_callback, self.action_loop)

        # Cheat gps cuz I don't wanna do odometry correction :/
        # This exploration algorithm is hard enough as is :(
        self.gps = ModelState
        rospy.Subscriber(f"{GREEN}/gps", Pose, self.gps_callback)

        # Explore vs hunting mode toggle
        self.mode = EXPLORE

        self.particle_cloud = []

        self.branch_dist = 0
        self.branch_time = rospy.Time.now().to_sec()
        self.move_to_branch_speed = 0.2
        self.branch_direction = ""

        self.run()

    def laser_scan_callback(self, data: LaserScan):
        # do nothing
        return

    def gps_callback(self, data: Pose):
        self.gps = data

    def image_callback(self, data):
        # TODO: Check if pacturtle color is present, and toggle mode
        return

    def move_forward(self):
        print("moving forward")
        # Move forward while avoiding obstacles
        twist = Twist()
        twist.linear.x = 0.2

        # Compute min distance of closest obstacle
        left_min = min(self.current_laser_data.ranges[280:])
        right_min = min(self.current_laser_data.ranges[:80])
        dist = min(left_min, right_min)
        if dist < 0.4:
            # Too close
            print("too close, backing off")
            twist.linear.x = -0.05
            twist.angular.z = 0.3 if left_min < right_min else -0.3
        else:
            # Continue moving forward
            twist.linear.x = 0.1
            twist.angular.z = 0
        print(twist.angular.z)
        self.cmd_vel_pub.publish(twist)

        # # Move forward while staying center of 2 walls
        # left = max(self.current_laser_data.ranges[260:290])
        # right = max(self.current_laser_data.ranges[70:100])
        # twist = Twist()
        # twist.linear.x = 0.1
        # k = 0.5
        # twist.angular.z = k * abs(left - right) * (1 if left < right else -1)
        # self.cmd_vel_pub.publish(twist)

        # # Move forward while staying near the center 2
        # top_left = min(self.current_laser_data.ranges[310:320])
        # top_right = min(self.current_laser_data.ranges[40:50])
        # bot_left = min(self.current_laser_data.ranges[220:230])
        # bot_right = min(self.current_laser_data.ranges[130:140])
        # front = min(self.current_laser_data.ranges[:30] + self.current_laser_data.ranges[330:])
        # if front < 0.3:
        #     print("too close, backing off")
        #     twist.linear.x = -0.05
        #     twist.angular.z = 0.3 if top_left < top_right else -0.3
        # else:
        #     top_diff = top_right-top_left
        #     bot_diff = bot_right-bot_left
        #     k = 0.5
        #     twist.angular.z = top_diff * k + bot_diff * k
        # self.cmd_vel_pub.publish(twist)  

    def move_to_branch(self):
        print("moving to branch")
        travelled_time = rospy.Time.now().to_sec() - self.branch_time
        distance_travelled = self.move_to_branch_speed * travelled_time
        twist = Twist()
        print("travelled", distance_travelled, self.branch_dist)
        if distance_travelled >= self.branch_dist:
            # rotate to branch 
            print("rotating", self.branch_direction)
            twist.angular.z = (math.pi / 8 if self.branch_direction == "right" else -1 * math.pi / 8)
            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(4)
            self.mode = EXPLORE
            self.branch_dist = 0
        else:
            # Move forward while avoiding collisions 
            self.move_forward()
            


    def action_loop(self):
        if self.mode == EXPLORE:
            if len(self.current_laser_data.ranges) != 360:
                return

            # update particle cloud
            new_particle_cloud = []
            for index, point in enumerate(self.current_laser_data.ranges):
                theta = index * (math.pi / 180)
                x = min(point, self.current_laser_data.range_max) * \
                    math.sin(theta)
                y = min(point, self.current_laser_data.range_max) * \
                    math.cos(theta)
                new_particle_cloud.append((x, y))

            # Check for any branches with the new particle cloud
            branches = []
            threshold = 1
            for i in range(len(new_particle_cloud) - 1):
                # Check the y-coordinate of 2 points
                dist = abs(new_particle_cloud[i]
                           [1] - new_particle_cloud[i+1][1])
                if dist > threshold and (i < 150 or i > 300):
                    branches.append(
                        (new_particle_cloud[i], i, new_particle_cloud[i + 1], i + 1, dist))

            # pick branch (if any) where point is closest to current location
            # TODO: Check with mem that we did not pick this branch
            if (len(branches) > 0):
                print("branch found")
                branch_indexes = [branch[1] for branch in branches]
                branch_dists = [
                    (math.sqrt(branch[0][0] ** 2 + branch[0][1] ** 2) +
                     math.sqrt(branch[2][0] ** 2 + branch[2][1] ** 2)) / 2
                    for branch in branches]
                closest_branch_index = branch_dists.index(min(branch_dists))
                # Traverse forward towards the branch, then turn to where the branch is
                theta = branch_indexes[closest_branch_index] * (math.pi / 180)
                forward_distance = abs(
                    branch_dists[closest_branch_index] * math.cos(theta))

                self.branch_time = rospy.Time.now().to_sec()
                self.branch_dist = forward_distance
                self.branch_direction = "left" if theta > (math.pi) else "right"
                self.mode = MOVE_TO_BRANCH

            else:
                # No branch, move forward
                print("no branches")
                self.move_forward()

            return

        elif self.mode == HUNT:  # Hunting mode
            # Charge straight at pacturtle
            return
        elif self.mode == MOVE_TO_BRANCH:
            self.move_to_branch()
        else:
            return


def get_point_dist(point1: Point, point2: Point):
    x1, y1, z1 = point1.x, point1.y, point1.z
    x2, y2, z2 = point2.x, point2.y, point2.z
    d = math.sqrt(math.pow(x2 - x1, 2) +
                  math.pow(y2 - y1, 2) +
                  math.pow(z2 - z1, 2) * 1.0)
    return d


if __name__ == "__main__":
    Greenturtle()

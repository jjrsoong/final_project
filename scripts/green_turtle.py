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
        self.branch_loc = Point()

        # Keep track of visited branch cells
        self.visited = set()

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
        # print("moving to branch")
        # travelled_time = rospy.Time.now().to_sec() - self.branch_time
        # distance_travelled = self.move_to_branch_speed * travelled_time
        # twist = Twist()
        # print("travelled", distance_travelled, self.branch_dist)
        # if distance_travelled >= self.branch_dist:
        #     # rotate to branch 
        #     print("rotating", self.branch_direction)
        #     twist.angular.z = (math.pi / 8 if self.branch_direction == "right" else -1 * math.pi / 8)
        #     twist.linear.x = 0
        #     self.cmd_vel_pub.publish(twist)
        #     rospy.sleep(4)
        #     self.mode = EXPLORE
        #     self.branch_dist = 0
        # else:
        #     # TODO: Move to branch location while avoiding collisions
        #     self.move_forward()
        #     # Should use gps data 
        current_pos = self.gps 
        # Move forward in current orientation 

        pos_to_branch_loc = [self.branch_loc.x - current_pos.position.x, self.branch_loc.y - current_pos.position.y]
        dist_to_branch = np.linalg.norm(pos_to_branch_loc)
        if dist_to_branch < 0.4:
            # change mode 
            # print("rotating", self.branch_direction)
            twist=Twist()
            # twist.angular.z = (math.pi / 8 if self.branch_direction == "right" else -1 * math.pi / 8)
            twist.linear.x = 0.2
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)
            self.mode = EXPLORE
            self.branch_dist = 0
            print("exploring")
        else: 
            # MOve to target
            # print("moving to target")
            # print("self", current_pos.position)
            # print("target", self.branch_loc)
            # print("pos_to_branch_loc", pos_to_branch_loc)
            twist = Twist()

            # Check if we're too close to a wall
            left_min = min(self.current_laser_data.ranges[315:])
            right_min = min(self.current_laser_data.ranges[:45])
            dist = min(left_min, right_min)
            if dist < 0.4:
                print("too close, backing off 2")
                twist.linear.x = -0.05
                twist.angular.z = 0.3 if left_min < right_min else -0.3
            else: 
                k = 0.3 if dist_to_branch < 1 else 0.5 
                current_theta = euler_from_quaternion([
                    current_pos.orientation.x, 
                    current_pos.orientation.y, 
                    current_pos.orientation.z, 
                    current_pos.orientation.w])[2]
                twist.angular.z = k * (math.atan(pos_to_branch_loc[1] / pos_to_branch_loc[0]) - current_theta)
                twist.linear.x = 0.1 if dist_to_branch > 1 else 0.05
                print(f"""current pos {current_pos.position.x}/{self.branch_loc.x} {current_pos.position.y}/{self.branch_loc.y} 
                dist: {dist_to_branch}
                curren_theta: {current_theta}
                angular z: {twist.angular.z}
                desired_z: {math.atan(pos_to_branch_loc[0] / pos_to_branch_loc[1])}
                """, end='\r')

            self.cmd_vel_pub.publish(twist)

    def action_loop(self):
        if self.mode == EXPLORE:
            if len(self.current_laser_data.ranges) != 360:
                return

            # update particle cloud
            new_particle_cloud = []
            current_pos = self.gps
            cur_orientation = euler_from_quaternion([
                current_pos.orientation.x, 
                current_pos.orientation.y, 
                current_pos.orientation.z, 
                current_pos.orientation.w])
            for index, point in enumerate(self.current_laser_data.ranges):
                theta = cur_orientation[2] + index * (math.pi / 180)
                x = min(point, self.current_laser_data.range_max) * math.cos(theta)
                y = min(point, self.current_laser_data.range_max) * math.sin(theta)
                new_particle_cloud.append((x, y))
            # print(self.current_laser_data.ranges[0], self.current_laser_data.ranges[90], self.current_laser_data.ranges[180], self.current_laser_data.ranges[270])
            # print(cur_orientation[2], new_particle_cloud[0], new_particle_cloud[90], new_particle_cloud[180], new_particle_cloud[270])
            # exit()

            # Check for any branches with the new particle cloud
            branches = []
            threshold = 1
            for i in range(len(new_particle_cloud) - 1):
                # Check the y-coordinate of 2 points
                dist_y = abs(new_particle_cloud[i][1] - new_particle_cloud[i+1][1])
                if dist_y > threshold and (i < 60 or i > 300):
                    print("branch found", new_particle_cloud[i], new_particle_cloud[i+1], i )
                    branch_point_center = [(new_particle_cloud[i][0] + new_particle_cloud[i+1][0])/2, (new_particle_cloud[i][1] + new_particle_cloud[i+1][1])/2]
                    branch_point_dist = math.sqrt((branch_point_center[0]) ** 2 + (branch_point_center[1]) ** 2)
                    if branch_point_dist > 1.5:
                        continue
                        
                    # check if this branch point is among one that we've branched to before 
                    branch_point_global_coord = [self.gps.position.x, self.gps.position.y]
                    branch_point_global_coord[0] += branch_point_center[0]
                    branch_point_global_coord[1] += branch_point_center[1]
                    branch_point_cell = int(branch_point_global_coord[0]) + int(branch_point_global_coord[1]) * 9
                    if branch_point_cell not in self.visited:
                        branches.append(
                            (new_particle_cloud[i], i, new_particle_cloud[i + 1], i + 1, dist_y))
            print("branches", branches)
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
                print("closest branch:", branches[closest_branch_index])
                print("closest distance:", branch_dists[closest_branch_index])
                # self.cmd_vel_pub.publish(Twist())
                # exit()
                # Traverse forward towards the branch, then turn to where the branch is
                theta = branch_indexes[closest_branch_index] * (math.pi / 180) + math.pi / 2
                forward_distance = abs(
                    branch_dists[closest_branch_index] * math.cos(theta))

                self.branch_time = rospy.Time.now().to_sec()
                self.branch_dist = forward_distance
                self.branch_loc = self.gps.position
                self.branch_loc.x += ((branches[closest_branch_index][0][0] + branches[closest_branch_index][2][0]) / 2)
                self.branch_loc.y += ((branches[closest_branch_index][0][1] + branches[closest_branch_index][2][1]) / 2)
                self.visited.add(int(self.branch_loc.x) + int(self.branch_loc.y) * 9)
                print("moving", (branches[closest_branch_index][0][0] + branches[closest_branch_index][2][0]) / 2, (branches[closest_branch_index][0][1] + branches[closest_branch_index][2][1]) / 2)
                print("going to", self.branch_loc.x, self.branch_loc.y)
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

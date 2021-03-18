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
MOVE_TO_BRANCH = "forward"


class Greenturtle(Ghostturtle):
    """Class for Greenturtle behaviour

    Args:
        Ghostturtle (Class)
    """

    def __init__(self):
        """Inits a Greenturtle node 
        """
        super().__init__(GREEN, self.laser_scan_callback,
                         self.image_callback, self.action_loop)

        # Cheat gps cuz I don't wanna do odometry correction :/
        # This exploration algorithm is hard enough as is :(
        self.gps = ModelState
        rospy.Subscriber(f"{GREEN}/gps", Pose, self.gps_callback)

        # Explore vs hunting mode toggle
        self.mode = EXPLORE

        # Used to track branches
        self.particle_cloud = []
        self.branch_dist = 0
        self.branch_time = rospy.Time.now().to_sec()
        self.move_to_branch_speed = 0.2
        self.branch_direction = ""
        self.branch_loc = Point()

        # Keep track of visited branch cells to avoid duplicates
        self.visited = set()

        self.bridge = cv_bridge.CvBridge()

        self.run()

    def laser_scan_callback(self, data: LaserScan):
        # do nothing
        return

    def gps_callback(self, data: Pose):
        """Callback when self's ModelState was received

        Args:
            data (Pose): [description]
        """
        self.gps = data

    def image_callback(self, data: Image):
        """Callback for image data, checks if Pacturtle is in sight, if so charges towards it. 

        Args:
            data (Image): 
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
        """Move Forward while avoiding obstacles
        """
        # Move forward while avoiding obstacles
        twist = Twist()
        twist.linear.x = 0.2

        # Compute min distance of closest obstacle
        left_min = min(self.current_laser_data.ranges[280:])
        right_min = min(self.current_laser_data.ranges[:80])
        dist = min(left_min, right_min)
        if dist < 0.4:
            # Too close
            twist.linear.x = -0.05
            twist.angular.z = 0.3 if left_min < right_min else -0.3
        else:
            # Continue moving forward
            twist.linear.x = 0.1
            twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def move_to_branch(self):
        """Moves to a given branch 
        """
        current_pos = self.gps
        pos_to_branch_loc = [self.branch_loc.x - current_pos.position.x,
                             self.branch_loc.y - current_pos.position.y]
        dist_to_branch = np.linalg.norm(pos_to_branch_loc)

        if dist_to_branch < 0.4:
            # Move forward
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)
            self.mode = EXPLORE
            self.branch_dist = 0
        else:
            # Move to target
            twist = Twist()

            # Check if we're too close to a wall
            left_min = min(self.current_laser_data.ranges[310:350])
            right_min = min(self.current_laser_data.ranges[10:50])
            dist = min(left_min, right_min)
            if dist < 0.4:
                # back off
                twist.linear.x = -0.2
                twist.angular.z = 0.4 if left_min < right_min else -0.4
            else:
                # move to the branch target
                k = 0.3 if dist_to_branch < 1 else 0.5
                current_theta = euler_from_quaternion([
                    current_pos.orientation.x,
                    current_pos.orientation.y,
                    current_pos.orientation.z,
                    current_pos.orientation.w])[2]
                desired_angle = math.atan(
                    pos_to_branch_loc[1] / pos_to_branch_loc[0])

                # Angle correction
                if pos_to_branch_loc[0] < 0 and pos_to_branch_loc[1] > 0:
                    desired_angle = math.pi - desired_angle
                elif pos_to_branch_loc[0] < 0 and pos_to_branch_loc[1] < 0:
                    desired_angle = -1 * math.pi + desired_angle

                correction = desired_angle - current_theta
                if current_theta > 0 and desired_angle < 0:
                    if current_theta - math.pi < desired_angle:
                        # shortest path is clockwise
                        correction = abs(desired_angle - current_theta) * -1
                    else:
                        # shortest path is counter clockwise
                        correction = math.pi - current_theta + desired_angle + math.pi
                elif current_theta < 0 and desired_angle > 0:
                    if current_theta + math.pi < desired_angle:
                        # shortest path is clockwise
                        correction = abs(
                            math.pi - desired_angle + math.pi - current_theta) * -1
                    else:
                        # shortest path is counter clockwise
                        correction = desired_angle + current_theta * -1

                twist.angular.z = k * (correction) / math.pi

                twist.linear.x = 0.1 if dist_to_branch > 1 else 0.05

            self.cmd_vel_pub.publish(twist)

    def action_loop(self):
        """Action loop that executes at 10Hz
        """
        if self.mode == EXPLORE:
            # catch not ready case
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
                x = min(point, self.current_laser_data.range_max) * \
                    math.cos(theta)
                y = min(point, self.current_laser_data.range_max) * \
                    math.sin(theta)
                new_particle_cloud.append((x, y))

            # Check for any branches with the new particle cloud
            branches = []
            threshold = 1
            for i in range(len(new_particle_cloud) - 1):
                # Check the y-coordinate of 2 points
                dist_y = abs(new_particle_cloud[i]
                             [1] - new_particle_cloud[i+1][1])
                if dist_y > threshold and (i < 60 or i > 300):
                    branch_point_center = [(new_particle_cloud[i][0] + new_particle_cloud[i+1][0])/2,
                                           (new_particle_cloud[i][1] + new_particle_cloud[i+1][1])/2]
                    branch_point_dist = math.sqrt(
                        (branch_point_center[0]) ** 2 + (branch_point_center[1]) ** 2)
                    if branch_point_dist > 1.5:
                        # too far 
                        continue

                    # check if this branch point is among one that we've branched to before
                    branch_point_global_coord = [
                        self.gps.position.x, self.gps.position.y]
                    branch_point_global_coord[0] += branch_point_center[0]
                    branch_point_global_coord[1] += branch_point_center[1]
                    branch_point_cell = int(
                        branch_point_global_coord[0]) + int(branch_point_global_coord[1]) * 9
                    if branch_point_cell not in self.visited:
                        branches.append(
                            (new_particle_cloud[i], i, new_particle_cloud[i + 1], i + 1, dist_y))
            # pick branch (if any) where point is closest to current location
            if (len(branches) > 0):
                branch_indexes = [branch[1] for branch in branches]
                branch_dists = [
                    (math.sqrt(branch[0][0] ** 2 + branch[0][1] ** 2) +
                     math.sqrt(branch[2][0] ** 2 + branch[2][1] ** 2)) / 2
                    for branch in branches]
                closest_branch_index = branch_dists.index(min(branch_dists))

                # Traverse forward towards the branch, then turn to where the branch is
                theta = branch_indexes[closest_branch_index] * \
                    (math.pi / 180) + math.pi / 2
                forward_distance = abs(
                    branch_dists[closest_branch_index] * math.cos(theta))

                self.branch_time = rospy.Time.now().to_sec()
                self.branch_dist = forward_distance
                self.branch_loc = self.gps.position
                self.branch_loc.x += ((branches[closest_branch_index]
                                       [0][0] + branches[closest_branch_index][2][0]) / 2)
                self.branch_loc.y += ((branches[closest_branch_index]
                                       [0][1] + branches[closest_branch_index][2][1]) / 2)
                self.visited.add(int(self.branch_loc.x) +
                                 int(self.branch_loc.y) * 9)
                if len(self.visited) >= 10:
                    # Reset visited after visiting a considerable amount of branches
                    self.visited = set()
                self.branch_direction = "left" if theta > (
                    math.pi) else "right"
                self.mode = MOVE_TO_BRANCH

            else:
                # No branch, move forward
                self.move_forward()
            return

        elif self.mode == HUNT:  # Hunting mode
            # Charge straight at pacturtle
            # Handled by image callback
            return
        elif self.mode == MOVE_TO_BRANCH:
            self.move_to_branch()
        else:
            return


def get_point_dist(point1: Point, point2: Point):
    """Returns the distance between 2 points

    Args:
        point1 (Point)
        point2 (Point)

    Returns:
        [float]: Distance
    """
    x1, y1, z1 = point1.x, point1.y, point1.z
    x2, y2, z2 = point2.x, point2.y, point2.z
    d = math.sqrt(math.pow(x2 - x1, 2) +
                  math.pow(y2 - y1, 2) +
                  math.pow(z2 - z1, 2) * 1.0)
    return d


if __name__ == "__main__":
    Greenturtle()

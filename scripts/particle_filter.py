#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random

from likelihood_field import LikelihoodField


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples

def get_coords_from_index(index: int, map: OccupancyGrid):
    x = (index % map.info.height) * map.info.resolution + map.info.origin.position.x
    y = (index // map.info.width) * map.info.resolution + map.info.origin.position.y
    return x, y

class Particle:
    def __str__(self):
        return 'Particle weight: {self.w}, Pose: {self.pose}'.format(self=self)
    def __init__(self, pose: Pose, w: float):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    # Given real coordinates, return the Occupancy grid index that it should be on
    def get_index(self, map: OccupancyGrid):
        width = map.info.width
        height = map.info.height 
        x = self.pose.position.x 
        y = self.pose.position.y 
        x_index = (x - map.info.origin.position.x) // map.info.resolution
        y_index = (y - map.info.origin.position.y) // map.info.resolution
        return x_index + y_index * width


class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.likelihood_field = None


        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        while(self.likelihood_field == None): # Make sure that we have a valid map before initializing particle cloud
            rospy.sleep(0.5)
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data: OccupancyGrid):
        self.map = data
        self.likelihood_field = LikelihoodField(data)

    def initialize_particle_cloud(self):
        
        # Find bounds to limit our initial particle cloud to be a polygon surrounding the house
        lower_y_bound = 0
        lower_x_bound = self.map.info.width
        upper_y_bound = 0
        upper_x_bound = 0
        ((x_lower, x_upper), (y_lower, y_upper)) = self.likelihood_field.get_obstacle_bounding_box()
        print(x_lower, x_upper, y_lower, y_upper)
        for y in range (self.map.info.height):
            for x in range (self.map.info.width):
                grid_cell = self.map.data[y * self.map.info.width + x]
                if (grid_cell != -1):
                    if (lower_y_bound == 0): # first valid cell we see as we're moving from y=0 upwards on the map data
                        lower_y_bound = y
                    if (x < lower_x_bound):
                        lower_x_bound = x
                    if (y > upper_y_bound):
                        upper_y_bound = y
                    if (x > upper_x_bound):
                        upper_x_bound = x

        for i in range(self.num_particles):
            rand_x = randint(lower_x_bound, upper_x_bound)
            rand_y = randint(lower_y_bound, upper_y_bound)
            rand_cell_index = rand_y * self.map.info.width + rand_x
            while(self.map.data[rand_cell_index] == -1):
                rand_x = randint(lower_x_bound, upper_x_bound)
                rand_y = randint(lower_y_bound, upper_y_bound)
                rand_cell_index = rand_y * self.map.info.width + rand_x
            pose = Pose()
            pose.position.x, pose.position.y = get_coords_from_index(rand_cell_index, self.map)
            q = quaternion_from_euler(0, 0 , np.random.uniform(0, 2 * np.pi))
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.particle_cloud.append(Particle(pose, 1))

        self.normalize_particles()

        self.publish_particle_cloud()



    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        # Calculate the current sum of all weights
        total_weight = 0
        for x in self.particle_cloud:
            particle_weight = x.w
            total_weight = total_weight + particle_weight

        # Transfer the particles and their normalized weights to the particle cloud
        for i, x in enumerate(self.particle_cloud):
            pose = x.pose
            particle_weight = x.w
            particle_weight = particle_weight / total_weight
            self.particle_cloud[i] = Particle(pose, particle_weight)

        return



    def publish_particle_cloud(self):
        print("publishing particle cloud", len(self.particle_cloud))

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO
        return



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        return


    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        return


        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        return



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()










# Particle Filter Project Writeup
## Name
Yves Shum, Joshua Soong

![ParticleFilterGif](./ParticleFilterRViz.gif)

## `Objectives`

This project aimed to build a working particle filter for our turtlebot. As such, we initialized a particle cloud, then constantly iterated through and updated individual particles to account for incoming measurement data and changes to our motion model.

## `High-Level Description`

We used a likelihood field to allow our robot to determine its location in the room. This technique involved the initialization of a cloud of several thousand particles, the updating of these particles’ location with data from our motion model, the updating of weights based on the measurement model data from several cardinal directions, and the subsequent resampling of these particles to increase the cloud’s density at the most likely robot locations. In essence, our likelihood field is a process of elimination that is able to account for environmental noise.

## `Code Structure`

### Movement 
Motion data was incorporated into our likelihood field in our update_particles_with_motion_model() function. This made sense because our program’s logic updated these particles’ locations with this new motion data before recomputating weights and resampling. Motion data was first isolated by comparing the difference in x and y coordinates and yaw between the most recent odometry reading and the previous odometry reading. Then, we used a for loop to iterate through our particle cloud and update each particle's location. Importantly, since the particle and robot headings (as given by the yaw) were not necessarily the same, we employed trigonometry to apportion the movement data properly along the x and y axes. Incorporating this movement data into our particle cloud is important because it ensures our program is not static but rather dynamic in that the particles mirror the robot’s every move. Finally, we injected Gaussian noise centered at 0 and a small standard deviation to each particle’s pose. 

### Computation of Weights
Weights were calculated in our update_particle_weights_with_measurement_model() function. Given the hardware limits of the computers we were working on, we decided to use one datapoint from every 45 degree angle (as opposed to every 1 degree angle) in our weight calculations. We then calculated the distance to the closest obstacle for each of the 45 degree angles and fed this distance into a zero-centered Gaussian function to find a probability value. As noted in our Challenges section, an if statement was used to attach low probabilities to particles who, after being updated with the robot’s movements, found themselves to be inside walls or outside the map. Computing weights is crucial to this program because it ensures particles closer to the robot’s location are more likely to be drawn than those that are farther away. In essence, weight computation provides our resampling function the material it needs to slowly pinpoint our robot’s location.

### Resampling
After weights have been normalized, they are inputted into our resample_particles() function. This function relies on the draw_random_sample(), a helper function that was provided in the starter code. Resample is the culmination of the likelihood field technique and it is crucial because it slowly whittles down the possible locations that the robot could be at. By resampling, we are applying probabilistic concepts over many iterations in a way that clusters the particles at, eventually, the robot’s true location.

## `Challenges`
Initially, we had trouble resampling our particle cloud. We kept getting a callback error when running our code, which we suspected was caused by either a performance issue or a race condition that resulted in our q value for some particles to return NaN. Interestingly, this problem occurred with clouds that had more than 35 particles but not with those that had less than 35 particles. Upon additional inspection, we found that this NaN error likely resulted from particles close to walls that, when subsequently updated with the motion model, traveled out of the map. With 35 particles as opposed to many thousands of particles, the probability of a particle ending up and then traveling into a wall or off the map was low. We solved this problem by explicitly inserting an if statement assigning such edge cases extremely low probabilities (during resampling, their likelihood of being chosen was very low).

Another challenge we faced was that our weights initially did not seem to be assigned properly. By going over our code again, we found that the culprit was not our logic but rather the standard deviation value being used in our gaussian function. By increasing this value from 0.1 to 0.7, our code worked as intended!

Finally, after running our robot at high speeds, we encountered a Transform Exception. Pouya explained that this occurs because the computer cannot perform the program’s computations as fast as the robot is outputting odometry readings. Put another way, our code cannot keep up with robot! We found that keeping our robot’s linear velocity under 0.05 m/s (0.1m/s if absolutely necessary) resolves this issue. If the need to go above 0.1 m/s arises, our code will not crash immediately, but one can expect the Transform Exception to appear after 20-30 seconds.

## `Future Work`
With more time, it would be interesting to see if we could optimize our likelihood field. By this, we mean minimizing the amount of time or iterations it takes for the robot to determine its location. We imagine this would take the form of optimizing the number of particles in the cloud, the number of cardinal directions used to compute weights, the incorporation of Z_max and Z_rand into our measurement model, the probabilities assigned to particles who moved into walls (or off the map), or the fine-tuning of the standard deviation value for our gaussian function in weight computations. Such optimization would not only provide a refreshing exercise but would likely also enable our robot to tackle larger and more complex maps without running out of computational power.

## `Takeaways`
While we are both still clueless about quaternions, we now understand Euler angles. This is important in case any of us become fighter pilots (in all seriousness, it was interesting to learn about this new type of coordinate system).

Modularize the work: the starter code was instrumental in helping us do this, but modularizing the program enables both partners to work simultaneously. This cuts down on the total time spent on the project while projecting a mechanism to both partners to work (and then double check each other’s work).

Visualize the program: it is important to understand how likelihood fields work and we found this was best done by visualizing the program before doing any coding. This can be done in your head or in RViz by using a blank map and then imagining how the particles start congregating.



> Anything below this line is part of the implementation plan
----------------------------------


## Implementation plan 
#### `initialize_particle_cloud()`
Given the map dimensions of x and z, we use numpy.random to generate random coordinates for a particle as well as its orientation. We repeat this procedure for the number of particles desired, and save the results to an array `particle_cloud`, probably as a dictionary keyed by `x`, `z`, `dir`. We’ll first try our particle filter with a small number of particles to make sure our algorithm is correct. Later on we’ll scale up the number of particles based on performance.
After sampling the particle coordinates, we can look through the map data and calculate what the sensor values for each particle should be in the directions (front, left, back, right). We can save these values to the dictionary keyed as `s_front`, `s_left`, `s_back`, `s_right`
To test this, we will either visualize it using RViz or we will print out the drawn coordinates and ensure that they are within the map parameters (ie. x and y coordinates corresponding to a point within the map, sensor values align with the orientation etc.).
#### `update_particles_with_motion_model()`
Given an input motion, we loop through the `particle_cloud` and increment each point’s x and z values, and orientation. If the motion is angular, we can just apply simple trigonometry to get the horizontal and vertical components and the new orientation.
Using the new x, z and orientation values, we can look through the map data and recalculate what the theoretical sensor values should be and update `s_front`, `s_left`, `s_back`, `s_right` 
To test this, we can visualize the “before” and “after” particle clouds and see if they are identical save for a shift by the expected motion magnitude. To test expected sensor values, we will attempt to calculate one or two sets of sensor values by hand and see if they match our program’s sensor values.
#### `update_particle_weights_with_measurement_model()`
Given sensor measurements, we loop through each particle and calculate its weight based on the given formula used in class. All of the needed calculation parameters will be available through then sensor measurements and `s_front`, `s_left`, `s_back`, `s_right`. We save the weight to the dictionary, keyed as `weight` 
To test this, we will calculate one or two particle weights by hand and see if they match the weight our program outputs.
#### `normalize_particles()`
Since we have found the particles’ respective weights, we can calculate a normalization factor by doing 1 / (sum of particle weights) then multiplying this factor to each particle’s weight, keyed into the particle’s dictionary as `normalized_weight`. 
To test this, we will sum the weights of all the particles and ensure it equals 1. If not, this means that an error occurred, and we need to review our work to find its source.
#### `resample_particles()`
Given each particle’s `normalized_weight`, we feed the normalized_weights and each particle into numpy.random.choice() to sample as many particles as we need, saving the final result to `particle_cloud` 
To test this, we can create a plot with pre-resampling particles’ coordinates and one with post-resampling coordinates. These two plots should appear similar but the post-resampling plot should contain areas of increased particle density as particles begin to cluster together.
#### `update_estimated_robot_pose()`
Given all the new resampled particles, we find the location of the most dense particles and set that as our estimated robot pose. We can apply a kernel density estimation with a gaussian kernel to retrieve the densest point. This can be done through the scipy stats package. See [stack overflow](https://stackoverflow.com/questions/58559880/how-to-obtain-coordinates-of-maximum-density)
To test this, we can plot the resampled particle locations along with the densest point and check it by eye. 
#### Incorporating noise 
Given a set of resampled particles, we can apply extra Gaussian noise to each particle by sampling from a Gaussian distribution using some numpy function. We will need to tune the magnitude of how much noise to inject through trial and error. 
To test this, we can plot the particles before and after the addition of noise and see if the difference in value is expected given our noise parameters.

####Timeline
Working on Feb 2, 3, 9
Feb 2: Complete half the methods 
Feb 3: Complete the rest of the methods
Feb 9: Writeup, video, documentation etc. 

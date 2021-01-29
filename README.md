# particle_filter_project

## Implementation plan 
### `initialize_particle_cloud()`
Given the map dimensions of x and z, we use numpy.random to generate random coordinates for a particle as well as its orientation. We repeat this procedure for the number of particles desired, and save the results to an array `particle_cloud`, probably as a dictionary keyed by `x`, `z`, `dir`. We’ll first try our particle filter with a small number of particles to make sure our algorithm is correct. Later on we’ll scale up the number of particles based on performance.
After sampling the particle coordinates, we can look through the map data and calculate what the sensor values for each particle should be in the directions (front, left, back, right). We can save these values to the dictionary keyed as `s_front`, `s_left`, `s_back`, `s_right`
To test this, we will either visualize it using RViz or we will print out the drawn coordinates and ensure that they are within the map parameters (ie. x and y coordinates corresponding to a point within the map, sensor values align with the orientation etc.).
### `update_particles_with_motion_model()`
Given an input motion, we loop through the `particle_cloud` and increment each point’s x and z values, and orientation. If the motion is angular, we can just apply simple trigonometry to get the horizontal and vertical components and the new orientation.
Using the new x, z and orientation values, we can look through the map data and recalculate what the theoretical sensor values should be and update `s_front`, `s_left`, `s_back`, `s_right` 
To test this, we can visualize the “before” and “after” particle clouds and see if they are identical save for a shift by the expected motion magnitude. To test expected sensor values, we will attempt to calculate one or two sets of sensor values by hand and see if they match our program’s sensor values.
### `update_particle_weights_with_measurement_model()`
Given sensor measurements, we loop through each particle and calculate its weight based on the given formula used in class. All of the needed calculation parameters will be available through then sensor measurements and `s_front`, `s_left`, `s_back`, `s_right`. We save the weight to the dictionary, keyed as `weight` 
To test this, we will calculate one or two particle weights by hand and see if they match the weight our program outputs.
#### `normalize_particles()`
Since we have found the particles’ respective weights, we can calculate a normalization factor by doing 1 / (sum of particle weights) then multiplying this factor to each particle’s weight, keyed into the particle’s dictionary as `normalized_weight`. 
To test this, we will sum the weights of all the particles and ensure it equals 1. If not, this means that an error occurred, and we need to review our work to find its source.
#### `resample_particles()`
Given each particle’s `normalized_weight`, we feed the normalized_weights and each particle into numpy.random.choice() to sample as many particles as we need, saving the final result to `particle_cloud` 
To test this, we can create a plot with pre-resampling particles’ coordinates and one with post-resampling coordinates. These two plots should appear similar but the post-resampling plot should contain areas of increased particle density as particles begin to cluster together.
### `update_estimated_robot_pose()`
Given all the new resampled particles, we find the location of the most dense particles and set that as our estimated robot pose. We can apply a kernel density estimation with a gaussian kernel to retrieve the densest point. This can be done through the scipy stats package. See [stack overflow](https://stackoverflow.com/questions/58559880/how-to-obtain-coordinates-of-maximum-density)
To test this, we can plot the resampled particle locations along with the densest point and check it by eye. 
### Incorporating noise 
Given a set of resampled particles, we can apply extra Gaussian noise to each particle by sampling from a Gaussian distribution using some numpy function. We will need to tune the magnitude of how much noise to inject through trial and error. 
To test this, we can plot the particles before and after the addition of noise and see if the difference in value is expected given our noise parameters.

## Timeline
- Working on Feb 2, 3, 9
- Feb 2: Complete half the methods 
- Feb 3: Complete the rest of the methods
- Feb 9: Writeup, video, documentation etc. 


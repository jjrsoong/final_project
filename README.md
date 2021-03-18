##Writeup
Yves Shum, Joshua Soong

### `Program Run Instructions`
- After cloning the repo and doing catkin_make and all that, run the launch file by doing `roslaunch final_project pacturtle_world.launch`. You should see Gazebo boot up and have 5 turtles spawned in 4 corners of the map. Note: if there were errors during the launch, you might see the locations of each turtle and the map be distorted. If so, just close Gazebo and rerun the `roslaunch` command. 
- Once the world is ready, navigate to your catkin workspace and go to `/src/final_project/scripts`
- Run the scripts for each turtle in a new terminal by doing `rosrun final_project red_turtle.py` or for any other turtle 
- You should now see the turtles start to move 
### `Project Description`
Dubbed Pacturtle, our final project was inspired by Pacman. Our program features a user-operated Pacturtle whose goal is to evade the four ghost turtles for as long as possible within a specially designed maze world. By giving all the ghost turtles different search algorithms, we hoped to make the game as challenging as possible and, in this way, immerse the user in an interactive and fun game. In its current incarnation, all components of the project function smoothly but small bugs during program execution may arise whereby the ghosts navigate into walls or collide into each other (as designed in “Future Work” these are items that we hope to improve upon).
### `System Architecture`
#### Map 
- Map creation
  - We built a special map in Gazebo for this project. Key map features include: a) several open areas that can serve a robot spawn points, b) intentionally designed a world without dead ends so as not to confuse the ghost turtles, and c) the use of 1 meter wide hallways so robots could navigate without effort. The final_project.world file can be found in the map folder of our repository.
- Map to Adjacency list
  - This was done by redrawing the center of each cell into an online graphing tool, and connected each node (cell) to the ones that they were reachable to. The cells were numbered by row-major order, which made it easy to translate Gazebo world coordinates into the corresponding cell 

#### Launch file 
- Launch file `pacturtle_world.launch` composed of setting up the 5 turtlebots, loading up the world, launching the teleop, running a world controller python script `world_controller.py`, and some extra setup for rviz debugging. 
- We made our own copies of the turtlebot3 waffle pi xacro file in order to receive an input argument to specify what color the turtlebot model should be 
#### Pacturtle teleop
- Since each turtle has a prefix in their subscribed topics, we had to modify the Gazebo builtin teleop script to ensure that it talks to only the orange pacturtle. The only change that was made is the name of the publisher it was publishing
#### Red turtle 
- The wisest of all the ghost turtles, red turtle knows the layout of the map and the exact positions of both itself and Pacturtle at all points in time. Practically, this translated into giving red turtle access to the map adjacency list and subscribing it to /ModelStates, a topic that constantly published the positions of all five of the turtles. The exact x-y coordinates were distilled by our determine_cell() function into a value associated with the row-major order of our map and the adjacency list. From here, the robot queries the shortest_path() function with the current cell locations to determine the best way to follow Pacturtle. 
- It should be noted that shortest_path() relies on a breadth first search (BFS) algorithm which is executed during initialization on every single one of the 99 cells. Specifically, each iteration of the BFS algorithm creates a dictionary that associates each traversed cell with the cell that it came from (the “parent” cell). This dictionary was subsequently stored in a manner that shortest_path() could easily use.
- Robot movement is done using the move() function, which first uses orientation data to point the robot in the direction of the next cell and then commands it to move forward until it reaches the midpoint of the next cell. At this point, the robot will stop moving so that the program can recalculate the positions of redturtle and Pacturtle and, if needed, update the path that redturtle should take.
For visuals, see the “RedTurtle_v4” gif.
#### Yellow turtle 
- Yellow turtle has full map knowledge but does not know where Pacturtle is (there are various rumors about why this is, the most common of which is that he either did not do his homework properly or he was paying attention in tracking class).
- Yellow turtle utilizes the BFS infrastructure from red turtle to chart a path to follow. Notably, since Pacturtle’s location is unknown, we have inserted predetermined points of interest that yellow turtle paths toward. These points of interest are the three open areas on the map which we felt gave yellow turtle the best chance of sighting Pacturtle.
- If Pacturtle is seen, yellow turtle will exit its path following behavior and instead move toward Pacturtle. This is done by orienting itself using camera data to seek out Pacturtle’s distinctively orange skin color.
- If Pacturtle turns a corner and passes out of sight of yellow turtle, yellow turtle will revert to its path following behavior.
- For visuals, see the “YellowTurtle_Contact” and “YellowTurtle_NoContact” gifs.
#### Green turtle
- Green turtle does not have predetermined knowledge of the map, nor does it have any idea where pacturtle is. This turtle runs an autonomous map exploration algorithm that is a simplified version of what Kameyama, Naoki & Hidaka, Koichi. (2017) came up with. The fundamental idea is to be able to detect new branches and take the shortest one whenever possible. 
- All code on the Greenturtle’s activity can be found in `green_turtle.py`, where it extends the class Ghostturtle in `ghostturtle.py`. While Ghostturtle sets up the basic subscribers (like laser scan and camera), the Greenturtle algorithm is based on an action loop `action_loop()` that is dependent on the current state (`EXPLORE`, `HUNT`, `MOVE_TO_BRANCH`).
- Explore mode commands the turtle to move forward while avoiding obstacles (handled in `move_forward()` called by `action_loop()`, while checking if there are any branches that it can take in front.
- Hunt mode commands the turtle to chase Pacturtle down if it is within its sights. This activity is handled in `image_callback`
- Move to branch commands the turtle to move to a given branch coordinate (i.e. 2 units right, 1 unit up etc.). This activity in handled in `move_to_branch()`
- As a side note, the Green turtle relies on a `gps` for precise position as opposed to using Odometry as stated in the research paper. The paper was able to achieve precise Odometry by using SLAM to conduct odometry correction, which was too much of a stretch goal for us. Instead, we “cheated” a little and had the turtle receive its `ModelState` from Gazebo
- For visuals see `green_turtle_2.mp4` and `green_turtle_3.mp4`
#### Blue turtle 
- Blue turtle is the completely drunk turtle, relying on random movements to hopefully identify Pacturtle. 
- All of the Blueturtle code can be found in `blue_turtle.py`, where it also extends the class `Ghostturtle` in `ghostturtle.py`. 
- The way this turtle works is that it selects a random angular velocity every second, while maintaining a constant linear velocity. If the robot detects that it gets too close from a wall, it backs off from the wall and continues its drunk moving operations. This is handled in `move_forward()` 
- The moment Blue turtle sees Pacturtle in its camera, it switches to Hunting mode , which is handled by the `image_callback()` function 
- For visuals see `blue_turtle_1.mp4`
### `Challenges`
- Should’ve paid more attention to trigonometry during high school 
  - Getting the robot to turn in the z-axis depending on its current Pose, lidar scan data, and the target Pose was surprisingly difficult. We spent a lot of time debugging trigonometry issues where the turtlebot would not turn in the right angle towards its branch target. 
- Presence of noise 
  - Whether due to processing delay or sensor accuracy, many topics we subscribed to had noise (ie. /Modelstates, /command_vel). This presented a major problem because some of our algorithms were designed for exact measurements (ie. ideally red turtle would be able to execute perfect 90 degree turns so as to always move along the middle of the cell).   - To solve this, almost all our inputs were built with some margin for error. For example, if the robot overshot the margin of error for turning, it would stop forward progress to re-oriented itself.

### `Future Work`
- Smarter algorithms 
  - while out current algorithms work, implementing more complex algorithms would have been an interesting challenge and may have allowed our ghost turtles to more quickly converge on Pacturtle. This would provide added difficulty for the user (and, hopefully, make the game all the more addictive). Smarter algorithms could include, but are not limited to, ones that better filtered out noise or allowed the ghost turtles to talk to each other and box Pacturtle in.
- Collision avoidance 
  - currently, our ghost turtles do not perform collision avoidance when near each other. Consequently, they may run into each other. By implementing a hierarchy of collision avoidance whereby ghost turtles will yield to one another, we can prevent such scenarios from occurring.
### `Takeaways`
- Understood how Gazeo models work and how they are set up. 
  - Through this project we’ve understood how to work with xacro files, in particular for Gazebo and urdf models. 
  - We’re able to customize our models to look the way we want 
- Learned how to write our own complete ros package with launch files that can launch multiple turtlebots 
  - We were able to build a ros package from scratch, and have a successfully launch file and launches all of our assets at the same time 

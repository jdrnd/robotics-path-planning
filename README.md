## Robotics/Path Planning 
### [DRAFT]

This repository contains code I developed to illustrate various robotics and path planning concepts while taking MTE 544 (Autonomous Mobile Robotics) at the University of Waterloo in Winter 2020.

Everything in the repository is my own work.

For all images in this document, I used the map provided in this repo in `sim_map.pgm`. It has a scale of 1px=10cm, with the black areas representing obstacles.

![Simulation map](images/map.jpg)

Where robot location/trajectory was simulated, I used the "unicycle" model in which the robot's forward and angular speed can be controlled independently.

![robot model](images/model.jpg)

This corresponds well to the [turtlebot](https://www.turtlebot.com/) robots which are well-supported under ROS. For control however, practical limits on performance such as delays and actuator saturation were not modeled or simulated. If required, ROS has an excellent [Turtlebot simulation package](http://wiki.ros.org/turtlebot3_gazebo?distro=melodic).

#### Running the code
Each `.py` (Python 3.5+) file is independent. Numpy and matplotlib are the only requirements.  

### Potential Fields
This method generates artificial potential fields using map information. The goal is to produce an artificial potential field such that the starting point has the highest potential and the destination has the lowest, with high potentials surrounding any obstacles. Then, using gradient descent, an artificial force vector can be generated at each position, indicating the direction the robot should move. Think of this as analogous to a ball rolling down a hill, it always moves towards the bottom of the hill in the steepest direction.

Two different potential fields are used: the first is simply based on distance to the destination; points further away have higher potential. For this example we use a simple linear field. The second field is generated around obstacles to keep the robot away from them. We use a quadratic field, where the potential is proportional to the distance from the point to an obstacle divided by a distance cutoff, squared. The fields are then summed to produce an overall potential field used to determine the path of the robot. Tuning the weights of each field is required in order to ensure the overall field functions correctly.

![](images/potential_1.png)
![](images/potential_2.png)
![](images/potential_3.png)

A drawback of potential fields is that in the case on concave obstacles, the robot may be drawn to and stuck in a local minimum. Using potential fields as a local planner only in concert with a global path planner such as PRM or RRTs can help to alleviate this.

#### Resources
- [Motion Planning 3: Artificial Potential Fields](https://www.dis.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf`)
- [Robotic Motion Planning: Potential Functions](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)
- [Local Path Planning Using Virtual Potential Field](https://www.cs.mcgill.ca/~hsafad/robotics/)

### Probabilistic Roadmapping
TODO
#### Resources

### Rapidly-Expanding Random Trees
TODO
#### Resources

### Importance Sampling
TODO
#### Resources


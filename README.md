# robotics-final-project-ros

This is the final project of robotics course - Spring 2022. 

## About

This project has two exciting scenarios. 

## Scenario 1

In the first scenario, we have to generate a map of the world we are in, using a wall following algorithm to search the world and the gmapping package to generate such map. 

The wall following algorithm is made of two parts. First, the robot tries to reach a wall and then, it follows the wall. We used a pid controller for this scenario. The algorithms mentioned above are implemented in `follow_wall.py`.

## Scenario 2
In the second scenario, we have to reach a given destination using VFH obstacle avoidance algorithm (Borenstein, J. and Koren, Y., [1991](https://doi.org/10.1109/70.88137)). First we tried to use a pid controller for this step but it did not work out very well and kept hitting walls and getting stuck. So, we used a simpler stop, rotate and go controller which worked out better. The controller and the obstacle avoidance algorithm are implemented in `navigator.py`.

### Note
You can find the project instructions in the `project.pdf`. You can also find the launch and world files used in this project in the `maps` folder and use them in your project if you want. In order to do that, you have to place the `custom_world.launch` file in `$(find turtlebot3_gazebo)/launch/` path and the `funky-maze.world` file in `$(find turtlebot3_gazebo)/worlds/` path. 
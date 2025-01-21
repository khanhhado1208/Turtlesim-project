## Turtlesim Project
A simple ROS 2 project demonstrating how to control a turtle in the Turtlesim simulator using Python. This project moves the turtle to specific target points using the /cmd_vel topic and provides a demonstration of ROS 2 topics, publishing, and subscribing.

## Features
+ Moves the turtle to predefined target points in the Turtlesim simulator.
+ Uses the /turtle1/cmd_vel topic to send velocity commands.
+ Subscribes to /turtle1/pose to track the turtle's position.
+ Demonstrates basic ROS 2 concepts:
+ Publishing and subscribing to topics.
+ Controlling a simulated robot (Turtlesim).

## Run the Turtlesim Node
1. Start the Turtlesim simulator
   ros2 run turtlesim turtlesim_node
   
3. Run the Python node to control the turtle draws 1 line
   ros2 run turtlesim_catch_all turtle_controller_catch_points

5. Run the Python node to control the turtle draws more than 1 lines
   ros2 run turtlesim_catch_all turtle_controller_targetpoint

## Concepts
1. Topics
/cmd_vel:
The geometry_msgs/Twist message is published on this topic to control the turtle’s linear and angular velocity.
/turtle1/pose:
The turtlesim/Pose message is subscribed to for obtaining the turtle's position and orientation.

## Commands to Explore Topics
ros2 topic list

## Check the message type of a topic
ros2 topic info /turtle1/cmd_vel

## Echo topic data to see what is being published
ros2 topic echo /turtle1/pose



#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.target_x = 8.0
        self.target_y = 4.0

        self.pose_ = None
        self.cmd_vel_publisher = self.create_publisher( # Create a publisher to publish the Twist message
            Twist, 
            'turtle1/cmd_vel', 
            10)
        
        self.pose_subscriber = self.create_subscription( # Create a subscriber to subscribe to the Pose message
            Pose, 
            'turtle1/pose', 
            self.callback_turtle_pose, 10)
        
        self.controller_timer = self.create_timer(0.01, self.move_turtle) # Create a timer to control the turtle

        self.get_logger().info('Turtle Controller has been started') # Print the message to the console
        
    def callback_turtle_pose(self, msg):   # Define the callback function
        self.pose_ = msg

    def move_turtle(self): # Define the function to control the turtle
        if self.pose_ == None:
            return
        
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        msg = Twist()
        if distance > 0.5:
            # position
            msg.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = 6 * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # List of target points (x, y)
        self.target_points = [(2.0, 2.0), (6.0, 6.0), (2.0, 6.0)]
        self.current_target_index = 0  # Index of the current target point

        self.pose_ = None
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.callback_turtle_pose, 10)

        self.controller_timer = self.create_timer(0.01, self.move_turtle)

        self.get_logger().info('Turtle Controller has been started')

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def move_turtle(self):
        if self.pose_ is None:
            return

        # Check if all target points have been reached
        if self.current_target_index >= len(self.target_points):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info('All target points reached!')
            self.destroy_timer(self.controller_timer)  # Stop the timer
            return

        # Get the current target point
        target_x, target_y = self.target_points[self.current_target_index]

        # Calculate distance and angle to the target
        dist_x = target_x - self.pose_.x
        dist_y = target_y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        angle_to_target = math.atan2(dist_y, dist_x)

        # Calculate angular difference
        angular_diff = angle_to_target - self.pose_.theta
        if angular_diff > math.pi:
            angular_diff -= 2 * math.pi
        elif angular_diff < -math.pi:
            angular_diff += 2 * math.pi

        msg = Twist()

        if distance > 0.5:  # Move towards the target
            msg.linear.x = 1.0 * distance
            msg.angular.z = 4.0 * angular_diff
        else:
            # Move to the next target when close to the current one
            self.get_logger().info(f'Reached target point {self.current_target_index + 1}')
            self.current_target_index += 1

        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

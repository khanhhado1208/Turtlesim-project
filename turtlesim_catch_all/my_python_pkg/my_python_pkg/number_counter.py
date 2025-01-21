#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NumberCounter(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter_ = 0

        # Create a subscription to the 'number' topic
        self.number_subscriber = self.create_subscription(
            Int64, 'number', self.number_callback, 10
        )

        # Create a publisher for the 'number_total' topic
        self.number_publisher = self.create_publisher(Int64, 'number_total', 10)

        self.get_logger().info('NumberCounter has started')

    def number_callback(self, msg):
        # Increment the counter by the value received in the message
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_

        # Publish the updated counter value
        self.number_publisher.publish(new_msg)
        self.get_logger().info(f'Counter updated and published: {new_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

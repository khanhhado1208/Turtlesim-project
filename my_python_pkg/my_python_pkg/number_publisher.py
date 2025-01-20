import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.number_ = 2
        self.number_publisher = self.create_publisher(Int64, 'number', 10)
        self.timer = self.create_timer(1, self.publish_number)  
        self.get_logger().info('Number publisher has been started')

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

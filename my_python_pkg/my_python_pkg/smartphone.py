import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SmartphoneSubscriber(Node):

    def __init__(self):
        super().__init__('smartphone_subscriber')
        self.subscription = self.create_subscription(String,'robot_news', self.callback_robot_news, 10)
        self.get_logger().info('Smartphone Subscriber has been started')

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    smartphone_subscriber = SmartphoneSubscriber()
    rclpy.spin(smartphone_subscriber)
    smartphone_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
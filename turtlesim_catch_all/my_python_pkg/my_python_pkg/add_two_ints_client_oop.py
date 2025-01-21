import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntClientOOP(Node):

    def __init__(self):
        super().__init__('Add_Two_Int_Client_OOP')
        self.call_add_two_ints_server(10, 20)
        self.call_add_two_ints_server(7, 8)
        self.call_add_two_ints_server(47, 21)

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, 'add_two_ints')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting for the server...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.future_call_back, a=a, b=b))

    def future_call_back(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"Response: a = {a}, b = {b}, sum = {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntClientOOP()  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node('add_two_ints_client')

    # Create a service client
    client = node.create_client(AddTwoInts, 'add_two_ints')

    # Wait for the service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Service not available, waiting for the server...')

    # Prepare the request
    request = AddTwoInts.Request()
    request.a = 10
    request.b = 20

    # Call the service asynchronously
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # Handle the response
    try:
        response = future.result()
        node.get_logger().info(f"Response: a = {request.a}, b = {request.b}, sum = {response.sum}")
    except Exception as e:
        node.get_logger().error(f"Service call failed: {str(e)}")

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

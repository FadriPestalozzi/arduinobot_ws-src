import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts 
import sys # to access main argv

class SimpleServiceClient(Node):
    def __init__(self, a, b):  # self = constructor as member of class SimpleServiceClient
        super().__init__('simple_service_client') # constructor of parent class

        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')  # AddTwoInts = message interface type

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req_ = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b
        self.future_ = self.client_.call_async(self.req_)  # send req msg to server
        self.future_.add_done_callback(self.responseCallback)  # once respose received

    def responseCallback(self, future):  # self = define as member of this class
        self.get_logger().info('Service Result: %d' % future.result().sum)

def main():
    rclpy.init()

    if len(sys.argv) != 3: # default sys starts with 1 arg, need 2 more (a,b)
        print("wrong number of arguments! Usage: simple_service_client a b")
        return -1

    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
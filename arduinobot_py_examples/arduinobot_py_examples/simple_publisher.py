import rclpy
from rclpy.node import Node # Node class from node module in rclpy library
from std_msgs.msg import String

class SimplePublisher(Node):  # inherit from Node
    def __init__(self): # constructor
        super().__init__("simple_publisher") # get input from parent class
        self.pub_ = self.create_publisher(String, "chatter",10) # use String msg to interact with topic of name chatter and buffer 10, if not receiving msg fast enough
        self.counter_ = 0 # additional variable for publisher class, count msg
        self.frequency_ = 1.0 # [Hz] msg publishing
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

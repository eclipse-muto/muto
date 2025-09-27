import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MutoTalker(Node):
    def __init__(self):
        super().__init__('muto_talker')
        self.get_logger().info('MutoTalker node has been started.')
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('message', 'Hello, SDV Hackathon Chapter III!')
        
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(self.get_parameter('publish_frequency').get_parameter_value().double_value, self.timer_callback)
        self.ct = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.get_parameter('message').get_parameter_value().string_value + f' {self.ct}'
        self.publisher_.publish(msg)
        self.ct += 1
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    muto_talker = MutoTalker()
    rclpy.spin(muto_talker)
    muto_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

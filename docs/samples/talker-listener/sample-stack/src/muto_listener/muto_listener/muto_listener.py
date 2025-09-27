import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MutoListener(Node):
    def __init__(self):
        super().__init__('muto_listener')
        self.get_logger().info('MutoListener node has been started.')
        self.create_subscription(String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    muto_listener = MutoListener()
    rclpy.spin(muto_listener)
    muto_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

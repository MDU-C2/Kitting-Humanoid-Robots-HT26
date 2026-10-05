import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VisionTestNode(Node):

    def __init__(self):
        super().__init__('vision_test_node')

        self.publisher_ = self.create_publisher(String, 'vision_test', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from the G1 vision node!'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = VisionTestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
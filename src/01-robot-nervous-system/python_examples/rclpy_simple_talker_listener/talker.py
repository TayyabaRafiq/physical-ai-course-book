import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_talker')
    publisher = node.create_publisher(String, 'simple_topic', 10)
    msg = String()
    i = 0
    while rclpy.ok():
        msg.data = 'Simple Hello World: %d' % i
        publisher.publish(msg)
        node.get_logger().info('Publishing: "%s"' % msg.data)
        i += 1
        time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
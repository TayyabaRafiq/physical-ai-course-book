import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_listener')
    subscription = node.create_subscription(
        String,
        'simple_topic',
        lambda msg: node.get_logger().info('I heard: "%s"' % msg.data),
        10)
    subscription  # prevent unused variable warning
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
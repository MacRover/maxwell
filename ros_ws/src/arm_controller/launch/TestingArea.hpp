import rclpy
from rclpy.node import Node

class WristNode(Node):

    def __init__(self):
        super().__init__('wrist_node')
        self.subscription = self.create_subscription(
            # topic - probably just named ODOM, 
            # 'topic', 
            # self.listener_callback,
            # 10 - idk what our preferred queue size is
        )
        self.subscription #Can be deleted when used

def main (args=None):
    rclpy.init(args=args)

    wrist_node = WristNode()

    rclpy.spin(wrist_node)

    wrist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#drive controller in drive directory for odom publisher
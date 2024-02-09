import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from custom_interfaces.msg import ControllerMode

class Toggler(Node):
    def __init__(self):
        super().__init__("manager")
        self.pub = self.create_publisher(ControllerMode, "/mode", 10)
        self.create_subscription(Joy, "/joy", self._toggle_mode_callback, 10)

        self.active = False
        self.state = 0
        self.num = 2

    def _toggle_mode_callback(self, msg):
        if (msg.buttons[5] and not self.active):
            self.active = True
            self.state = (self.state + 1) % self.num
        elif (not msg.buttons[5]):
            self.active = False

        modes = ControllerMode(
            drivetrain=(self.state == 0),
            arm=(self.state == 1)
        )
        self.pub.publish(modes)


def main(args=None):
    rclpy.init(args=args)
    node = Toggler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from custom_interfaces.msg import ControllerMode

DRIVE_MODE = 0
ARM_MODE = 1

class Toggler(Node):
    def __init__(self):
        super().__init__("manager")
        self.drive = self.create_publisher(Joy, "/joy_drive", 10)
        self.arm = self.create_publisher(Joy, "/joy_arm", 10)
        self.create_subscription(Joy, "/joy", self._toggle_mode_callback, 10)

        self.active = False
        self.state = 0

    def _toggle_mode_callback(self, msg):
        if (msg.buttons[5] and not self.active):
            self.active = True
            self.state = (self.state + 1) % 2
        elif (not msg.buttons[5]):
            self.active = False

        if (self.state == DRIVE_MODE):
            self.drive.publish(msg)
        elif (self.state == ARM_MODE):
            self.arm.publish(msg)
        else:
            self.drive.publish(msg)


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

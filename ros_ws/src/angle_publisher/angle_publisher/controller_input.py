#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class DpadServoController(Node):
    def __init__(self):
        super().__init__("controller_angle_controller")
        self.publisher_servo1 = self.create_publisher(Float32, "servo1_angle", 10)
        self.publisher_servo2 = self.create_publisher(Float32, "servo2_angle", 10)
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.get_logger().info("Dpad Servo Controller Node Started")

    def joy_callback(self, msg):
       
        servo_msg1 = Float32()
        servo_msg2 = Float32()

        
        dpad_horizontal = msg.axes[6]  # Left: 1.0, Right: -1.0
        dpad_vertical = msg.axes[7]    # Up: 1.0, Down: -1.0

        # D-pad control logic
        if dpad_vertical == 1.0:  # D-pad up
            servo_msg1.data = 1.0
        elif dpad_vertical == -1.0:  # D-pad down
            servo_msg1.data = -1.0
        else:
            servo_msg1.data = 0.0

        if dpad_horizontal == 1.0:  # D-pad left
            servo_msg2.data = 1.0
        elif dpad_horizontal == -1.0:  # D-pad right
            servo_msg2.data = -1.0
        else:
            servo_msg2.data = 0.0

   
        self.publisher_servo1.publish(servo_msg1)
        self.publisher_servo2.publish(servo_msg2)

        self.get_logger().info(f"servo_1: {servo_msg1.data}, servo_2: {servo_msg2.data}")


def main():
    rclpy.init()
    try:
        dpad_controller = DpadServoController()
        rclpy.spin(dpad_controller)
    except KeyboardInterrupt:
        pass
    finally:
        dpad_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
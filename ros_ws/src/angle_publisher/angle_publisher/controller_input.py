#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class DpadServoController(Node):
    def __init__(self):
        super().__init__("dpad_servo_controller")
        self.publisher_servo1 = self.create_publisher(Float32, "/obc/servo1_angle", 10)
        self.publisher_servo2 = self.create_publisher(Float32, "/obc/servo2_angle", 10)
        self.publisher_servo3 = self.create_publisher(Float32, "/obc/servo3_angle", 10)

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.get_logger().info("Dpad Servo Controller Node Started")
        self.servo3_active = False 

        self.servo1_angle = 90.0  
        self.servo2_angle = 90.0  
        self.servo3_angle = 90.0  
        self.servo3_active = False
    def joy_callback(self, msg):
        servo_msg1 = Float32()
        servo_msg2 = Float32()
        servo_msg3 = Float32()

    def joy_callback(self, msg):
        # Read D-pad axes
        dpad_horizontal = msg.axes[6]  # Left: 1.0, Right: -1.0
        dpad_vertical = msg.axes[7]    # Up: 1.0, Down: -1.0

      
        if dpad_vertical == 1.0 and dpad_horizontal == 1.0:
            self.servo3_angle = min(self.servo3_angle + 5.0, 180.0)  
            self.servo3_active = True
        elif dpad_vertical == 1.0 and dpad_horizontal == -1.0:
            self.servo3_angle = max(self.servo3_angle - 5.0, 0.0)  
            self.servo3_active = True
        else:
            self.servo3_active = False

        if not self.servo3_active:
            if dpad_vertical == 1.0:  # D-pad up
                self.servo1_angle = min(self.servo1_angle + 5.0, 180.0)
            elif dpad_vertical == -1.0:  # D-pad down
                self.servo1_angle = max(self.servo1_angle - 5.0, 0.0)

            if dpad_horizontal == 1.0:  # D-pad left
                self.servo2_angle = min(self.servo2_angle + 5.0, 180.0)
            elif dpad_horizontal == -1.0:  # D-pad right
                self.servo2_angle = max(self.servo2_angle - 5.0, 0.0)

     
        self.publish_servo_angles()

    def publish_servo_angles(self):
      
        servo_msg1 = Float32()
        servo_msg2 = Float32()
        servo_msg3 = Float32()

        servo_msg1.data = self.servo1_angle
        servo_msg2.data = self.servo2_angle
        servo_msg3.data = self.servo3_angle

        self.publisher_servo1.publish(servo_msg1)
        self.publisher_servo2.publish(servo_msg2)
        self.publisher_servo3.publish(servo_msg3)

        self.get_logger().info(f"Servo1: {servo_msg1.data}, Servo2: {servo_msg2.data}, Servo3: {servo_msg3.data}")
        # D-pad control logic
        enabled = True
        while enabled:
            if dpad_vertical == 1.0 and dpad_horizontal == 1.0:
                servo_msg3.data +=5.0
                self.servo3_active = True
            elif dpad_vertical == 1.0 and dpad_horizontal == -1.0:
                servo_msg3.data -=5.0
                self.servo3_active = True
            else:
                self.servo3_active = False

            if not self.servo3_active:
                if dpad_vertical == 1.0:  # D-pad up
                    servo_msg1.data +=5.0
                elif dpad_vertical == -1.0:  # D-pad down
                    servo_msg1.data -=5.0
            

                if dpad_horizontal == 1.0:  # D-pad left
                    servo_msg2.data +=5.0
                elif dpad_horizontal == -1.0:  # D-pad right
                    servo_msg2.data -=5.0
                

    
            self.publisher_servo1.publish(servo_msg1)
            self.publisher_servo2.publish(servo_msg2)
            self.publisher_servo3.publish(servo_msg3)

            self.get_logger().info(f"servo_1: {servo_msg1.data}, servo_2: {servo_msg2.data}, servo_3: {servo_msg3.data}")


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

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped
from custom_interfaces.msg import RadStatus, CANstamped
from .VESC import VESC

class ArmBaseController(Node):
    def __init__(self):
        super().__init__('arm_base_controller')

        # Robot's integrated yaw rotation
        self.robot_yaw = 0.0

        # Latest known arm base angle (from joint0)
        self.current_arm_angle = 0.0

        # Timestamp of last twist update
        self.last_twist_time = None

        # Subscriber to robot twist
        self.sub_twist = self.create_subscription(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            self.twist_callback,
            10
        )

        # Subscriber to current arm base joint angle
        self.sub_status = self.create_subscription(
            RadStatus,
            '/arm/joint0/rad_status',
            self.status_callback,
            10
        )

        # CAN publisher to send command to base
        self.pub = self.create_publisher(CANstamped, '/can/can_out_queue', 10)

        # Initialize VESC controller for arm base 
        #self.vesc_base = VESC(vesc_id=0x05) 

        # Timer to continuously update
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

    def twist_callback(self, msg: TwistStamped):
        now = self.get_clock().now()
        if self.last_twist_time is None:
            self.last_twist_time = now
            return

        dt = (now - self.last_twist_time).nanoseconds / 1e9
        self.last_twist_time = now

        # Accumulate robot yaw (rotation around y)
        linear_y = msg.twist.linear.y
        self.robot_yaw += linear_y * dt

    def status_callback(self, msg: RadStatus):
        self.current_arm_angle = msg.position  # Assuming radians

    def control_loop(self):
        # Calculate target arm base angle to cancel out robot rotation
        desired_angle = -self.robot_yaw  # Cancels out robot yaw

        # Wrap into CAN message
        can_msg = CANstamped()
        can_msg.stamp = self.get_clock().now().to_msg()

        self.pub.publish(can_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

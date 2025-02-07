import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from foxglove_msgs.msg import CompressedVideo
from builtin_interfaces.msg import Time  # Import the Time message type

class CompressedImageToCompressedVideo(Node):
    def __init__(self):
        super().__init__('compressed_image_to_compressed_video')
        
        # Subscriber to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_compressed',  # Input topic
            self.image_callback,
            10  # QoS depth
        )

        # Publisher to compressed video topic
        self.publisher = self.create_publisher(
            CompressedVideo,
            '/video_compressed',  # Output topic
            10  # QoS depth
        )

        self.get_logger().info("Compressed Image to Compressed Video Node Started")

    def image_callback(self, msg: CompressedImage):
        # Convert CompressedImage to CompressedVideo by copying data
        video_msg = CompressedVideo()
        video_msg.format = msg.format  # Keep original format (e.g., "h264")
        video_msg.data = msg.data  # Copy the raw compressed data
        
        # Ensure correct timestamps using a Time object
        video_msg.timestamp = msg.header.stamp  # Directly assign the ROS 2 Time message

        # Publish the converted message
        self.publisher.publish(video_msg)
        self.get_logger().info("Published CompressedVideo frame")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageToCompressedVideo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

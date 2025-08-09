#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Int32
from sensor_msgs.msg import NavSatFix
import csv
import threading
import time
from datetime import datetime

class TopicLogger(Node):
    def __init__(self):
        super().__init__('topic_logger')
        
        # Configuration - adjust these as needed
        self.gps_topic = '/gps'
        self.hydrogen_topic = '/hydrogen'
        self.o3_topic = '/o3'
        self.geiger_topic = '/geiger'
        self.csv_filename = 'sensor_data_' + str(datetime.now().strftime('%H_%M')) + '.csv'
        self.log_rate_hz = 1.0  # Adjustable logging rate
        
        # Storage for most recent values
        self.latest_data = {
            'latitude': None,
            'longitude': None,
            'altitude': None,
            'hydrogen': None,
            'o3': None,
            'geiger': None,
        }
        
        # Create subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10
        )
        
        self.hydrogen_sub = self.create_subscription(
            Float32,
            self.hydrogen_topic,
            lambda msg: self.update_data('hydrogen', msg.data),
            10
        )
        
        self.o3_sub = self.create_subscription(
            Int16,
            self.o3_topic,
            lambda msg: self.update_data('o3', msg.data),
            10
        )
        
        self.geiger_sub = self.create_subscription(
            Int32,
            self.geiger_topic,
            lambda msg: self.update_data('geiger', msg.data),
            10
        )
        
        # Initialize CSV file with headers
        self.init_csv()
        
        # Start logging thread
        self.logging_thread = threading.Thread(target=self.log_data, daemon=True)
        self.logging_thread.start()
        
        self.get_logger().info(f'Topic logger started. Logging to {self.csv_filename} at {self.log_rate_hz} Hz')
    
    def gps_callback(self, msg):
        """Handle GPS data - extract latitude, longitude, altitude"""
        self.latest_data['latitude'] = msg.latitude
        self.latest_data['longitude'] = msg.longitude
        self.latest_data['altitude'] = msg.altitude
    
    def update_data(self, topic_key, value):
        """Update the latest data for a topic"""
        self.latest_data[topic_key] = value
    
    def init_csv(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'hydrogen', 'o3', 'geiger'])
    
    def log_data(self):
        """Logging thread function - runs at specified rate"""
        while rclpy.ok():
            try:
                # Get current timestamp
                current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
                # Write current data to CSV
                with open(self.csv_filename, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([
                        current_time,
                        self.latest_data['latitude'],
                        self.latest_data['longitude'],
                        self.latest_data['altitude'],
                        self.latest_data['hydrogen'],
                        self.latest_data['o3'],
                        self.latest_data['geiger']
                    ])
                
                # Sleep for the specified interval
                time.sleep(1.0 / self.log_rate_hz)
                
            except Exception as e:
                self.get_logger().error(f'Error logging data: {e}')
                time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        logger = TopicLogger()
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
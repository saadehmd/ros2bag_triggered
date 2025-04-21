import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import random
import argparse

class NavSatFixPublisher(Node):

    def __init__(self, valid_fix, inside_zone):
        super().__init__('navsatfix_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/robot/fix', 10)
        self.valid_fix = valid_fix
        self.inside_zone = inside_zone
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = NavSatFix()
        # Fill in the NavSatFix message fields here
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        if self.inside_zone:
                # Generate random data within the specified range
                msg.latitude = random.uniform(34.20, 34.50)
                msg.longitude = random.uniform(122.20, 122.50)
                msg.altitude = random.uniform(20.0, 90.0)
        else:
            # Generate random data outside the specified range
            msg.latitude = random.choice([random.uniform(-90.0, 34.20), random.uniform(34.50, 90.0)])
            msg.longitude = random.choice([random.uniform(-180.0, 122.20), random.uniform(122.50, 180.0)])
            msg.altitude = random.choice([random.uniform(-1000.0, 20.0), random.uniform(90.0, 10000.0)])

        if self.valid_fix:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_UNKNOWN

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    parser = argparse.ArgumentParser(description='NavSatFix Publisher')
    parser.add_argument('--valid_fix', action='store_true', help='Generate valid GPS fix')
    parser.add_argument('--inside_zone', action='store_true', help='Generate GPS fix within the specified range')
    args = parser.parse_args()

    rclpy.init()
    navsatfix_publisher = NavSatFixPublisher(valid_fix=args.valid_fix, inside_zone=args.inside_zone)
    rclpy.spin(navsatfix_publisher)
    navsatfix_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

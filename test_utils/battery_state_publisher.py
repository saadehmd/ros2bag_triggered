import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import argparse

class BatteryStatePublisher(Node):
    def __init__(self, battery_health):
        super().__init__('battery_state_publisher')
        self.publisher_ = self.create_publisher(BatteryState, '/robot/battery_state', 10)
        self.battery_health = battery_health
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_health = self.battery_health
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    parser = argparse.ArgumentParser(description='Battery State Publisher')
    parser.add_argument('--battery_health', type=int, default=BatteryState.POWER_SUPPLY_HEALTH_DEAD,
                        help='Battery health status')
    args = parser.parse_args()

    rclpy.init()
    battery_state_publisher = BatteryStatePublisher(args.battery_health)
    rclpy.spin(battery_state_publisher)
    battery_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
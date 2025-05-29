from . import DummyBatteryHealthTrigger#, DummyTriggerUI

def main():
    rclpy.init(args=None)
    battery_trigger = DummyBatteryHealthTrigger()
    ui = DummyTriggerUI([battery_trigger])
    ui.run()
    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
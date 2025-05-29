from .dummy_trigger_display import DummyTrigger
from sensor_msgs.msg import BatteryState

class DummyBatteryHealthTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/battery_state'):

        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=BatteryState,
            trigger_name="Battery Health Trigger",
            
        )
        self.clock = node.get_clock()

    def get_positive_msg(self):
        msg = BatteryState()
        msg.header.stamp = self.clock.now().to_msg()
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        return msg
    
    def get_negative_msg(self):
        msg = BatteryState()
        msg.header.stamp = self.clock.now().to_msg()
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        return msg

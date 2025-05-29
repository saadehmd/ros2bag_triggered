from .dummy_trigger_display import DummyTrigger
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

class DummyNavSatInvalidFixTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/navsat_fix'):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=NavSatFix,
            trigger_name="NavSat Invalid Fix Trigger",
        )
        self.clock = node.get_clock()

    def get_positive_msg(self):
        msg = NavSatFix()
        msg.header.stamp = self.clock.now().to_msg()
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_UNKNOWN
        return msg
    
    def get_negative_msg(self):
        msg = NavSatFix()
        msg.header.stamp = self.clock.now().to_msg()
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        return msg
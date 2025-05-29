from .dummy_trigger_display import DummyTrigger
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

class DummyNavSatZoneTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/navsat_fix', latitude_range=(-90.0, 90.0), longitude_range=(-180.0, 180.0), altitude_range=(0, 100)):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=NavSatFix,
            trigger_name="NavSat Zone Trigger",
        )
        self.clock = node.get_clock()
        self.latitude_range = latitude_range
        self.longitude_range = longitude_range
        self.altitude_range = altitude_range

    def get_positive_msg(self):
        msg = NavSatFix()
        msg.header.stamp = self.clock.now().to_msg()
        msg.latitude = (self.latitude_range[0] + self.latitude_range[1]) / 2
        msg.longitude = (self.longitude_range[0] + self.longitude_range[1]) / 2
        msg.altitude = (self.altitude_range[0] + self.altitude_range[1]) / 2
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        return msg
    
    def get_negative_msg(self):
        msg = NavSatFix()
        msg.header.stamp = self.clock.now().to_msg()
        msg.latitude = self.latitude_range[0] - 1.0
        msg.longitude = self.longitude_range[0] - 1.0
        msg.altitude = self.altitude_range[0] - 1.0
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        return msg
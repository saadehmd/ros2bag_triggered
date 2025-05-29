from .dummy_trigger_display import DummyTrigger
from geometry_msgs.msg import PoseStamped

class DummyPoseStampedZoneTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/pose', position_range=((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=PoseStamped,
            trigger_name="Pose Stamped Zone Trigger",
        )
        self.position_range = position_range
        self.clock = node.get_clock()

    def get_positive_msg(self):
        msg = PoseStamped()
        msg.header.stamp = self.clock.now().to_msg()
        msg.pose.position.x = (self.position_range[0][0] + self.position_range[1][0]) / 2
        msg.pose.position.y = (self.position_range[0][1] + self.position_range[1][1]) / 2
        msg.pose.position.z = (self.position_range[0][2] + self.position_range[1][2]) / 2
        return msg

    def get_negative_msg(self):
        msg = PoseStamped()
        msg.header.stamp = self.clock.now().to_msg()
        msg.pose.position.x = self.position_range[0][0] - 1.0
        msg.pose.position.y = self.position_range[0][1] - 1.0
        msg.pose.position.z = self.position_range[0][2] - 1.0
        return msg
from .dummy_trigger_display import DummyTrigger
from nav_msgs.msg import Odometry

class DummyVelocityTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/odometry', linear_velocity_threshold=0.5, angular_velocity_threshold=0.5):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=Odometry,
            trigger_name="Velocity Trigger",
        )
        self.clock = node.get_clock()
        self.linear_velocity_threshold = linear_velocity_threshold
        self.angular_velocity_threshold = angular_velocity_threshold

    def get_positive_msg(self):
        msg = Odometry()
        msg.header.stamp = self.clock.now().to_msg()
        msg.twist.twist.linear.x = self.linear_velocity_threshold + 0.1
        msg.twist.twist.linear.y = self.linear_velocity_threshold + 0.1
        msg.twist.twist.angular.z = self.angular_velocity_threshold + 0.1
        return msg

    def get_negative_msg(self):
        msg = Odometry()
        msg.header.stamp = self.clock.now().to_msg()
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0
        return msg
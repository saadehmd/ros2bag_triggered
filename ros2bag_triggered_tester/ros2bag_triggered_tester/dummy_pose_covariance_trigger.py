from .dummy_trigger_display import DummyTrigger
from nav_msgs.msg import Odometry

class DummyPoseCovarianceTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/odometry', position_covariance_threshold=0.01, orientation_covariance_threshold=0.01):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=Odometry,
            trigger_name="Pose Covariance Trigger",
        )
        self.clock = node.get_clock()
        self.position_covariance_threshold = position_covariance_threshold
        self.orientation_covariance_threshold = orientation_covariance_threshold

    def get_positive_msg(self):
        msg = Odometry()
        msg.header.stamp = self.clock.now().to_msg()
        #msg.pose.covariance = [self.position_covariance_threshold] 
        msg.pose.covariance[0] = 2 * self.position_covariance_threshold 
        msg.pose.covariance[7] = 2 * self.position_covariance_threshold
        msg.pose.covariance[14] = 2 * self.position_covariance_threshold
        msg.pose.covariance[21] = 2 * self.orientation_covariance_threshold
        msg.pose.covariance[28] = 2 * self.orientation_covariance_threshold
        msg.pose.covariance[35] = 2 * self.orientation_covariance_threshold

        return msg
    
    def get_negative_msg(self):
        msg = Odometry()
        msg.header.stamp = self.clock.now().to_msg()
        msg.pose.covariance = [0.0] * 36  
        return msg
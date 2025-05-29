from .dummy_trigger_display import DummyTrigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DummyJointEffortTrigger(DummyTrigger):
    def __init__(self, node, topic_name='/robot/joint_trajectory', joint_effort_threshold=0.5):
        super().__init__(
            node=node,
            topic_name=topic_name,
            msg_type=JointTrajectory,
            trigger_name="Joint Effort Trigger",
        )
        self.clock = node.get_clock()
        self.joint_effort_threshold = joint_effort_threshold

    def get_positive_msg(self):
        msg = JointTrajectory()
        msg.header.stamp = self.clock.now().to_msg()
        point = JointTrajectoryPoint()
        point.effort = [self.joint_effort_threshold + 0.1]  # Positive effort
        msg.points.append(point)
        return msg

    def get_negative_msg(self):
        msg = JointTrajectory()
        msg.header.stamp = self.clock.now().to_msg()
        point = JointTrajectoryPoint()
        point.effort = [self.joint_effort_threshold - 0.1]  # Negative effort
        msg.points.append(point)
        return msg
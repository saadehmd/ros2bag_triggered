import rclpy
from ros2bag_triggered_tester.dummy_battery_health_trigger import DummyBatteryHealthTrigger
from ros2bag_triggered_tester.dummy_navsat_fix_zone_trigger import DummyNavSatZoneTrigger
from ros2bag_triggered_tester.dummy_pose_stamped_zone_trigger import DummyPoseStampedZoneTrigger
from ros2bag_triggered_tester.dummy_navsat_invalid_fix_trigger import DummyNavSatInvalidFixTrigger
from ros2bag_triggered_tester.dummy_velocity_trigger import DummyVelocityTrigger
from ros2bag_triggered_tester.dummy_joint_effort_trigger import DummyJointEffortTrigger 
from ros2bag_triggered_tester.dummy_pose_covariance_trigger import DummyPoseCovarianceTrigger
from ros2bag_triggered_tester.dummy_trigger_display import DummyTriggerUI

def main():
    rclpy.init(args=None)
    
    ui = DummyTriggerUI()
    battery_trigger = DummyBatteryHealthTrigger(node=ui, topic_name='/robot/battery_state')
    navsat_invalid_fix_trigger = DummyNavSatInvalidFixTrigger(node=ui, topic_name='/robot/fix')
    navsat_zone_trigger = DummyNavSatZoneTrigger(node=ui,
                                                   topic_name='/robot/fix', 
                                                   latitude_range=(0, 90), 
                                                   longitude_range=(100, 120), 
                                                   altitude_range=(10, 100))
    velocity_trigger = DummyVelocityTrigger(node=ui, 
                                            topic_name='/robot/odometry', 
                                            linear_velocity_threshold=1.0,
                                            angular_velocity_threshold=0.1)
    pose_stamped_zone_trigger = DummyPoseStampedZoneTrigger(node=ui, 
                                                         topic_name='/robot/pose', 
                                                         position_range=((0.0, 0.0, 0.0), (10.0, 5.0, 100.0)))

    joint_effort_trigger = DummyJointEffortTrigger(node=ui, topic_name='/robot/joint_trajectories', joint_effort_threshold=9.8)
    pose_covariance_trigger = DummyPoseCovarianceTrigger(node=ui, topic_name='/robot/odometry', position_covariance_threshold=0.01, orientation_covariance_threshold=0.001)

    ui.add_triggers([battery_trigger, navsat_invalid_fix_trigger, navsat_zone_trigger, pose_stamped_zone_trigger, velocity_trigger, joint_effort_trigger, pose_covariance_trigger])
    ui.run()
    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

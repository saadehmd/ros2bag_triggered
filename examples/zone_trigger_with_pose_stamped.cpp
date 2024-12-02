#include <examples/zone_trigger_with_pose_stamped.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithPoseStamped::isTriggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    return msg->pose.position.x >= trigger_zone_.x_min && msg->pose.position.x <= trigger_zone_.x_max &&
           msg->pose.position.y >= trigger_zone_.y_min && msg->pose.position.y <= trigger_zone_.y_max &&
           msg->pose.position.z >= trigger_zone_.z_min && msg->pose.position.z <= trigger_zone_.z_max;
}

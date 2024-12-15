#include <examples/zone_trigger_with_pose_stamped.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithPoseStamped::isTriggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
    return msg->pose.position.x >= trigger_zone_.x_min && msg->pose.position.x <= trigger_zone_.x_max &&
           msg->pose.position.y >= trigger_zone_.y_min && msg->pose.position.y <= trigger_zone_.y_max &&
           msg->pose.position.z >= trigger_zone_.z_min && msg->pose.position.z <= trigger_zone_.z_max;
}

void ZoneTriggerWithPoseStamped::fromYaml(const YAML::Node& node)
{   
    // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions 
    // on problems, parsing the configuration from yaml. Including missing key-value pairs or wrong access-types.
    try
    {
        persistance_duration_ =  rclcpp::Duration::from_seconds(node["persistance_duration"].as<uint64_t>());
        use_msg_stamp_ = node["use_msg_stamp"].as<bool>();
        trigger_zone_.x_min = node["trigger_zone"]["x_min"].as<double>();
        trigger_zone_.x_max = node["trigger_zone"]["x_max"].as<double>();
        trigger_zone_.y_min = node["trigger_zone"]["y_min"].as<double>();
        trigger_zone_.y_max = node["trigger_zone"]["y_max"].as<double>();
        trigger_zone_.z_min = node["trigger_zone"]["z_min"].as<double>();
        trigger_zone_.z_max = node["trigger_zone"]["z_max"].as<double>();
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << "Exception in parsing "<< name << " from YAML: " << e.what() << '\n';
    }
    
}
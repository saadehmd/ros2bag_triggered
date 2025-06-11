#ifndef ROS2BAG_TRIGGERED_TYPE_TRAITS_HPP
#define ROS2BAG_TRIGGERED_TYPE_TRAITS_HPP

#include <std_msgs/msg/header.hpp>
#include <type_traits>

namespace ros2bag_triggered
{

namespace type_traits
{

// Trait to check if a message has a `header` field of type `std_msgs::msg::Header`
template <typename T, typename = void>
struct HasHeader : std::false_type
{
};

// Specialization: Checks if `T` has a member named `header` of type `std_msgs::msg::Header`
template <typename T>
struct HasHeader<T, std::void_t<decltype(std::declval<T>().header)>>
: std::is_same<decltype(std::declval<T>().header), std_msgs::msg::Header>
{
};

// Check if a type is a ROS 2 IDL message
template <typename T, typename = void>
struct IsRosIdlType : std::false_type
{
};

// Specialize for valid ROS 2 message types using `rosidl_generator_traits`
template <typename T>
struct IsRosIdlType<T, std::void_t<typename rosidl_generator_traits::is_message<T>::type>>
: std::true_type
{
};

}  // namespace type_traits

}  // namespace ros2bag_triggered

#endif  // ROS2BAG_TRIGGERED_TYPE_TRAITS_HPP

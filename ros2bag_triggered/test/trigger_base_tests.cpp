#include <gtest/gtest.h>

#include <test/test_helpers.hpp>

constexpr double match_threshold{1e-4};  // seconds

void surge_tests(
  ros2bag_triggered::tests::EmptyTrigger & triggers, double persistance_duration,
  size_t no_of_surges, bool use_null_termination = false, bool should_trigger = false)
{
  triggers.reset();
  auto pos_msg = std::make_shared<std_msgs::msg::Bool>();
  pos_msg->data = true;
  auto neg_msg = std::make_shared<std_msgs::msg::Bool>();
  neg_msg->data = false;
  if (use_null_termination)
  {
    neg_msg = nullptr;
  }

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  const float buffer_duration{
    0.05};  // This is added so that the test loop doesn't run slighlty short of the required persistance duration.
  const auto trigger_duration = persistance_duration + buffer_duration;

  for (size_t i = 0; i < no_of_surges; i++)
  {
    const auto surge_start = clock->now();
    while ((clock->now() - surge_start).seconds() < trigger_duration)
    {
      EXPECT_FALSE(triggers.on_surge(pos_msg));
    }
    const auto surge_end = clock->now();

    if (should_trigger)
    {
      EXPECT_TRUE(triggers.on_surge(neg_msg));
      const auto last_trigger = triggers.get_trigger_pulses().back();
      EXPECT_EQ(triggers.get_trigger_pulses().size(), i + 1);
      EXPECT_NEAR(
        rclcpp::Duration::from_nanoseconds(last_trigger.start_time_).seconds(),
        surge_start.seconds(), match_threshold);
      EXPECT_NEAR(
        surge_end.seconds(), rclcpp::Duration::from_nanoseconds(last_trigger.end_time_).seconds(),
        match_threshold);
      EXPECT_NEAR(
        rclcpp::Duration::from_nanoseconds(last_trigger.end_time_ - last_trigger.start_time_)
          .seconds(),
        trigger_duration, match_threshold);
    } else
    {
      EXPECT_FALSE(triggers.on_surge(neg_msg));
      EXPECT_EQ(triggers.get_trigger_pulses().size(), 0);
    }
  }
}

TEST(HeaderLessTriggerTests, test_time_source_initialization)
{
  auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HeaderlessTimeSourceTest"));
  EXPECT_THROW(
    ros2bag_triggered::tests::EmptyTrigger(1, nullptr, logger, /*use_time_stamp=*/false),
    std::runtime_error);
  EXPECT_THROW(
    ros2bag_triggered::tests::EmptyTrigger(1, nullptr, logger, /*use_time_stamp=*/true),
    std::runtime_error);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_NO_THROW(
    ros2bag_triggered::tests::EmptyTrigger(1, clock, logger, /*use_time_stamp=*/false));
  EXPECT_NO_THROW(
    ros2bag_triggered::tests::EmptyTrigger(1, clock, logger, /*use_time_stamp=*/true));
}

TEST(TiggerWithHeaderTests, test_time_source_initialization)
{
  auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HeaderTimeSourceTest"));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_THROW(
    ros2bag_triggered::tests::BatteryHealthTrigger(1, nullptr, logger, /*use_time_stamp=*/false),
    std::runtime_error);
  EXPECT_NO_THROW(
    ros2bag_triggered::tests::BatteryHealthTrigger(1, nullptr, logger, /*use_time_stamp=*/true));
  EXPECT_NO_THROW(
    ros2bag_triggered::tests::BatteryHealthTrigger(1, clock, logger, /*use_time_stamp=*/false));
}

TEST(TiggerWithHeaderTests, test_trigger_stamps)
{
  auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("StampsTest"));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  const float persistance_duration{0.25};
  const float buffer_duration{
    0.05};  // This is added so that the test loop doesn't run slighlty short of the required persistance duration.

  auto triggers_with_stamps = ros2bag_triggered::tests::BatteryHealthTrigger(
    persistance_duration, clock, logger, /*use_time_stamp=*/true);
  auto triggers_with_clock = ros2bag_triggered::tests::BatteryHealthTrigger(
    persistance_duration, clock, logger, /*use_time_stamp=*/false);

  triggers_with_stamps.set_enabled(true);
  triggers_with_clock.set_enabled(true);

  const auto time_offset = rclcpp::Duration::from_seconds(10.0);
  const auto surge_start = clock->now();
  std::shared_ptr<rclcpp::Time> clock_start;
  while ((clock->now() - surge_start).seconds() < persistance_duration + buffer_duration)
  {
    auto pos_msg = std::make_shared<sensor_msgs::msg::BatteryState>();
    pos_msg->header.stamp = clock->now() + time_offset;
    pos_msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;

    EXPECT_FALSE(triggers_with_stamps.on_surge(pos_msg));

    if (!clock_start)
    {
      clock_start = std::make_shared<rclcpp::Time>(clock->now());
    }
    EXPECT_FALSE(triggers_with_clock.on_surge(pos_msg));
  }

  const auto surge_end = clock->now();
  auto neg_msg = std::make_shared<sensor_msgs::msg::BatteryState>();
  neg_msg->header.stamp = surge_end + time_offset;
  neg_msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

  EXPECT_TRUE(triggers_with_stamps.on_surge(neg_msg));

  auto clock_end = clock->now();
  EXPECT_TRUE(triggers_with_clock.on_surge(neg_msg));

  const auto last_trigger_with_stamps = triggers_with_stamps.get_trigger_pulses().back();
  EXPECT_NEAR(
    rclcpp::Duration::from_nanoseconds(last_trigger_with_stamps.start_time_).seconds(),
    (surge_start + time_offset).seconds(), match_threshold);
  EXPECT_NEAR(
    rclcpp::Duration::from_nanoseconds(last_trigger_with_stamps.end_time_).seconds(),
    (surge_end + time_offset).seconds(), match_threshold);

  const auto last_trigger_with_clock = triggers_with_clock.get_trigger_pulses().back();
  EXPECT_NEAR(
    rclcpp::Duration::from_nanoseconds(last_trigger_with_clock.start_time_).seconds(),
    clock_start->seconds(), match_threshold);
  EXPECT_NEAR(
    rclcpp::Duration::from_nanoseconds(last_trigger_with_clock.end_time_).seconds(),
    clock_end.seconds(), match_threshold);
}

TEST(AllTriggerTests, on_surge_test)
{
  auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("on_surge_test"));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // No persistance case.
  auto triggers_unpersisted =
    ros2bag_triggered::tests::EmptyTrigger(0, clock, logger, /*use_time_stamp=*/false);
  triggers_unpersisted.set_enabled(true);
  surge_tests(
    triggers_unpersisted, /*trigger_duration=*/0, /*no_of_surges=*/10,
    /*use_null_termination=*/false, /*should_trigger=*/true);
  surge_tests(
    triggers_unpersisted, /*trigger_duration=*/0, /*no_of_surges=*/10,
    /*use_null_termination=*/true, /*should_trigger=*/true);  //Test abort surge

  // Persistance case.
  const double trigger_duration = 0.25;
  auto triggers_persisted = ros2bag_triggered::tests::EmptyTrigger(
    trigger_duration, clock, logger, /*use_time_stamp=*/false);
  triggers_persisted.set_enabled(true);

  // Test false triggers with the smaller duration.
  const auto test_durations = std::vector<double>{0.05, 0.15, 0.25};
  for (const auto & duration : test_durations)
  {
    surge_tests(
      triggers_persisted, /*trigger_duration=*/duration, /*no_of_surges=*/3,
      /*use_null_termination=*/false, /*should_trigger=*/duration == trigger_duration);
    surge_tests(
      triggers_persisted, /*trigger_duration=*/duration, /*no_of_surges=*/3,
      /*use_null_termination=*/true,
      /*should_trigger=*/duration == trigger_duration);  //Test abort surge
  }
}

TEST(AllTriggerTests, test_enabled)
{
  auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("EnabledTest"));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto triggers =
    ros2bag_triggered::tests::EmptyTrigger(0.1, clock, logger, /*use_time_stamp=*/true);
  surge_tests(
    triggers, /*trigger_duration=*/0.1, /*no_of_surges=*/3, /*use_null_termination=*/false,
    /*should_trigger=*/false);
  triggers.set_enabled(true);
  surge_tests(
    triggers, /*trigger_duration=*/0.1, /*no_of_surges=*/3, /*use_null_termination=*/false,
    /*should_trigger=*/true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
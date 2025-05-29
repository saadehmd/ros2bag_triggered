#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <test/test_helpers.hpp>
#include <gtest/gtest.h>

using namespace ros2bag_triggered;

using TriggerVariant = std::variant<std::monostate, tests::EmptyTrigger, tests::BatteryHealthTrigger>;

class TriggeredRecorderNodeTestHelper : public TriggeredRecorderNode<TriggerVariant>
{
  public:
    TriggeredRecorderNodeTestHelper(std::string&& node_name = "triggered_recorder_node_test_helper", 
                                    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(), 
                                    const std::string& config_dir = "")
        : TriggeredRecorderNode<TriggerVariant>(std::move(node_name), node_options, config_dir)
    {
        battery_pub = create_publisher<sensor_msgs::msg::BatteryState>("robot/battery_state", 1);
        bool_pub = create_publisher<std_msgs::msg::Bool>("empty_msg", 1);
    }

    std::unordered_map<std::string, TriggerVariant> getTriggers() const
    {
        return triggers_;
    }
    size_t getNumberOfSubscriptions() const
    {
       return triggered_subscriptions_.size() + untriggered_subscriptions_.size();
    }

    YAML::Node getTopicsConfig() const
    {
        return topics_config_;
    }

    bool isWriterInitialized()
    {
        auto& triggered_writer = get_writer_impl();
        auto storage_opts = triggered_writer.get_storage_options();
        return triggered_writer.is_open() && !triggered_writer.get_bag_name().empty() && !storage_opts.uri.empty();
    }

    size_t getRecordedTopicCount()
    {
        auto& triggered_writer = get_writer_impl();
        return triggered_writer.get_topic_count();
    }

    bool isTriggerBufferTimmerInitialized() const
    {
        return trigger_buffer_timer_ != nullptr;
    }

    std::pair<int64_t, int64_t> getCropPoints() const
    {
        return crop_points_;
    }

    void testInit(size_t expected_topics_recorded, size_t expected_subscriptions, std::vector<std::string> expected_trigger_types)
    {
        
        EXPECT_EQ(getRecordedTopicCount(), expected_topics_recorded);
        EXPECT_EQ(triggers_.size(), std::variant_size<TriggerVariant>::value - 1);
        EXPECT_EQ(getNumberOfSubscriptions(), expected_subscriptions);
        
        for (const auto& trigger_type : expected_trigger_types)
        {
            EXPECT_TRUE(triggers_.find(trigger_type) != triggers_.end());
        }
         
        for (const auto& trigger : triggers_)
        {
            if(std::find(expected_trigger_types.begin(), expected_trigger_types.end(), trigger.first) == expected_trigger_types.end())
            {
                EXPECT_FALSE(std::visit(isTriggerEnabled, trigger.second));
                continue;
            }
            EXPECT_TRUE(std::visit(isTriggerEnabled, trigger.second));
        }
        EXPECT_TRUE(isWriterInitialized());
        EXPECT_TRUE(trigger_buffer_timer_ != nullptr);
    }

    void callReset() {reset();}

    void publishBatteryMsg(bool is_activated)
    {
        using sensor_msgs::msg::BatteryState;
        auto battery_status = is_activated ? BatteryState::POWER_SUPPLY_HEALTH_DEAD : BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        auto msg = sensor_msgs::msg::BatteryState().set__power_supply_health(battery_status);
        battery_pub->publish(msg);
        rclcpp::spin_some(this->get_node_base_interface());
    }

    void publishBoolMsg(bool is_activated)
    {
        auto msg = std_msgs::msg::Bool().set__data(is_activated);
        bool_pub->publish(msg);
        rclcpp::spin_some(this->get_node_base_interface());
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
};

class TriggeredRecorderNodeTestFixture : public ::testing::Test
{
  protected:
    TriggeredRecorderNodeTestFixture()
    {}

    ~TriggeredRecorderNodeTestFixture()
    {
        rclcpp::shutdown();
    }

    virtual void SetUp() override {}
    
    void SetUp(const std::string& config_dir)
    {
        rclcpp::init(0, nullptr);
        test_helper_ = std::make_shared<TriggeredRecorderNodeTestHelper>("triggered_recorder_node_test_helper", rclcpp::NodeOptions(), config_dir);
    }

    virtual void TearDown() override
    {}
    
    std::shared_ptr<TriggeredRecorderNodeTestHelper> test_helper_;
};

TEST_F(TriggeredRecorderNodeTestFixture, test_valid_initialization)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");

    const size_t expected_topics_recorded = 2;
    std::vector<std::string> expected_trigger_types = {"EmptyTrigger", "BatteryHealthTrigger"};
    const size_t expected_subscriptions = 3; //Topics that are neither triggered nor recorded are not added to subscriptions.
    
    test_helper_->testInit(expected_topics_recorded, expected_subscriptions, expected_trigger_types);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_invalid_config_dir)
{
    EXPECT_THROW(SetUp("invalid_path"), std::runtime_error);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_invalid_config_filenames)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/invalid_filenames_config"), YAML::BadFile);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_extra_triggers)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/config_with_extra_triggers"), std::out_of_range);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_invalid_trigger_names)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/config_with_invalid_trigger_names"), std::out_of_range);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_missing_triggers)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/config_with_missing_triggers");

    const size_t expected_topics_recorded = 1;
    std::vector<std::string> expected_trigger_types = {"EmptyTrigger"};
    const size_t expected_subscriptions = 2; //Topics that are neither triggered nor recorded are not added to subscriptions.
    
    test_helper_->testInit(expected_topics_recorded, expected_subscriptions, expected_trigger_types);    
}

TEST_F(TriggeredRecorderNodeTestFixture, test_incomplete_trigger_configs)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/incomplete_trigger_config"), YAML::InvalidNode);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_topic_callback)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");
    const auto trigger_duration = rclcpp::Duration::from_seconds(2.2);

    auto start1 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start1 < trigger_duration)
    {
        test_helper_->publishBatteryMsg(/*is_activated=*/true);
    }
    test_helper_->publishBatteryMsg(/*is_activated=*/false);

    auto start2 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start2 < trigger_duration)
    {
        test_helper_->publishBoolMsg(/*is_activated=*/true);
    }
    test_helper_->publishBoolMsg(/*is_activated=*/false);
    

    auto triggers = test_helper_->getTriggers();
    auto crop_points = test_helper_->getCropPoints();
    EXPECT_TRUE(std::visit(getTriggerPulses, triggers.at("BatteryHealthTrigger")).size() == 1);
    EXPECT_TRUE(std::visit(getTriggerPulses, triggers.at("EmptyTrigger")).size() == 1);
    EXPECT_TRUE(crop_points.first > 0 && crop_points.second > 0);
    EXPECT_EQ(crop_points.first , std::visit(getTriggerPulses, triggers.at("BatteryHealthTrigger")).at(0).start_time_);
    EXPECT_EQ(crop_points.second , std::visit(getTriggerPulses, triggers.at("EmptyTrigger")).at(0).end_time_);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_reset)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");
    const auto trigger_duration = rclcpp::Duration::from_seconds(2.2);

    auto start1 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start1 < trigger_duration)
    {
        test_helper_->publishBatteryMsg(/*is_activated=*/true);
        test_helper_->publishBoolMsg(/*is_activated=*/true);
    }
    test_helper_->publishBatteryMsg(/*is_activated=*/false);
    test_helper_->publishBoolMsg(/*is_activated=*/false);
    test_helper_->callReset();
    auto triggers = test_helper_->getTriggers();
    auto crop_points = test_helper_->getCropPoints();
    EXPECT_TRUE(std::visit(getTriggerPulses, triggers.at("BatteryHealthTrigger")).size() == 0);
    EXPECT_TRUE(std::visit(getTriggerPulses, triggers.at("EmptyTrigger")).size() == 0);
    EXPECT_TRUE(crop_points.first == -1 && crop_points.second == -1);
    EXPECT_FALSE(test_helper_->isWriterInitialized());
    
}
#include <ros2bag_triggered/triggered_writer.hpp>
#include <gtest/gtest.h>

using namespace ros2bag_triggered;

class TriggeredWriterTestHelper : public TriggeredWriter
{
    public:
        TriggeredWriterTestHelper(const rclcpp::Logger& logger) : TriggeredWriter(logger)
        {}
    
        ~TriggeredWriterTestHelper()
        {}
};

class TriggeredWriterTestFixture : public ::testing::Test
{
  protected:
    TriggeredWriterTestFixture()
    {}

    ~TriggeredWriterTestFixture()
    {}

    virtual void SetUp() override { test_helper_ = std::make_shared<TriggeredWriterTestHelper>(rclcpp::get_logger("TriggeredWriterTests")); }

    virtual void TearDown() override
    {}
    
    std::shared_ptr<TriggeredWriterTestHelper> test_helper_;
};

TEST_F(TriggeredWriterTestFixture, test_valid_initialization)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_NO_THROW(test_helper_->initialize(prefix_path / "test/configs/valid_config/writer_config.yaml"));
    EXPECT_TRUE(test_helper_->get_config().bag_root_dir == "/test_bags");
    EXPECT_TRUE(test_helper_->get_config().trigger_buffer_duration == 600.0);
    EXPECT_TRUE(test_helper_->get_config().crop_gap == 10.0);
    EXPECT_TRUE(test_helper_->get_config().bag_cropping == true);
    EXPECT_TRUE(test_helper_->get_config().write_trigger_stats == true);
    EXPECT_TRUE(test_helper_->get_storage_options().max_bagfile_size == 1000000000);
    EXPECT_TRUE(test_helper_->get_storage_options().max_bagfile_duration == 300);
    EXPECT_TRUE(test_helper_->get_storage_options().max_cache_size == 0);

}

TEST_F(TriggeredWriterTestFixture, test_invalid_config_filenames)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(test_helper_->initialize(prefix_path / "test/configs/invalid_filenames_config/writer_config.yaml"), YAML::BadFile);
}

TEST_F(TriggeredWriterTestFixture, test_incomplete_config)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_NO_THROW(test_helper_->initialize(prefix_path / "test/configs/incomplete_config/writer_config.yaml"));
}

TEST_F(TriggeredWriterTestFixture, test_config_with_invalid_field_types)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(test_helper_->initialize(prefix_path / "test/configs/config_with_wrong_field_types/writer_config.yaml"), YAML::BadConversion);
}

TEST_F(TriggeredWriterTestFixture, test_open)
{
    rosbag2_storage::StorageOptions storage_options; 
    const rosbag2_cpp::ConverterOptions converter_options;
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    test_helper_->initialize(prefix_path / "test/configs/valid_config/writer_config.yaml");
    std::string bag_name = "test_bag_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    storage_options.uri = bag_name;

    EXPECT_NO_THROW(test_helper_->open(storage_options, converter_options));
    EXPECT_TRUE(test_helper_->is_open());
    EXPECT_TRUE(test_helper_->get_bag_name() == bag_name);
    EXPECT_TRUE(test_helper_->get_base_folder() == test_helper_->get_config().bag_root_dir + "/" + bag_name);
    EXPECT_TRUE(std::filesystem::exists(test_helper_->get_base_folder()));
}
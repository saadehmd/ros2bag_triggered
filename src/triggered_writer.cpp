#include <ros2bag_triggered/triggered_writer.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/bag_rewrite.hpp>
#include <chrono>

using std::chrono::duration_cast;

namespace ros2bag_triggered
{
    void TriggeredWriter::set_crop_points(int64_t start_time, int64_t end_time)
    {   
        auto gap = rclcpp::Duration::from_seconds(config_.crop_gap).nanoseconds();

        start_time -= static_cast<int64_t>(gap);
        end_time += static_cast<int64_t>(gap);
        auto bag_start_time = duration_cast<std::chrono::nanoseconds>(metadata_.starting_time.time_since_epoch());
        auto bag_end_time = bag_start_time + duration_cast<std::chrono::nanoseconds>(metadata_.duration);

        rewrite_options_ = std::make_optional<rosbag2_storage::StorageOptions>(storage_options_);
        rewrite_options_->uri = base_folder_ +"_triggered";
        rewrite_options_->start_time_ns = std::max(start_time, bag_start_time.count());
        rewrite_options_->end_time_ns = std::min(end_time, bag_end_time.count());

    }

    void TriggeredWriter::initialize(const std::optional<TriggeredWriter::Config>& config)
    {
        auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        
        if (config.has_value())
        {
            config_ = config.value();
            return;
        }

        YAML::Node writer_cfg = YAML::LoadFile(prefix_path + "/config/config.yaml");

        storage_options_.storage_id = rosbag2_storage::get_default_storage_id();
        storage_options_.max_bagfile_size = writer_cfg["max_bagfile_size"].as<uint64_t>();
        storage_options_.max_bagfile_duration = writer_cfg["max_bagfile_duration"].as<uint64_t>();
        storage_options_.max_cache_size = writer_cfg["max_cache_size"].as<uint64_t>();
        config_.trigger_buffer_duration = writer_cfg["trigger_buffer_interval"].as<double>();
        config_.crop_gap = writer_cfg["crop_gap"].as<double>();
        config_.bag_root_dir =  writer_cfg["bag_root_dir"].as<std::string>();
        config_.write_trigger_stats = writer_cfg["write_trigger_stats"].as<bool>();
    
    }

    void TriggeredWriter::close()
    {
        rosbag2_cpp::writers::SequentialWriter::close();

        /* Crop is simply a rewrite with new start/end timestamp information.
        Only supported with ROS2 >= Jazzy */
        if (rewrite_options_.has_value())
        {
            storage_options_.uri = base_folder_;
            rosbag2_transport::RecordOptions record_options;
            record_options.all_topics = true;
            std::cout<<"Rewriting bag... on "<<rewrite_options_->uri<<"\n";
            rosbag2_transport::bag_rewrite({storage_options_}, {std::make_pair(*rewrite_options_, record_options)});
            rewrite_options_.reset(); 
        }

        // Delete the original bag whether or not the tirggers were detected.
        if (std::filesystem::exists(base_folder_))
        {
            std::filesystem::remove_all(base_folder_);
        }
          
    }

    void TriggeredWriter::open(const rosbag2_storage::StorageOptions& storage_options, const rosbag2_cpp::ConverterOptions& converter_options)
    {
        base_folder_ = storage_options.uri;
        std::cout<<"Base folder: "<<base_folder_<<std::endl;
        rosbag2_cpp::writers::SequentialWriter::open(storage_options, converter_options);
    }
      
}


# ros2bag_triggered
A collection of helper classes for rosbag2 focused mainly at providing triggering functionality with ros2 bags.

## Brief Introduction:  
This package is aimed at extending the functionality provided by [rosbag2_snapshot](https://github.com/gaia-platform/rosbag2_snapshot). The idea of keeping a small data-buffer of recorded data and being able to trigger, when to save this buffered  bag (instead of an eternally running rosbag recorder) is insanely helpful in a lot of production environments where even minutes worth recorded rosbags can turn into gigabytes of data or when routine clean-ups of disk-space are not practical. 

Yet, even with a handy tool like rosbag_snapshot, users ought to handle the triggering logic themselves. Writing it either as separate client nodes, monitoring data on rostopics-of-interest or as extra logic within their own nodes publishing those topics.
This can easily get out-of-hands, in an environment where you can have tens of rostopics-of-interests (whether being recorded or simply acting as mere triggers for saving the recording.). Besides triggering, one might also want to analyse why a certain bag was triggered and what kind of triggers have been produced over previous N recording sessions. 

## Objectives:
Solving the above problems partially or entirely is the main motivation behind this project. 

The core idea is to provide a convinient way to :-

- Write triggering logic isolated from the recording or publishing node.
- Organize triggering logic into individual modules with a uniform base-interface.
- Fine-tune how triggers crop, delete or copy the recorded bag using a quickly-configurable YAML interface.
- Re-configure trigger-logic-specific parameters from YAML rather than recompiling the code.
- Combine multiple triggers into a single module and delegate the tasks of initialization, resetting, and aggregating these triggers to a dedicated recorder node. 
- Plot triggers or collect important statistics on the triggered ros-bags 

## How triggering works:
Every trigger essentially registers a persistant occurance of a triggering condition. The `TriggerRecorderNode` owns and manages these triggers and binds them to the respective topic-subscribers being used to monitor the triggering condition. `TriggerRecorderNode` records a bag for a pre-configured duration and buffers all the triggers in this duration. At the end, this bag is marked as "Triggered" and moved to user-configured directory for stashing all the triggered bags. The `TriggerRecorderNode` then resets all the triggers and opens a new bag for recording.

## Bag-cropping:
The users also have option to crop only the interesting(triggered) duration from the whole bag. This cropped section of the bag is from ```start_time``` of the earliest occuring trigger to the ```ènd_time``` of latest occuring trigger. Users can also add a sort of padding or gap on each end of crop point. This is useful because, the events leading upto the point of a trigger situation might be equally interesting as the triggering situation itself. This is configured by the ```crop_gap``` parameter explained in following sections.

## Useage:
Working with ros2bag_triggered follows a simple paradigm:-
1. You implement your "triggering-logic" to a custom `TriggerType` inheriting the interface from `TriggerBase` class.  
2. You write some basic configuration for each trigger into a single `config.yaml`
3. You hand over your custom TriggerTypes to the templated `TriggeredRecorderNode<std::variant<YourTriggerTypes>>` class. This class safely initializes your custom trigger types based on the `config.yaml` file and manages their lifecycle within the recorder.
4. You use this `TriggeredRecorderNode<>` class instance anywhere inside your code, with either single-executor, multi-executor or composable-nodes setup.

### Implementing & Configuring Custom TriggerTypes:
Refer to : [examples](examples/README.md)

### Configuring TriggeredRecorderNode Params:
```yaml

The TriggeredRecorderNode takes path to the config folder as input and expects both 'topic_config.yaml' and 'writer_config.yaml' in the provided config folder.

The following gives an example for the 'writer_config.yaml':-

bag_root_dir: "./ros2bag_triggered/bags"    # The directory where the bag files will be stored. The bag name would be inferred as 
                                            # Unix timestamp. Change this root dir accordingly.

max_bagfile_size: 1000000000                # The maximum size a bagfile can be, in bytes, before it is split.
                                            # A value of 0 indicates that bagfile splitting will not be used.

max_bagfile_duration: 300                   # The maximum duration a bagfile can be, in seconds, before it is split.
                                            # A value of 0 indicates that bagfile splitting will not be used.

max_cache_size: 0                           # Bytes. A value of 0 disables caching and every write happens directly to disk.

trigger_buffer_interval: 600.0              # The interval in seconds for which the triggers are buffered before being checked.
                                            # Once this interval is over, the triggers are checked and reset, the bagfile along with
                                            # a trigger report and plot is saved if any trigger was active otherwise removed and a 
                                            # new bagfile is opened for writing. 
                                            # NOTE!! This is different than the above "bag-splitting" feature, which is rosbag2's 
                                            # inherent feature for splitting a single ros2bag into multiple .db files while the 
                                            # bag-folder and metadata remain similar for the whole bag. This param actually closes 
                                            # the last buffered bag and opens a new one.

bag_cropping: True                          # Enable/disable bag cropping. If enabled, the bag is rewritten from first to last
                                            # trigger stamp. i.e.; From start-time of the first trigger to the end-time of the last.
                                            # If disabled, the whole bag is kept (and not rewritten) if triggered.

crop_gap: 10.0                              # The gap (padding) in seconds to add before and after the crop-points of the bag.If this
                                            # padding results in cropping points outside the recorded bag duration, the crop-points 
                                            # are simply clamped upto actual bag-start or bag-end time.

write_trigger_stats: true                   # If true, the trigger stats and plot will be written to the bag directory. 
```

### Using TriggeredRecordNode with the custom TriggerTypes
Refer to a simple triggered_recorder_node [example](examples/main.cpp) using [custom](include/examples/) TriggerTypes:-
- [BatteryHealthTrigger](examples/src/battery_health_trigger.cpp) 
- [NavSatInvalidFixTrigger](examples/src/navsat_invalid_fix.cpp) 
- [ZoneTriggerWithNavSatFix](examples/src/zone_trigger_with_navsat_fix.cpp) 
- [ZoneTriggerWithPoseStamped](examples/src/zone_trigger_with_pose_stamped.cpp) 
- [VelocityTrigger] ()
- [ProximityTrigger] ()

### Build & Run the example TriggeredRecordNode:
```bash
# Build the package
colcon build --packages-select ros2bag_triggered

# Run the example node
ros2 run ros2bag_triggered example_node

# Note: If running the following script inside a Docker container, ensure GUI access is enabled.
# You can enable GUI access by using X11-forwarding:
# Refer: https://medium.com/@priyamsanodiya340/running-gui-applications-in-docker-containers-a-step-by-step-guide-335b54472e4b
# Alternatively, run the following script outside the container if you're comfortable using "docker run --network=host" 

# Run the dummy trigger generator script
python test_utils/dummy_trigger_generator.py
```
### Trigger stats and plots: 
If you have enabled ```write_trigger_stats``` option, the writer will output a bunch of analytical reports for you from the triggers that occured during each bag and save those in the bag folder itself. You should have following in the bag folder if it was triggered:-
1. ```metadata.yaml```
2. N number of bag files 
3. ```trigger_stats.txt```  (This is a human-readable report of trigger pulses and their durations for a quick scan).
4. ```trigger_plot.pdf```   (This is a visual report of trigger pulses).
5. ```trigger_stats.json``` (This is a record of triggers readable by some python script, useful for your own plotting/analysis).

#### Example Trigger Stats Plot:
<img src="./plot_example.png" width="960" height="640" />

#### Example trigger_stats.txt:
![Trigger Stats txt](stats_txt_example.png)

#### Example trigger_stats.json:
![Trigger Stats json](stats_json_example.png)


### ToDo:
- Support registering a single trigger type to multiple topics.
- Provide a self-unlatch configuration for triggers.
- Benchmark bag-cropping/rewrite for huge bagfiles. Move to a separate thread if it creates a bottleneck for bag switch-over.
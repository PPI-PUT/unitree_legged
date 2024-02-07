import rosbag2_py
from rosidl_runtime_py import convert_ros_message_to_dictionary
from unitree_a1_legged_msgs.msg import LowState

# Path to your ROS bag file
bag_file_path = '/home/mackop/intention_policy/rosbag2_2024_02_05-19_54_01'

# Create a ROS 2 bag reader
storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
)
reader = rosbag2_py.SequentialReader()

# Open the bag file
reader.open(storage_options, converter_options)

# Iterate through topics to find '/unitree_a1_legged/state'
topic_info = reader.get_all_topics_and_types()
state_topic_name = '/unitree_a1_legged/state'
state_topic_type = 'unitree_a1_legged_msgs/msg/LowState'  # Ensure this matches your topic type

# Filter topic to read messages from
filtered_topics = [
    topic for topic in topic_info
    if topic.topic_name == state_topic_name and topic.type == state_topic_type
]

if not filtered_topics:
    print(f"No messages found for topic '{state_topic_name}' with type '{state_topic_type}'.")
else:
    # Reading messages
    while reader.has_next():
        (topic, msg, t) = reader.read_next()
        if topic == state_topic_name:
            # Deserialize message to Python object
            ros_msg = LowState().deserialize(msg)
            # Optionally convert to dictionary for easier access
            msg_dict = convert_ros_message_to_dictionary(ros_msg)
            print(f"Header: {msg_dict['header']}")

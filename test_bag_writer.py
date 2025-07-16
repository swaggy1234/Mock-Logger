from rclpy.serialization import serialize_message
from std_msgs.msg import String
import rosbag2_py

writer = rosbag2_py.SequentialWriter()

storage_options = rosbag2_py.StorageOptions(
    uri='test_bag',
    storage_id='mcap'
)
converter_options = rosbag2_py.ConverterOptions('', '')
writer.open(storage_options, converter_options)

topic_info = rosbag2_py.TopicMetadata(
    name='test_topic',
    type='std_msgs/msg/String',
    serialization_format='cdr'
)
writer.create_topic(topic_info)

msg = String()
msg.data = "hello world"
writer.write('test_topic', serialize_message(msg), 0)

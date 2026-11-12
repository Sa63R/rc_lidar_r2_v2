"""Read existing odometry bags without publishing or rewriting their stamps."""
from pathlib import Path
import yaml


def odometry_messages(path, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    path = Path(path)
    metadata = yaml.safe_load((path/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    entry = next((t for t in metadata['topics_with_message_count']
                  if t['topic_metadata']['name'] == topic), None)
    if entry is None or entry['message_count'] <= 0:
        raise ValueError(f'bag has no {topic} messages: {path}')
    if entry['topic_metadata']['type'] != 'nav_msgs/msg/Odometry':
        raise ValueError('selected topic must be nav_msgs/msg/Odometry')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        name, data, receipt_ns = reader.read_next()
        if name == topic:
            yield receipt_ns, deserialize_message(data, Odometry)

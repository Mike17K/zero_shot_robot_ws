"""Save / load single JointTrajectory messages as mcap bags - e.g. to replay
a planned motion without re-planning."""
import os
from typing import Optional

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import (ConverterOptions, SequentialReader, SequentialWriter, StorageFilter,
                        StorageOptions, TopicMetadata)
from trajectory_msgs.msg import JointTrajectory

TOPIC = 'joint_trajectory'
_CDR = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')


def write_trajectory(traj: JointTrajectory, bag_dir: str, timestamp_ns: int = 0) -> str:
    """Write traj into a new bag directory bag_dir (must not exist yet).
    Returns bag_dir."""
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=bag_dir, storage_id='mcap'), _CDR)
    writer.create_topic(TopicMetadata(id=0, name=TOPIC, type='trajectory_msgs/msg/JointTrajectory',
                                      serialization_format='cdr'))
    writer.write(TOPIC, serialize_message(traj), timestamp_ns)
    del writer  # closes the bag
    return bag_dir


def read_trajectory(bag_dir: str) -> Optional[JointTrajectory]:
    """Last JointTrajectory stored in the bag at bag_dir, or None if it has none."""
    if not os.path.exists(bag_dir):
        raise FileNotFoundError(bag_dir)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='mcap'), _CDR)
    reader.set_filter(StorageFilter(topics=[TOPIC]))
    traj = None
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        traj = deserialize_message(data, JointTrajectory)
    return traj

import csv
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from .math_utils import quat_matrix, matrix_state, transform


class Logger(Node):
    def __init__(self, **kwargs):
        super().__init__('task1_logger', **kwargs)
        self.declare_parameter('output_file', 'results/real_run.csv')
        self.declare_parameter('reference_topic', '/robot/pose_continuous')
        path = Path(self.get_parameter('output_file').value)
        path.parent.mkdir(parents=True, exist_ok=True)
        diag_path = path.with_suffix('.diagnostics.jsonl')
        if path.exists() or diag_path.exists():
            raise FileExistsError(f'Choose a new csv:= filename; existing recording will be preserved: {path}')
        self.file = path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=['receipt_t', 'source_t', 'stream', 'x', 'y', 'z', 'yaw'])
        self.writer.writeheader()
        self.diag_file = diag_path.open('x', encoding='utf-8')
        topics = {'fused': '/task1/fused/pose', 'lidar': '/task1/baseline/lidar',
                  'naive': '/task1/baseline/naive', 'reference': self.get_parameter('reference_topic').value}
        self.subscriptions_kept = [self.create_subscription(PoseStamped, topic,
            lambda msg, name=name: self.record(name, msg), qos_profile_sensor_data) for name, topic in topics.items()]
        self.create_subscription(DiagnosticArray, '/task1/diagnostics', self.record_diag, qos_profile_sensor_data)
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Recording actual receipts and source stamps to {path.resolve()}')

    def record(self, stream, msg):
        p, q = msg.pose.position, msg.pose.orientation
        try:
            x, _ = matrix_state(transform([p.x, p.y, p.z], quat_matrix([q.x, q.y, q.z, q.w])))
        except ValueError:
            return
        self.writer.writerow({'receipt_t': self.get_clock().now().nanoseconds*1e-9,
            'source_t': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'stream': stream, 'x': x[0], 'y': x[1], 'z': x[2], 'yaw': x[3]})

    def record_diag(self, msg):
        record = {'receipt_t': self.get_clock().now().nanoseconds*1e-9, 'status': [
            {'name': s.name, 'level': s.level[0] if isinstance(s.level, bytes) else s.level, 'message': s.message,
             'values': {v.key: v.value for v in s.values}} for s in msg.status]}
        self.diag_file.write(json.dumps(record, ensure_ascii=False)+'\n')

    def flush(self):
        self.file.flush()
        self.diag_file.flush()

    def destroy_node(self):
        self.flush()
        self.file.close()
        self.diag_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

import rclpy
import csv
from pathlib import Path
from rclpy.node import Node
from rclpy.time import Time
# import numpy as np
# from scipy.spatial.transform import Rotation

from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException


class CalibrationCapture(Node):

    def __init__(self):
        super().__init__('calibration_capture')

        # Configurable coordinate frames
        self.declare_parameter('camera_frame', 'head_camera_color_optical_frame')
        # self.declare_parameter('camera_frame', 'd435_link')
        self.declare_parameter('tag_frame', 'tagStandard41h12:0')
        self.declare_parameter('base_frame', 'pelvis')
        self.declare_parameter('hand_frame', 'right_wrist_yaw_link')
        self.declare_parameter('recording_enabled', False)

        self.declare_parameter('output_csv', str(Path.home() / 'ros2g1_ws/src/karis_calibration/results/measurements.csv'))

        self.last_recorded_tag_stamp = None
        self.sample_id = 0
        self.latest_detection = None
        self.declare_parameter('pose_id', 1)

        self.csv_columns = [
            'sample_id',
            'pose_id',
            'tag_time_s',
            'wrist_time_s',
            'decision_margin',
            'tag_tx', 'tag_ty', 'tag_tz',
            'tag_qx', 'tag_qy', 'tag_qz', 'tag_qw',
            'wrist_tx', 'wrist_ty', 'wrist_tz',
            'wrist_qx', 'wrist_qy', 'wrist_qz', 'wrist_qw'
        ]

        # Subscribe to AprilTag detections
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)

        # Store and listen for TF transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Check the latest tag pose every 0.1 seconds
        # Check whether a measurement should be recorded every 0.1 seconds
        self.record_timer = self.create_timer(0.1, self.record_measurement)
        self.get_logger().info('Calibration capture node started')

    def detection_callback(self, msg):

        for detection in msg.detections:
            self.latest_detection = {
                'stamp': msg.header.stamp,
                'decision_margin': detection.decision_margin
            }

            self.get_logger().info(
                f'AprilTag detected: '
                f'Family={detection.family}, '
                f'ID={detection.id}, '
                f'Decision margin={detection.decision_margin:.2f}'
            )

    
    def record_measurement(self):

        if not self.get_parameter('recording_enabled').value:
            return

        if self.latest_detection is None:
            return

        camera_frame = self.get_parameter('camera_frame').value
        tag_frame = self.get_parameter('tag_frame').value
        base_frame = self.get_parameter('base_frame').value
        hand_frame = self.get_parameter('hand_frame').value

        if not tag_frame or not hand_frame:
            return

        try:
            # Get the latest AprilTag transformation
            tag_tf = self.tf_buffer.lookup_transform(
                camera_frame, tag_frame, Time()
            )

            tag_stamp = tag_tf.header.stamp
            tag_stamp_ns = tag_stamp.sec * 1_000_000_000 + tag_stamp.nanosec

            # Avoid recording the same tag observation twice
            if tag_stamp_ns == self.last_recorded_tag_stamp:
                return

            # Match detection metadata with the tag transformation
            detection_stamp = self.latest_detection['stamp']
            detection_ns = (
                detection_stamp.sec * 1_000_000_000
                + detection_stamp.nanosec
            )

            if abs(tag_stamp_ns - detection_ns) > 50_000_000:
                return

            # Retrieve wrist pose at the AprilTag timestamp
            wrist_tf = self.tf_buffer.lookup_transform(
                base_frame,
                hand_frame,
                Time.from_msg(tag_stamp)
            )

        except TransformException:
            return

        # Extract translation and rotation
        tag_t = tag_tf.transform.translation
        tag_q = tag_tf.transform.rotation

        wrist_t = wrist_tf.transform.translation
        wrist_q = wrist_tf.transform.rotation

        # Convert timestamps to seconds
        tag_time_s = tag_stamp.sec + tag_stamp.nanosec / 1e9
        wrist_stamp = wrist_tf.header.stamp
        wrist_time_s = wrist_stamp.sec + wrist_stamp.nanosec / 1e9

        self.sample_id += 1

        row = [
            self.sample_id,
            self.get_parameter('pose_id').value,
            f'{tag_time_s:.3f}',
            f'{wrist_time_s:.3f}',
            self.latest_detection['decision_margin'],
            tag_t.x, tag_t.y, tag_t.z,
            tag_q.x, tag_q.y, tag_q.z, tag_q.w,
            wrist_t.x, wrist_t.y, wrist_t.z,
            wrist_q.x, wrist_q.y, wrist_q.z, wrist_q.w
        ]

        output_path = Path(self.get_parameter('output_csv').value)

        # Open the existing CSV without creating or overwriting it
        with output_path.open('r+', newline='') as file:
            file.seek(0, 2)

            writer = csv.writer(file)

            if file.tell() == 0:
                writer.writerow(self.csv_columns)

            writer.writerow(row)

        self.last_recorded_tag_stamp = tag_stamp_ns

        self.get_logger().info(
            f'Recorded sample {self.sample_id}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = CalibrationCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

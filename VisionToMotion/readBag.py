from pathlib import Path
import csv

import rosbag2_py
from rclpy.serialization import deserialize_message
from karis_interfaces.msg import ObjectPoseArray # MUST colcon build karis_interfaces first, otherwise wont work

# Paths, change to your own
BAG_PATH = Path("/home/gustaubuntu/ros2g1_ws/perception_out")
# TOPIC_NAME = "/karis/object_poses_base"
TOPIC_NAME = "/karis/object_poses"
OUTPUT_PATH = Path("/home/gustaubuntu/ros2g1_ws/Skript/coordinatesFP/coordinatesFP.txt")


def main():
    if not BAG_PATH.is_dir():
        print(f"Bag directory not found: {BAG_PATH}")
        return

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(
        uri=str(BAG_PATH),
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    message_count = 0
    object_count = 0

    with OUTPUT_PATH.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file, delimiter="\t")

        # Column names for fp2moveIt.py
        writer.writerow([
            "message",
            "timestamp_ns",
            "frame_id",
            "object_id",
            "class_name",
            "confidence",
            "has_orientation",
            "x", "y", "z",
            "qx", "qy", "qz", "qw"
        ])

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            # Only process the topic we are interested in
            if topic != TOPIC_NAME:
                continue

            # Convert the binary data into a ROS 2 message
            msg = deserialize_message(data, ObjectPoseArray)
            message_count += 1

            print(f"\nMessage {message_count}")
            print(f"Reference frame: {msg.header.frame_id}")
            print(f"Objects detected: {len(msg.objects)}")

            for obj in msg.objects:
                object_count += 1

                p = obj.pose.position
                q = obj.pose.orientation

                print(f"\n  Object ID: {obj.object_id}")
                print(f"  Class: {obj.class_name}")
                print(f"  Confidence: {obj.confidence:.3f}")
                print(f"  Has orientation: {obj.has_orientation}")

                print("  Position (m):")
                print(f"    x: {p.x:.4f}")
                print(f"    y: {p.y:.4f}")
                print(f"    z: {p.z:.4f}")

                print("  Orientation (quaternion):")
                print(f"    x: {q.x:.4f}")
                print(f"    y: {q.y:.4f}")
                print(f"    z: {q.z:.4f}")
                print(f"    w: {q.w:.4f}")

                # Save extracted coordinates to .txt file, set path at top
                writer.writerow([
                    message_count,
                    timestamp,
                    msg.header.frame_id,
                    obj.object_id,
                    obj.class_name,
                    obj.confidence,
                    obj.has_orientation,
                    p.x, p.y, p.z,
                    q.x, q.y, q.z, q.w
                ])

    print("\n" + "=" * 50)
    print(f"Total messages processed: {message_count}")
    print(f"Total object observations: {object_count}")
    print(f"Coordinates saved to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()

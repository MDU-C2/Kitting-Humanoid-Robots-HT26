
from pathlib import Path
import csv

import numpy as np
from scipy.spatial.transform import Rotation


# Paths, change to your own
INPUT_PATH = Path("/home/gustaubuntu/ros2g1_ws/Skript/coordinatesFP/coordinatesFP.txt")
OUTPUT_PATH = Path("/home/gustaubuntu/ros2g1_ws/Skript/coordinatesMoveIt/coordinatesMoveIt.txt")

# Transformation from d435_link to pelvis
# Obtained using ros2 run tf2_ros tf2_echo pelvis d435_link in bash
T_PELVIS_D435 = np.array([
    [ 0.674, 0.0, 0.738, 0.054],
    [ 0.0,    1.0, 0.0,    0.018],
    [-0.738, 0.0, 0.674, 0.474],
    [ 0.0,    0.0, 0.0,    1.0   ]
])

# Optical frame to d435_link transformation to change xyz axes from cameras xyz convention
T_D435_OPTICAL = np.array([
    [ 0.0, 0.0,  1.0, 0.0],
    [-1.0, 0.0,  0.0, 0.0],
    [ 0.0,-1.0,  0.0, 0.0],
    [ 0.0, 0.0,  0.0, 1.0]
])

# Complete transformation: optical frame -> pelvis
T_PELVIS_OPTICAL = T_PELVIS_D435 @ T_D435_OPTICAL


def transform_position(x, y, z):
    # Homogeneous coordinates
    p_optical = np.array([x, y, z, 1.0])

    # Apply complete transformation
    p_pelvis = T_PELVIS_OPTICAL @ p_optical

    return p_pelvis[:3]


def transform_orientation(qx, qy, qz, qw):
    # Convert FoundationPose quaternion to rotation matrix
    R_optical_object = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

    # Transform object orientation into pelvis frame
    R_pelvis_object = T_PELVIS_OPTICAL[:3, :3] @ R_optical_object

    # Convert resulting rotation matrix back to quaternion (xyzw)
    return Rotation.from_matrix(R_pelvis_object).as_quat()


def main():
    if not INPUT_PATH.is_file():
        print(f"Input file not found: {INPUT_PATH}")
        return

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    object_count = 0

    with INPUT_PATH.open("r", newline="", encoding="utf-8") as input_file, \
         OUTPUT_PATH.open("w", newline="", encoding="utf-8") as output_file:

        reader = csv.DictReader(input_file, delimiter="\t")

        # Preserve original metadata, but replace xyz and frame
        fieldnames = [
            "message", "timestamp_ns", "frame_id",
            "object_id", "class_name", "confidence",
            "has_orientation", "x", "y", "z",
            "qx", "qy", "qz", "qw"
        ]

        writer = csv.DictWriter(output_file, fieldnames=fieldnames, delimiter="\t")
        writer.writeheader()

        for row in reader:
            if row["frame_id"] != "head_camera_color_optical_frame":
                print(f"Skipping object in unexpected frame: {row['frame_id']}")
                continue

            # Extract original FoundationPose position
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])

            # Convert position into pelvis frame
            p_pelvis = transform_position(x, y, z)

            # Update position and reference frame
            row["x"] = f"{p_pelvis[0]:.6f}"
            row["y"] = f"{p_pelvis[1]:.6f}"
            row["z"] = f"{p_pelvis[2]:.6f}"
            row["frame_id"] = "pelvis"

            # Clear quaternion fields to avoid mislabeling them as pelvis-frame.
            if row["has_orientation"].strip().lower() == "true":
                q_pelvis = transform_orientation(
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"])
                )

                row["qx"] = f"{q_pelvis[0]:.6f}"
                row["qy"] = f"{q_pelvis[1]:.6f}"
                row["qz"] = f"{q_pelvis[2]:.6f}"
                row["qw"] = f"{q_pelvis[3]:.6f}"
                row["has_orientation"] = "True"
            else:
                row["qx"] = ""
                row["qy"] = ""
                row["qz"] = ""
                row["qw"] = ""
                row["has_orientation"] = "False"

            writer.writerow(row)
            object_count += 1

            print(f"\nObject: {row['object_id']}")
            print(f"  X: {p_pelvis[0]:.4f} m")
            print(f"  Y: {p_pelvis[1]:.4f} m")
            print(f"  Z: {p_pelvis[2]:.4f} m")

    print("\n" + "=" * 50)
    print(f"Total objects transformed: {object_count}")
    print(f"Coordinates saved to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()

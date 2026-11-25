import csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import yaml


# Input measurement file
MEASUREMENTS_CSV = Path.home() / 'ros2g1_ws/src/karis_calibration/results/measurements.csv'
OUTPUT_YAML = Path.home() / 'ros2g1_ws/src/karis_calibration/results/camera_extrinsics.yaml'

BASE_FRAME = 'pelvis'
CAMERA_FRAME = 'head_camera_color_optical_frame'

EVALUATE_CALIBRATION = True


def load_measurements(csv_path):
    measurements = []

    with open(csv_path, 'r', newline='') as file:
        reader = csv.DictReader(file)

        for row in reader:
            measurement = {
                'sample_id': int(row['sample_id']),
                'pose_id': int(row['pose_id']),
                'tag_time_s': float(row['tag_time_s']),
                'wrist_time_s': float(row['wrist_time_s']),
                'decision_margin': float(row['decision_margin']),

                'tag_translation': [
                    float(row['tag_tx']),
                    float(row['tag_ty']),
                    float(row['tag_tz'])
                ],

                'tag_quaternion': [
                    float(row['tag_qx']),
                    float(row['tag_qy']),
                    float(row['tag_qz']),
                    float(row['tag_qw'])
                ],

                'wrist_translation': [
                    float(row['wrist_tx']),
                    float(row['wrist_ty']),
                    float(row['wrist_tz'])
                ],

                'wrist_quaternion': [
                    float(row['wrist_qx']),
                    float(row['wrist_qy']),
                    float(row['wrist_qz']),
                    float(row['wrist_qw'])
                ]
            }

            measurements.append(measurement)

    return measurements


def make_transform(translation, quaternion):
    # Create a 4x4 identity matrix
    T = np.eye(4)

    # Convert quaternion (qx, qy, qz, qw) into a rotation matrix
    R = Rotation.from_quat(quaternion).as_matrix()

    # Insert rotation and translation
    T[:3, :3] = R
    T[:3, 3] = translation

    return T


def calculate_wrist_to_tag(A, X0, B):
    # Invert pelvis-to-wrist to obtain wrist-to-pelvis
    A_inv = np.linalg.inv(A)

    # Calculate wrist-to-AprilTag transformation
    Y = A_inv @ X0 @ B

    return Y


def calibrate_camera(A_list, B_list):

    # Check that we have enough paired measurements
    if len(A_list) != len(B_list) or len(A_list) < 3:
        raise ValueError(
            "Calibration requires at least 3 paired measurements"
        )

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for A, B in zip(A_list, B_list):

        # A = ^pelvis T_wrist
        # Inverse A = ^wrist T_pelvis
        A_inv = np.linalg.inv(A)

        # Robot transformations for OpenCV eye-to-hand calibration
        R_gripper2base.append(A_inv[:3, :3])
        t_gripper2base.append(A_inv[:3, 3].reshape(3, 1))

        # Camera-to-AprilTag transformations
        R_target2cam.append(B[:3, :3])
        t_target2cam.append(B[:3, 3].reshape(3, 1))

    # OpenCV hand-eye calibration using Park's method
    R_camera, t_camera = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK # change method here
    )

    # Construct the resulting pelvis-to-camera transformation
    X = np.eye(4)
    X[:3, :3] = R_camera
    X[:3, 3] = t_camera.flatten()

    return X


def evaluate_calibration(A_list, B_list, X, Y):

    if len(A_list) != len(B_list) or len(A_list) == 0:
        raise ValueError("A_list and B_list must contain the same number of measurements")

    translation_errors = []
    rotation_errors = []

    for A, B in zip(A_list, B_list):

        # AprilTag pose in pelvis frame, calculated using the robot
        T_robot = A @ Y

        # AprilTag pose in pelvis frame, calculated using the camera
        T_camera = X @ B

        # Translation difference in millimeters
        t_error = np.linalg.norm(
            T_robot[:3, 3] - T_camera[:3, 3]
        ) * 1000

        # Relative rotation between the two estimates
        R_error = T_robot[:3, :3].T @ T_camera[:3, :3]

        # Convert relative rotation to an angle in degrees
        r_error = np.degrees(
            Rotation.from_matrix(R_error).magnitude()
        )

        translation_errors.append(t_error)
        rotation_errors.append(r_error)

    translation_errors = np.array(translation_errors)
    rotation_errors = np.array(rotation_errors)

    print("\n--- Calibration Error Evaluation ---")
    print(f"Number of measurements: {len(A_list)}")

    print(f"Mean translation error: {np.mean(translation_errors):.3f} mm")
    print(f"Maximum translation error: {np.max(translation_errors):.3f} mm")
    print(f"Translation RMSE: {np.sqrt(np.mean(translation_errors**2)):.3f} mm")

    print(f"Mean rotation error: {np.mean(rotation_errors):.3f} deg")
    print(f"Maximum rotation error: {np.max(rotation_errors):.3f} deg")
    print(f"Rotation RMSE: {np.sqrt(np.mean(rotation_errors**2)):.3f} deg")

    return translation_errors, rotation_errors



def save_calibration(X, output_path):

    # Extract translation and rotation from the transformation matrix
    translation = X[:3, 3]
    rotation_matrix = X[:3, :3]

    # Convert rotation matrix to quaternion (qx, qy, qz, qw)
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

    # Organize the calibration results
    result = {
        'parent_frame': BASE_FRAME,
        'child_frame': CAMERA_FRAME,

        'translation': {
            'x': float(translation[0]),
            'y': float(translation[1]),
            'z': float(translation[2])
        },

        'rotation': {
            'x': float(quaternion[0]),
            'y': float(quaternion[1]),
            'z': float(quaternion[2]),
            'w': float(quaternion[3])
        },

        'matrix': X.tolist()
    }

    # Write calibration results to YAML
    with open(output_path, 'w') as file:
        yaml.safe_dump(result, file, sort_keys=False)

    print(f"\nCalibration saved to: {output_path}")


def main():
    print("Starting G1 camera calibration solver")

    measurements = load_measurements(MEASUREMENTS_CSV)
    print(f"Loaded {len(measurements)} measurements")

    A_list = []
    B_list = []

    for measurement in measurements:
        # Pelvis-to-wrist transformation
        A = make_transform(
            measurement['wrist_translation'],
            measurement['wrist_quaternion']
        )

        # Camera-to-AprilTag transformation
        B = make_transform(
            measurement['tag_translation'],
            measurement['tag_quaternion']
        )

        A_list.append(A)
        B_list.append(B)


    if len(A_list) < 3:
        print("Not enough measurements for calibration.")
        return

    # Estimate the camera transformation using OpenCV
    X_calibrated = calibrate_camera(A_list, B_list)

    print("\n--- Calibrated pelvis-to-camera transformation ---")
    print(np.round(X_calibrated, 6))


    # Optional calibration error evaluation
    if EVALUATE_CALIBRATION:

        # Calculate wrist-to-tag transformation for each measurement
        Y_list = [
            calculate_wrist_to_tag(A, X_calibrated, B)
            for A, B in zip(A_list, B_list)
        ]

        # Calculate representative translation and rotation
        mean_translation = np.mean(
            [Y[:3, 3] for Y in Y_list],
            axis=0
        )

        mean_rotation = Rotation.from_matrix(
            [Y[:3, :3] for Y in Y_list]
        ).mean()

        # Construct representative wrist-to-tag transformation
        Y_mean = np.eye(4)
        Y_mean[:3, :3] = mean_rotation.as_matrix()
        Y_mean[:3, 3] = mean_translation

        # Evaluate how well A_i Y = X B_i holds
        evaluate_calibration(
            A_list,
            B_list,
            X_calibrated,
            Y_mean
        )


    # Export the calibrated camera transformation
    save_calibration(X_calibrated, OUTPUT_YAML)


if __name__ == '__main__':
    main()

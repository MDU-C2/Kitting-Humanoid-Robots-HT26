"""
Authors: Kim Svedberg
Description: Live RGB and depth view from the Unitree G1 head mounted
RealSense D435i, capable of taking a picure and save the photo on the 
host computer by pressing spacebar.

Run this on the G1's developer computer (192.168.123.164), over an SSH
session started with X forwarding (ssh -X unitree@192.168.123.164).

Each photo is first captured locally on the dev computer, then 
immediately sent to the host computer and deleted from the dev computer.

Requires passwordless SSH from the dev computer to host computer.

Controls:
    Spacebar - caputer a RGB and depth photo
    Esc - Exit the application
"""

import os
import subprocess
import tempfile
import time 

import pyrealsense2 as rs
import numpy as np 
import cv2

# Edit these parameters to match your system
LAPTOP_USER = "marc"
LAPTOP_IP = "192.168.123.222"
LAPTOP_DEST = "~/g1_photos"     # Folder where the photo is stored


# Makes sure the destination folder exist on the host computer
subprocess.run(
    ["ssh", f"{LAPTOP_USER}@{LAPTOP_IP}", f"mkdir -p {LAPTOP_DEST}"],
    check = False,
)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

photo_count = 0

try:
    with tempfile.TemporaryDirectory() as tmpdir:
        while True:
            frames = pipeline.wait_for_frames()
            color = np.asanyarray(frames.get_color_frame().get_data())
            depth = np.asanyarray(frames.get_depth_frame().get_data())
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
            )
 
            cv2.imshow("G1 color", color)
            cv2.imshow("G1 depth", depth_vis)
 
            key = cv2.waitKey(1)
 
            if key == 32:  # Spacebar
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                color_path = os.path.join(
                    tmpdir, f"color_{timestamp}.png"
                )
                depth_path = os.path.join(
                    tmpdir, f"depth_{timestamp}.png"
                )
                depth_raw_path = os.path.join(
                    tmpdir, f"depth_raw_{timestamp}.png"
                )
 
                cv2.imwrite(color_path, color)
                cv2.imwrite(depth_path, depth_vis)
                cv2.imwrite(depth_raw_path, depth)
 
                # Send all three photos to the host computer, then 
                # they're removed automatically when tmpdir is cleaned up 
                result = subprocess.run(
                    [
                        "scp",
                        color_path, depth_path, depth_raw_path,
                        f"{LAPTOP_USER}@{LAPTOP_IP}:{LAPTOP_DEST}/",
                    ],
                    capture_output=True,
                    text=True,
                )
 
                photo_count += 1
                if result.returncode == 0:
                    print(
                        f"[{photo_count}] Sent to host computer: "
                        f"color_{timestamp}.png (+ depth)"
                    )
                else:
                    print(
                        f"[{photo_count}] Transfer FAILED: "
                        f"{result.stderr.strip()}"
                    )
 
            elif key == 27:  # Esc
                break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"Done. {photo_count} photos sent to "
          f"{LAPTOP_USER}@{LAPTOP_IP}:{LAPTOP_DEST}")



# Every part the perception sees in one camera frame.
std_msgs/Header header    # stamp = camera frame time, frame_id = frame the poses are in
ObjectPose[] objects
string pipeline_mode      # "track" | "detect+track" | "detect+register"
float32 processing_ms     # detector + FoundationPose time for this frame

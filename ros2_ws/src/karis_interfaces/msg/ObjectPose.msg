# One part seen by the KARIS perception (detector + FoundationPose).
# All poses are in the header.frame_id of the ObjectPoseArray that holds this.

string object_id          # stable while the part is tracked, lower case: e.g. "rund_mutter_1"; TF frame "karis/<object_id>"
string class_name         # detector class, e.g. "Rund mutter" or "Rund mutter HOLE 11432713"
int32 class_id            # detector class index
float32 confidence        # detector confidence at the last detection that matched this part (0..1)

string mode               # "register": pose found from scratch this frame
                          # "track":    pose updated from the previous frame
                          # "centre":   class without a CAD mesh (tray slot): position only
bool has_orientation      # false for mode "centre": only pose.position is valid

geometry_msgs/Pose pose           # the CAD model's own frame (origin and axes of the .stl)
geometry_msgs/Pose box_pose       # centre and axes of the part's tight bounding box
geometry_msgs/Vector3 box_size    # edge lengths of that box [m] (0 for "centre")

float32 fit_iou           # rendered-model vs detected-mask overlap at the last check (0..1), -1 = not checked
float32 age_s             # seconds since the part was last confirmed by the detector

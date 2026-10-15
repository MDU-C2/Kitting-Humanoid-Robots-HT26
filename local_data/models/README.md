# Detector models (not in git)

Put the trained detectors here; they are too large for git and stay on each computer:

- YOLO: `*.pt` (e.g. `YOLO_26s.pt`), TensorRT: `*.engine` (made with `export_trt`)
- RF-DETR: `*.pth`, TensorRT: `*.trt` + `*.karis.json`

Select one with `model:=<file name>` in `perception.launch.py`.
Share models with the team outside git (e.g. Teams/OneDrive), or with Git LFS.

# Isaac helper image with TensorRT 10.13

The Isaac FoundationPose helper runs in `karis/isaac_fp:3.2-trt10.13`: the Isaac ROS 3.2 image
(`karis/isaac_fp:3.2`) with the TensorRT runtime upgraded from 10.3 to **10.13.3.9**. It is the
default in `docker/compose.yaml`; day to day you use it exactly as in `Perception_startup.md`.
This page is how the image is built, and how to switch to it (or back).

## Why

TensorRT 10.3 cannot build engines for Blackwell GPUs (RTX 50xx, e.g. the Eskilstuna PC's RTX 5090):

```
IBuilder::buildSerializedNetwork: Error Code 1: Internal Error (Unsupported SM: 0xc00)
```

FoundationPose runs only through TensorRT engines, so without a newer TensorRT there is no 6D pose
on such a GPU. 10.13 works on the RTX 5090 and on the laptop's RTX 3060 (tested: same results as
10.3). CUDA stays 12.6; the TensorRT `-dev` packages are removed (nothing in the image compiles
against TensorRT).

The detector models (`.pth` RF-DETR, `.pt` YOLO) do not use TensorRT; only FoundationPose's two
networks (`refine_model.onnx`, `score_model.onnx`) do.

## Engines

`prepare_models.sh` turns the two `.onnx` files into TensorRT engines (`.plan`) in
`docker/isaac_fp/models/` (`/models` in the container). Engines only work with the TensorRT
version **and** the GPU they were built for:

- never copy `.plan` files to another computer;
- after switching image (10.3 ↔ 10.13): delete the engines and build them again.

| Folder | Engines |
|---|---|
| `docker/isaac_fp/models/` | TensorRT 10.13, this computer's GPU (in use) |
| `docker/isaac_fp/models/trt10.3/` | laptop only: the old TensorRT 10.3 engines, kept for going back |

Always use the repo's `prepare_models.sh` (`bash $KARIS_REPO/docker/isaac_fp/prepare_models.sh`),
not `/opt/karis/prepare_models.sh`: the copy inside the image is older and does not build the
batch-1 engine (needed for `lean:=true`). It only builds missing engines; `--force` rebuilds all.

---

## 1. WSL: build the image (once per computer)

Needs `karis/isaac_fp:3.2` first (`bash docker/isaac_fp/build.sh`). Downloads about 1.25 GB from
NVIDIA's CUDA repo; the image gets about 5 GB larger.

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker images karis/isaac_fp                                   # 3.2 must exist
docker build --network host -f docker/isaac_fp/Dockerfile.trt10 -t karis/isaac_fp:3.2-trt10.13 docker/isaac_fp
docker run --rm --entrypoint /usr/src/tensorrt/bin/trtexec karis/isaac_fp:3.2-trt10.13 --version   # TensorRT v101303
```

On the Eskilstuna PC the image is copied from the laptop instead (20 GB, no build there):
`Perception_on_PC_Zenoh.md`, A4.

## 2. Switch to it (once per computer)

Stop the Isaac helper (Ctrl+C). WSL:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
git pull --no-rebase
grep '^ISAAC_IMAGE' docker/.env                                # nothing, or ...3.2-trt10.13
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d --force-recreate isaac_fp
docker ps --format '{{.Names}}  {{.Image}}' | grep isaac       # karis_isaac_fp  karis/isaac_fp:3.2-trt10.13
```

(No `-f docker/compose.wsl.yaml` outside WSL, e.g. on the PC.)

Isaac terminal (`docker compose -f docker/compose.yaml exec isaac_fp bash`), new engines:

```bash
source $KARIS_WS/karis_ros_env.sh
mkdir -p /models/trt10.3 && mv /models/*.plan /models/trt10.3/ 2>/dev/null   # keep the 10.3 engines aside
bash $KARIS_REPO/docker/isaac_fp/prepare_models.sh                          # 3 engines, a few minutes
ls /models/*.plan                                                           # refine, refine_batch1, score
```

Then as usual: `Perception_startup.md`, step 3 onwards.

## 3. Going back to TensorRT 10.3 (laptop only)

Not possible on a Blackwell GPU. On the laptop, if ever needed:

```bash
echo 'ISAAC_IMAGE=karis/isaac_fp:3.2' >> docker/.env
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d --force-recreate isaac_fp
```

and in the Isaac terminal swap the engines: move `/models/*.plan` to `/models/trt10.13/`, then
`mv /models/trt10.3/*.plan /models/`. To return to 10.13: remove the `ISAAC_IMAGE` line and
swap back.

## Messages you can ignore

| Message | Why |
|---|---|
| `TRT WARNING: The logger passed into createInferRuntime differs ...` | Several TensorRT engines in one process share one logger; information only. |
| GXF warnings `Using unregistered parameter` / `'min_size' parameter ... is deprecated` | From NVIDIA's graph files, same as with 10.3. |

## Troubleshooting

| Problem | Fix |
|---|---|
| `Unsupported SM: 0xc00` | The container runs the old image: step 2 (`ISAAC_IMAGE` not set to `...:3.2`, then `--force-recreate`). |
| Helper: `deserialize` / `engine ... incompatible` | Engines from the other TensorRT version or another GPU: `rm /models/*.plan`, run `prepare_models.sh` again. |
| `pull access denied for karis/isaac_fp` / `No such image: karis/isaac_fp:3.2-trt10.13` | Not built on this computer yet: step 1. |
| `prepare_models.sh`: `CUDA driver version is insufficient` (WSL) | Non-interactive shell: use `docker compose ... exec isaac_fp bash` (loads the WSL GPU path). |
| `bash: docker: command not found` | You are inside a container: `exit` to plain WSL. |

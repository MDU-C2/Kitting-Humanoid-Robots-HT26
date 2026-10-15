#!/usr/bin/env bash
# Downloads NVIDIA's FoundationPose models (ONNX, from NGC) and builds the TensorRT
# engines for THIS GPU. Run once inside the Isaac container (engines are kept in
# $KARIS_ISAAC_MODELS, a folder on the host). Takes a few minutes.
#     /opt/karis/prepare_models.sh            (again with --force after changing GPU / driver)
set -e
M="${KARIS_ISAAC_MODELS:-/models}"
NGC="https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx/files"
TRTEXEC=/usr/src/tensorrt/bin/trtexec
mkdir -p "$M"
[ "$1" = "--force" ] && rm -f "$M"/*.plan

for f in refine_model score_model; do
    if [ ! -s "$M/$f.onnx" ]; then
        echo "Downloading $f.onnx from NGC..."
        wget -q --show-progress -O "$M/$f.onnx.part" "$NGC/$f.onnx" && mv "$M/$f.onnx.part" "$M/$f.onnx"
    fi
done

SHAPE=160x160x6
if [ ! -s "$M/refine_trt_engine.plan" ]; then
    echo "Building the refine TensorRT engine (a few minutes)..."
    $TRTEXEC --onnx="$M/refine_model.onnx" --saveEngine="$M/refine_trt_engine.plan" --fp16 --skipInference \
        --minShapes=input1:1x$SHAPE,input2:1x$SHAPE --optShapes=input1:1x$SHAPE,input2:1x$SHAPE \
        --maxShapes=input1:42x$SHAPE,input2:42x$SHAPE > "$M/refine_trtexec.log" 2>&1 \
        || { echo "trtexec failed, see $M/refine_trtexec.log"; exit 1; }
fi
if [ ! -s "$M/score_trt_engine.plan" ]; then
    echo "Building the score TensorRT engine (a few minutes)..."
    $TRTEXEC --onnx="$M/score_model.onnx" --saveEngine="$M/score_trt_engine.plan" --fp16 --skipInference \
        --minShapes=input1:1x$SHAPE,input2:1x$SHAPE --optShapes=input1:1x$SHAPE,input2:1x$SHAPE \
        --maxShapes=input1:252x$SHAPE,input2:252x$SHAPE > "$M/score_trtexec.log" 2>&1 \
        || { echo "trtexec failed, see $M/score_trtexec.log"; exit 1; }
fi
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader > "$M/engines_built_for.txt" 2>/dev/null || true
echo "Models ready in $M: $(ls "$M" | tr '\n' ' ')"

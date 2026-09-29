from pathlib import Path # this should come with python installation, otherwise install
from ultralytics import YOLO # pip install ultralytics
# also run the command under this line for CUDA
# pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128

# config

DATASET_ROOT = Path(
    r"C:\Users\gusta\Desktop\Skolarbete\Skolarbete\KARISprojektBilder\KARIS.v1-test1.yolo26" # point this to your data folder, we used an exported folder from Roboflow
)

DATA_YAML = DATASET_ROOT / "data.yaml" 

MODEL = "yolo26m-seg.pt" # model m for medium version

# training settings, adjust for the hardware you train on
EPOCHS = 100
IMAGE_SIZE = 1280 # pixels
BATCH_SIZE = 2 # if you run out of GPU memory lower this
DEVICE = 0 # GPU training


# data augmentation occurs randomly on each image during training
AUGMENTATION_ON = True #TRUE for on, FALSE for off

# detailed explanations for augmentation values
# https://docs.ultralytics.com/guides/yolo26-training-recipe
# https://docs.ultralytics.com/guides/yolo-data-augmentation

# hue saturation value
HSV_H = 0.015
HSV_S = 0.4
HSV_V = 0.3

# geometric augmentations
DEGREES = 15.0 # +- deg
TRANSLATE = 0.1 # object can be moved % away from orig pos
SCALE = 0.3 # size scaling
PERSPECTIVE = 0.0005 # changes perspective through size change

# flipping
FLIPLR = 0.5 # chance to flip object horizontally
FLIPUD = 0.5 # chance to flip object vertically

# YOLO-specific, these augmentations can be heavy (hardware-wise)
MOSAIC = 0.5 # chance to combine many images into one
MIXUP = 0.0 # chance to blend images into one
COPY_PASTE = 0.0 # chance to take objects from one img into another


# training

def train():

    if not DATA_YAML.exists():
        raise FileNotFoundError(
            f"Could not find dataset configuration:\n{DATA_YAML}"
        )

    print("Starting training")
    print(f"Model:   {MODEL}")
    print(f"Dataset: {DATA_YAML}")
    print(f"Epochs:  {EPOCHS}")
    print(f"Image:   {IMAGE_SIZE}")
    print(f"Batch:   {BATCH_SIZE}")

    # load model
    model = YOLO(MODEL)

    if AUGMENTATION_ON:

        print("Augmentation on")

        results = model.train(
            data=str(DATA_YAML),

            epochs=EPOCHS,
            imgsz=IMAGE_SIZE,
            batch=BATCH_SIZE,
            device=DEVICE,

            project="runs/segment",
            name="karis_yolo26m",

            hsv_h=HSV_H,
            hsv_s=HSV_S,
            hsv_v=HSV_V,
            degrees=DEGREES,
            translate=TRANSLATE,
            scale=SCALE,
            perspective=PERSPECTIVE,
            fliplr=FLIPLR,
            flipud=FLIPUD,
            mosaic=MOSAIC,
            mixup=MIXUP,
            copy_paste=COPY_PASTE,
        )

    else:  # No augmentation case

        print("Augmentation off")

        results = model.train(
            data=str(DATA_YAML),

            epochs=EPOCHS,
            imgsz=IMAGE_SIZE,
            batch=BATCH_SIZE,
            device=DEVICE,

            project="runs/segment",
            name="karis_yolo26m",

            hsv_h=0.0,
            hsv_s=0.0,
            hsv_v=0.0,
            degrees=0.0,
            translate=0.0,
            scale=0.0,
            perspective=0.0,
            fliplr=0.0,
            flipud=0.0,
            mosaic=0.0,
            mixup=0.0,
            copy_paste=0.0,
        )

    return results


# MAIN

if __name__ == "__main__":
    train()
from ultralytics import YOLO

# Load model (YOLOv8n pretrained weights)
model = YOLO("yolov8n.pt")

# Train
model.train(
    data="data.yaml",
    epochs=100,
    imgsz=(640, 480),
    rect=True,
    batch=16,
    device=3,
    mosaic=1.0,
    mixup=0.2,
    # blur=0.2,
    # noise=0.1,
    hsv_h=0.015,       # hue
    hsv_s=0.7,         # saturation
    hsv_v=0.4,         # brightness
    fliplr=0.0,        # disable left-right flip
    flipud=0.0,        # disable up-down flip
    name="aug_21"
)

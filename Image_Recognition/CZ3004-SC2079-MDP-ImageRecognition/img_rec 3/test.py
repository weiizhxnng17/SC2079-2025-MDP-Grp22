from ultralytics import YOLO

# Load your trained model
# model = YOLO("img_rec 3/runs/detect/aug_21/weights/best(125epochs).pt")
model = YOLO(r"C:\Users\envy\OneDrive\Documents\MDP2025\CZ3004-SC2079-MDP-ImageRecognition\img_rec 3\runs\detect\aug_21\weights\best(125epochs).pt")


# model.predict(source=0, show=True)


model.predict(
    source=r"C:\Users\envy\OneDrive\Documents\MDP2025\own_results\raw_image_One-11_1760652471.jpg",
    project="results",
    name="heuristic_test_x",
    save=True
)

print(model.names)
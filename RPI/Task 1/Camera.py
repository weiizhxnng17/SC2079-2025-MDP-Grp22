import base64
import json
import os
from typing import Dict, Any, Optional
from picamera import PiCamera
import cv2
import time
from datetime import datetime

FOLDER_PATH = "/home/pi/mdp-rpi/ImageCapture"
IMAGE_PREPROCESSED_FOLDER_PATH = "/home/pi/mdp-rpi/ImagePreProcessed"

def capture(img_pth: str) -> None:
    """
    Capture image using PiCamera and save it to the specified path.

    Parameters:
        img_pth (str): Path to save the captured image.
    """
    camera = PiCamera()
    print(img_pth)
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    camera.capture(image_save_location)
    camera.close()
    print("[Camera] Image captured")

def preprocess_img(img_pth: str) -> None:
    """
    Read image, resize it, and save the resized image.

    Parameters:
        img_pth (str): Path of the image to be preprocessed.
    """
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    img = cv2.imread(image_save_location)

    resized_img = cv2.resize(img, (640, 480))  # (Width, Height) because we trained our dataset on 640x480 images
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    cv2.imwrite(image_save_location, resized_img)
    print("[Camera] Image preprocessing complete")

def get_image(final_image: bool = False, obstacle_id: Optional[int] = None) -> bytes:
    """
    Capture an image, preprocess it, and return a JSON message with the encoded image.

    Returns:
        bytes: Encoded JSON message containing image data.
    """
    # Capture the exact time of image capture
    capture_time = time.time()
    timestamp = int(capture_time)

    if obstacle_id is not None:
        # For SNAP images: Use permanent filename with timestamp and obstacle ID
        img_pth = f"{timestamp}_{obstacle_id}.jpg"
    else:
        # For final_image or other: Use temporary datetime-based filename
        formatted_time = datetime.fromtimestamp(capture_time).strftime('%d-%m_%H-%M-%S.%f')[:-3]
        img_pth = f"img_{formatted_time}.jpg"

    # Capture and preprocess the image
    capture(img_pth)
    preprocess_img(img_pth)

    # Construct a message with the encoded image
    encoded_string = ""
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    if os.path.isfile(image_save_location):
        with open(image_save_location, "rb") as img_file:
            encoded_string = base64.b64encode(img_file.read()).decode('utf-8')

    # Create a JSON message containing the image data, filename, and timestamp
    message: Dict[str, Any] = {
        "type": "IMAGE_TAKEN",
        "final_image": final_image,
        "data": {
            "image": encoded_string,
            "filename": img_pth,
            "timestamp": timestamp
        }
    }
    return json.dumps(message).encode("utf-8")
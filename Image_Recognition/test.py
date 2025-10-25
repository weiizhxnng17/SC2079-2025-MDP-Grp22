import requests

# --- Flask server URL ---
url = "http://localhost:5000/imageyolo"

# --- Path to your image file ---
image_path = "C:\\Users\\envy\\OneDrive\\Documents\\MDP2025\\CZ3004-SC2079-MDP-ImageRecognition\\runs\\opencv\\1758197831_Eight.jpg"   # change this to your actual image file

# --- Send multipart/form-data request ---
with open(image_path, "rb") as img_file:
    files = {"file": (image_path, img_file, "image/jpeg")}
    response = requests.post(url, files=files)

# --- Print response ---
print("Status code:", response.status_code)
try:
    print("Response JSON:", response.json())
except Exception:
    print("Raw response:", response.text)


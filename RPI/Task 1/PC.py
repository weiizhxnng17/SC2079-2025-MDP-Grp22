import socket
import json
from queue import Queue
import requests
import base64
from rpi_config import *

# External services
ALGO_URL = "http://192.168.22.25:5050/path"
YOLO_URL = "http://192.168.22.25:5000/imageyolo"

# Tiny 1x1 PNG (red pixel) as base64, used for testing if no image provided
DUMMY_BASE64_IMG = (
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVQ"
    "ImWNgYGBgAAAABAABJzQnCgAAAABJRU5ErkJggg=="
)

class PCInterface:
    def __init__(self, RPiMain, task2):
        self.RPiMain = RPiMain
        self.host = RPI_IP          # This should be the algo PC’s IP if we want to connect outward
        self.port = PC_PORT         # Algo PC must be listening on this port
        self.client_socket = None
        self.msg_queue = Queue()
        self.send_message = False
        self.obs_id = 1
        self.task2 = task2

    def connect(self):
        if self.client_socket is not None:
            self.disconnect()

        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"[PC] Trying to connect to server at {self.host}:{self.port} ...")
            self.client_socket.connect((self.host, self.port))
            self.send_message = True
            print("[PC] Connected successfully to PC server.")
        except socket.error as e:
            print("[PC] ERROR: Could not connect -", e)
            self.client_socket = None
            self.send_message = False

    def disconnect(self):
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                self.send_message = False
                print("[PC] Disconnected from PC.")
        except Exception as e:
            print("[PC] ERROR on disconnect:", e)

    def reconnect(self):
        self.disconnect()
        self.connect()

    def listen(self):
        while True:
            try:
                if not self.client_socket:
                    continue

                # Read length-prefixed message
                length_bytes = self.client_socket.recv(4)
                if not length_bytes:
                    print("[PC] Server closed connection.")
                    self.reconnect()
                    continue

                message_length = int.from_bytes(length_bytes, "big")
                message = self.client_socket.recv(message_length)

                if not message:
                    print("[PC] Empty message, reconnecting...")
                    self.reconnect()
                    continue

                decoded_msg = message.decode("utf-8")
                parsed_msg = json.loads(decoded_msg)
                msg_type = parsed_msg.get("type")
                print("[PC] Read:", decoded_msg[:MSG_LOG_MAX_SIZE])

                if msg_type == "FASTEST_PATH":
                    self.handle_fastest_path(parsed_msg)

                elif msg_type == "IMAGE_TAKEN":
                    self.handle_image(parsed_msg)

                else:
                    print("[PC] Unknown msg type:", msg_type)

            except Exception as e:
                print("[PC] ERROR in listen:", e)
                self.reconnect()

    def handle_fastest_path(self, parsed_msg):
        try:
            payload = parsed_msg["data"]
            resp = requests.post(ALGO_URL, json=payload).json()
            print("[PC] Algo response:", json.dumps(resp, indent=2))

            # Forward commands to STM only if STM exists
            if resp["data"]["commands"] and hasattr(self.RPiMain, "STM"):
                nav_msg = {
                    "type": "NAVIGATION",
                    "data": {
                        "commands": resp["data"]["commands"],
                        "path": resp["data"]["path"]
                    }
                }
                self.RPiMain.STM.msg_queue.put(json.dumps(nav_msg).encode("utf-8"))

            # Forward path to Android if available
            if hasattr(self.RPiMain, "Android"):
                self.RPiMain.forward_algo_path_to_android(resp["data"]["path"])

            # Send full response back to the connected PC
            self.msg_queue.put(json.dumps(resp).encode("utf-8"))

        except Exception as e:
            print("[PC] ERROR handling FASTEST_PATH:", e)

    def handle_image(self, parsed_msg):
        try:
            image_b64 = parsed_msg["data"].get("image", "")

            # If empty, replace with dummy image
            if not image_b64:
                print("[PC] No image provided, using dummy image instead")
                image_b64 = DUMMY_BASE64_IMG

            img_bytes = base64.b64decode(image_b64)

            # Use filename from message for YOLO upload (e.g., <timestamp>_<id>.jpg); fallback to "capture.jpg"
            filename = parsed_msg["data"].get("filename", "capture.jpg")
            # Strip .jpg extension for YOLO to parse obstacle_id cleanly (e.g., "1759407893_1.jpg" -> "1759407893_1")
            yolo_filename = filename.replace('.jpg', '')
            files = {"file": (yolo_filename, img_bytes, "image/jpeg")}
            resp = requests.post(YOLO_URL, files=files).json()
            print("[PC] Image rec response:", resp)

            # Forward results to Android only if Android exists
            if hasattr(self.RPiMain, "Android"):
                self.RPiMain.Android.msg_queue.put(json.dumps({
                    "type": "IMAGE_RESULTS",
                    "data": resp
                }).encode("utf-8"))

            # Handle task2 arrow logic only if STM exists
            if self.task2 and hasattr(self.RPiMain, "STM"):
                direction = "FIRSTLEFT" if resp.get("image_id") == "39" else "FIRSTRIGHT"
                nav_msg = {"type": "NAVIGATION", "data": {"commands": [direction], "path": []}}
                self.RPiMain.STM.msg_queue.put(json.dumps(nav_msg).encode("utf-8"))

            # Send results back to PC
            self.msg_queue.put(json.dumps(resp).encode("utf-8"))

        except Exception as e:
            print("[PC] ERROR handling IMAGE_TAKEN:", e)

    def send(self):
        while True:
            if self.send_message:
                message = self.msg_queue.get()
                if isinstance(message, bytes):
                    message = message.decode("utf-8")

                try:
                    packet = self.prepend_msg_size(message)
                    self.client_socket.sendall(packet)
                    print("[PC] Sent:", message[:100])
                except Exception as e:
                    print("[PC] ERROR send:", e)
                    self.reconnect()

    def prepend_msg_size(self, message: str) -> bytes:
        b = message.encode("utf-8")
        return len(b).to_bytes(4, "big") + b
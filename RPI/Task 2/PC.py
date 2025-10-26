import json
import socket
import requests
import time
from queue import Queue
from rpi_config import *

# External services
YOLO_URL = "http://192.168.22.23:5000/imageyolo"

class PCInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.msg_queue = Queue()
        self.client_socket = None
        self.send_message = False
        self.host = RPI_IP
        self.port = PC_PORT
        self.y_dists = []
        self.x_dists = []  # New array to store x distances
        self.second_arrow = None
        self.obs_id = 1

    def connect(self):
        # No connection attempt since algo server is not needed
        self.client_socket = None
        self.send_message = False
        print("[PC] Connection to server disabled (algo server not required).")

    def disconnect(self):
        if self.client_socket is not None:
            self.client_socket.close()
            self.client_socket = None
            self.send_message = False
            print("[PC] Disconnected from PC server.")

    def listen(self):
        while self.send_message:
            try:
                message = self.client_socket.recv(1024).decode("utf-8")
                if message:
                    print("[PC] Received from PC:", message)
                    parsed_msg = json.loads(message)
                    self.handle_message(parsed_msg)
            except Exception as e:
                print("[PC] ERROR: Failed to receive from PC -", e)
                self.disconnect()
                break

    def send(self):
        while self.send_message:
            try:
                message = self.msg_queue.get()
                if self.client_socket is not None:
                    self.client_socket.send(message.encode("utf-8"))
                    print("[PC] Sent to PC:", message)
            except Exception as e:
                print("[PC] ERROR: Failed to send to PC -", e)
                self.disconnect()
                break

    def handle_message(self, parsed_msg):
        message_type = parsed_msg["type"]
        if message_type == "FASTEST_PATH" or message_type == "START_TASK":
            self.handle_fastest_path(parsed_msg)
        elif message_type == "IMAGE":
            self.handle_image(parsed_msg)

    def handle_fastest_path(self, parsed_msg):
        try:
            # Hardcoded commands for Task 2, including new IR commands
            commands = ["YF100", "SNAP1", "YF100", "SNAP2", "FIN"]
            resp = {
                "data": {
                    "commands": commands,
                    "path": []
                }
            }
            print("[PC] Generated Task 2 commands:", json.dumps(resp, indent=2))

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

    def handle_y_dist(self, dist):
        self.y_dists.append(dist)
        print(f"[PC] Stored y_dist: {dist}, Current y_dists: {self.y_dists}")

    def handle_x_dist(self, dist):
        self.x_dists.append(dist)
        print(f"[PC] Stored x_dist: {dist}, Current x_dists: {self.x_dists}")

    def get_arrow_direction(self, message):
        import base64
        try:
            image_b64 = message["data"].get("image", "")
            if not image_b64:
                print("[PC] No image provided in message; skipping")
                return None

            img_bytes = base64.b64decode(image_b64)
            filename = message["data"].get("filename", "capture.jpg")
            yolo_filename = filename.replace('.jpg', '')  # Strip .jpg for YOLO to parse obstacle_id

            files = {"file": (yolo_filename, img_bytes, "image/jpeg")}
            response = requests.post(YOLO_URL, files=files, timeout=10).json()
            print("[PC] YOLO response:", json.dumps(response, indent=2))

            # Determine direction based on image_id (per your plan: 39=left, 38=right)
            image_id = response.get("image_id")
            if image_id not in ["38", "39"]:
                print("[PC] ERROR: Invalid image_id from YOLO:", image_id)
                return None

            return image_id

        except Exception as e:
            print("[PC] ERROR getting arrow direction:", e)
            return None

    def handle_image(self, message):
        # This method is retained but not used in Task 2 since no socket connection
        pass  # Or remove if not needed

    def handle_fin(self):
        if self.y_dists and self.x_dists and self.second_arrow:
            print(f"[PC] Initiating return to carpark with y_dists: {self.y_dists}, x_dists: {self.x_dists}, second_arrow: {self.second_arrow}")
            commands = self.get_commands_to_carpark()
            if commands:
                nav_msg = {"type": "NAVIGATION", "data": {"commands": commands, "path": []}}
                self.RPiMain.STM.msg_queue.put(json.dumps(nav_msg).encode("utf-8"))
                print("[PC] Sent return commands to STM:", commands)
            else:
                print("[PC] ERROR: No valid return commands generated.")
        else:
            print("[PC] ERROR: Missing y_dists or x_dists or second_arrow for return to carpark.")

    def get_commands_to_carpark(self):
        if not self.y_dists or not self.x_dists or not self.second_arrow:
            return []

        movement_list = []
        total_y_dist = sum(self.y_dists)
        if self.second_arrow == 'L':
            y_adjustment = total_y_dist + 5
            x_adjustment = self.x_dists[1] - 55
        elif self.second_arrow == 'R':
            y_adjustment = total_y_dist + 5
            x_adjustment = self.x_dists[1] - 59
        else:
            print("[PC] ERROR: Invalid second_arrow -", self.second_arrow)
            return []
        
        if self.second_arrow == 'R':
            movement_list.append(f"FW{y_adjustment:03d}")
            movement_list.append("FL090")
            if x_adjustment < 0:
                x_adjustment = -x_adjustment
                movement_list.append(f"BW{x_adjustment:03d}")
            else:
                movement_list.append(f"FW{x_adjustment:03d}")
            movement_list.append("FR090")
            movement_list.append("YF100")
            movement_list.append("FW012")
        elif self.second_arrow == 'L':
            movement_list.append(f"FW{y_adjustment:03d}")
            movement_list.append("FR090")
            if x_adjustment < 0:
                x_adjustment = -x_adjustment
                movement_list.append(f"BW{x_adjustment:03d}")
            else:
                movement_list.append(f"FW{x_adjustment:03d}")
            movement_list.append("FL090")
            movement_list.append("YF100")
            movement_list.append("FW012")

        return movement_list
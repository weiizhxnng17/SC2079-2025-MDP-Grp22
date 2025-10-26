from queue import Queue
import bluetooth as bt
import socket
import sys
import subprocess
import json
import time
from rpi_config import *

class AndroidInterface:
    """
    Represents the interface between the Raspberry Pi and an Android device over Bluetooth.

    Args:
    - RPiMain: Instance of the RPiMain class.

    Attributes:
    - RPiMain (RPiMain): Instance of the RPiMain class.
    - host (str): IP address of the Raspberry Pi.
    - uuid (str): Bluetooth UUID for the service.
    - msg_queue (Queue): Queue for storing messages.
    - socket (BluetoothSocket): Bluetooth socket for communication.
    - port (int): Port number for the socket connection.
    - client_socket (BluetoothSocket): Socket for communication with the connected Android device.
    - client_info (tuple): Information about the connected Android client.
    """
    def __init__(self, RPiMain):
        # Initialize AndroidInterface with RPiMain instance
        self.RPiMain = RPiMain
        self.host = RPI_IP
        self.uuid = BT_UUID
        self.msg_queue = Queue()
        self.socket = None  # Initialize socket as None
        self.client_socket = None  # Initialize client_socket as None

    def connect(self):
        # Grant permission for Bluetooth access
        subprocess.run("sudo chmod o+rw /var/run/sdp", shell=True) 

        # Establish and bind socket
        self.socket = bt.BluetoothSocket(bt.RFCOMM)
        print("[Android] BT socket established successfully.")
    
        try:
            self.socket.bind(("", 2))  # Use fixed RFCOMM channel 1
            self.port = 2
            print("[Android] BT socket bound to RFCOMM channel 1")
            
            # Ensure adapter is discoverable
            subprocess.run("sudo hciconfig hci0 piscan", shell=True)
            self.socket.listen(128)
            
            # Advertise Bluetooth service
            bt.advertise_service(self.socket, "Group29-Server", service_id=self.uuid, 
                               service_classes=[self.uuid, bt.SERIAL_PORT_CLASS], 
                               profiles=[bt.SERIAL_PORT_PROFILE])
            print("[Android] Waiting for Android connection...")
            time.sleep(3)  # Delay for SDP propagation

            self.client_socket, self.client_info = self.socket.accept()
            print("[Android] Accepted connection from", self.client_info)
            
        except socket.error as e:
            print("[Android] ERROR: connection failed -", str(e))
            self.disconnect()  # Clean up on failure
            raise  # Re-raise to allow caller to handle

    def disconnect(self):
        # Close the Bluetooth sockets
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            if self.socket:
                bt.stop_advertising(self.socket)  # Stop SDP advertising
                self.socket.close()
                self.socket = None
            print("[Android] Disconnected from Android successfully.")
        except Exception as e:
            print("[Android] ERROR: Failed to disconnect from Android -", str(e))

    def reconnect(self):
        # Disconnect and then connect again
        self.disconnect()
        self.connect()

    def listen(self):
        # Continuously listen for messages from Android
        buffer = b""  # Buffer to accumulate incoming data
        last_received = time.time()
        MAX_BUFFER_SIZE = 16384  # 16KB maximum buffer size
        TIMEOUT = 5  # 5 seconds timeout for incomplete messages
        while True:
            try:
                # Receive data from the Bluetooth socket
                chunk = self.client_socket.recv(BT_BUFFER_SIZE)
                if not chunk:
                    print("[Android] Android disconnected remotely. Reconnecting...")
                    self.reconnect()
                    buffer = b""
                    last_received = time.time()
                    continue

                buffer += chunk
                last_received = time.time()
                print("[Android] Received chunk of size:", len(chunk), "Total buffer size:", len(buffer))
                print("[Android] Buffer contents (hex):", buffer.hex()[:200])  # Log first 200 hex chars for debugging

                if len(buffer) > MAX_BUFFER_SIZE:
                    print("[Android] ERROR: Buffer size exceeds maximum, clearing buffer")
                    buffer = b""
                    last_received = time.time()
                    continue

                try:
                    decodedMsg = buffer.decode("utf-8")
                    parsedMsg = json.loads(decodedMsg)
                    msg_type = parsedMsg["type"]
                    print("[Android] Read from Android:", decodedMsg)  # Print full JSON for debugging

                    # Route messages to the appropriate destination
                    if msg_type == 'NAVIGATION':
                        self.RPiMain.STM.msg_queue.put(buffer)
                    elif msg_type == 'START_TASK' or msg_type == 'FASTEST_PATH':
                        # Directly call handler instead of queue (bypass socket)
                        self.RPiMain.PC.handle_fastest_path(parsedMsg)
                        print("[Android] Directly handled FASTEST_PATH/START_TASK via PC handler:", decodedMsg)

                    buffer = b""  # Clear buffer after successful parsing
                    last_received = time.time()

                except json.JSONDecodeError as e:
                    print("[Android] Partial JSON received, waiting for more data:", str(e))
                    if time.time() - last_received > TIMEOUT:
                        print("[Android] ERROR: Timeout waiting for complete JSON, clearing buffer")
                        buffer = b""
                        last_received = time.time()
                    continue  # Wait for more data
                except UnicodeDecodeError as e:
                    print("[Android] Partial UTF-8 sequence, waiting for more data:", str(e))
                    if time.time() - last_received > TIMEOUT:
                        print("[Android] ERROR: Timeout waiting for complete JSON, clearing buffer")
                        buffer = b""
                        last_received = time.time()
                    continue  # Wait for more data

            except (socket.error, IOError, ConnectionResetError) as e:
                print("[Android] ERROR:", str(e))
                self.reconnect()
                buffer = b""
                last_received = time.time()

    def send(self):
        # Continuously send messages to Android
        while True: 
            message = self.msg_queue.get()
            exception = True
            while exception: 
                try:
                    self.client_socket.sendall(message)
                    print("[Android] Write to Android: " + message.decode("utf-8")[:MSG_LOG_MAX_SIZE])
                except Exception as e:
                    print("[Android] ERROR: Failed to write to Android -", str(e))
                    self.reconnect()  # reconnect and resend
                else:
                    exception = False  # done sending, get next message
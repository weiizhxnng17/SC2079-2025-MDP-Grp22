import json
from queue import Queue
import re
import threading
import time
import serial
import glob
from Camera import get_image
from rpi_config import *
import time
from time import sleep

class STMInterface:
    def __init__(self, RPiMain, task2):
        # Initialize STMInterface with necessary attributes
        self.RPiMain = RPiMain
        self.baudrate = STM_BAUDRATE
        self.serial = None
        self.msg_queue = Queue()
        self.task2 = task2

    def connect(self):
        # Connect to STM by finding the first available ttyACM* port
        tty_ports = glob.glob('/dev/ttyACM*')
        if not tty_ports:
            print("[STM] ERROR: No ttyACM* ports found.")
            return

        for port in tty_ports:
            try:
                self.serial = serial.Serial(port, self.baudrate, write_timeout=0, timeout=5)
                print(f"[STM] Connected to STM successfully on {port}.")
                self.clean_buffers()
                return
            except Exception as e:
                print(f"[STM] Failed to connect to {port} - {str(e)}")
                continue
        print("[STM] ERROR: Failed to connect to any ttyACM* port.")
        self.serial = None
        if self.serial is None:
            print("[STM] WARNING: No serial connection established. Operations will fail.")

    def disconnect(self):
        # Disconnect from STM
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            print("[STM] Disconnected from STM.")

    def reconnect(self):
        # Reconnect to STM by closing the current connection and establishing a new one
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self.connect()

    def clean_buffers(self):
        # Reset input and output buffers of the serial communication
        if self.serial is not None and self.serial.is_open:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

    def listen(self):
        # Listen for messages from STM
        message = None
        try:
            if self.serial is not None and self.serial.is_open:
                message = self.serial.read().decode("utf-8")
                print("[STM] Read from STM:", message[:MSG_LOG_MAX_SIZE])
            else:
                print("[STM] ERROR: Serial connection not available.")
        except Exception as e:
            message = str(e)
            print("[STM] Listen error:", str(e))

        return message if message else None

    def wait_for_ack(self, timeout=5):
        print("[STM] No ACK required from STM")
        return True
        # Wait for ACK message from STM with a timeout
        #start_time = time.time()
        #while time.time() - start_time < timeout:
        #    message = self.listen()
        #    if message == STM_ACK_MSG:
        #        print("[STM] Received ACK from STM")
        #        return True
        #    time.sleep(0.1)
        #print("[STM] ERROR: No ACK received from STM within timeout.")
        #return False

    def wait_for_dist(self, timeout=15.0, line_timeout=0.1, prefix="YF"):
        if self.serial is None or not self.serial.is_open:
            print("[STM] Error: Serial connection not available.")
            return -1

        pattern = re.compile(rf"{re.escape(prefix)}\s*(\d+)")
        end_time = time.time() + timeout
        # use serial timeout for readline-like behavior
        old_timeout = getattr(self.serial, "timeout", None)
        self.serial.timeout = line_timeout
        try:
            buffer = ""
            while time.time() < end_time:
                # Try to read a line if device sends newline terminated messages
                try:
                    line = self.serial.readline().decode("utf-8", errors="replace")
                except AttributeError:
                    # If `readline` is not available, fall back to reading bytes
                    byte = self.serial.read(1)
                    if not byte:
                        time.sleep(0.01)
                        continue
                    line = byte.decode("utf-8", errors="replace")

                if not line:
                    # no data this iteration
                    time.sleep(0.01)
                    continue

                buffer += line
                # Try to find the prefix + digits anywhere in the accumulated buffer
                m = pattern.search(buffer)
                if m:
                    raw_message = m.group(0)  # Capture the full matched string (e.g., "YF59")
                    dist = int(m.group(1))
                    print(f"[STM] Received from STM: {raw_message}")
                    print(f"[STM] Received full distance: {dist}")
                    return dist

                # Keep buffer reasonably small to avoid memory growth
                if len(buffer) > 1024:
                    buffer = buffer[-512:]
            print(f"[STM] Timeout waiting for distance. Last buffer: '{buffer}'")
            return -1
        except serial.SerialException as e:
            print(f"[STM] Serial error: {e}")
            return -1
        finally:
            # restore previous timeout
            self.serial.timeout = old_timeout

    def wait_for_xdist(self, timeout=15.0, line_timeout=0.1, prefix="IR"):
        if self.serial is None or not self.serial.is_open:
            print("[STM] Error: Serial connection not available.")
            return -1

        pattern = re.compile(rf"{re.escape(prefix)}\s*(\d+)")
        end_time = time.time() + timeout

        old_timeout = getattr(self.serial, "timeout", None)
        self.serial.timeout = line_timeout

        try:
            buffer = ""
            while time.time() < end_time:
                try:
                    # Try reading a full line if possible
                    line = self.serial.readline().decode("utf-8", errors="replace")
                except AttributeError:
                    # If readline() isn't supported, fall back to byte-by-byte
                    byte = self.serial.read(1)
                    if not byte:
                        time.sleep(0.01)
                        continue
                    line = byte.decode("utf-8", errors="replace")

                if not line:
                    time.sleep(0.01)
                    continue

                buffer += line
                match = pattern.search(buffer)
                if match:
                    raw_message = match.group(0)  # Capture the full matched string (e.g., "IR32")
                    dist = int(match.group(1))
                    print(f"[STM] Received from STM: {raw_message}")
                    print(f"[STM] Received x distance: {dist}")
                    return dist

                if len(buffer) > 1024:
                    buffer = buffer[-512:]

            print(f"[STM] Timeout waiting for x distance. Last buffer: '{buffer}'")
            return -1

        except serial.SerialException as e:
            print(f"[STM] Serial error: {e}")
            return -1

        finally:
            # Restore original timeout
            self.serial.timeout = old_timeout

    def send(self):
        while True:
            try:
                message_byte = self.msg_queue.get()
                message_str = message_byte.decode("utf-8")
                message = json.loads(message_str)
                message_type = message["type"]
            except Exception as e:
                print("[STM] ERROR: Failed to process message -", str(e))
                self.reconnect()
                continue

            if message_type == "NAVIGATION":
                # Print the full NAVIGATION JSON before processing commands
                print("[STM] Sending NAVIGATION commands:", json.dumps(message, indent=2))

                # Send path to Android
                self.send_path_to_android(message)

                # Convert/adjust turn or obstacle routing commands
                commands = self.adjust_commands(message["data"]["commands"])

                # Process commands with dynamic insertion for routing after SNAP
                i = 0
                while i < len(commands):
                    command = commands[i]
                    if command.startswith("SNAP"):
                        # Extract obstacle_id from SNAP command (e.g., SNAP1 -> 1)
                        sleep(2)
                        obstacle_id = int(command[4:])
                        image_msg = get_image(final_image=False, obstacle_id=obstacle_id)
                        # Get arrow direction from PC (image_id "39" or "38")
                        image_json = json.loads(image_msg.decode("utf-8"))
                        image_id = self.RPiMain.PC.get_arrow_direction(image_json)
                        if image_id is None:
                            print("[STM] ERROR: Failed to get arrow direction, skipping routing")
                            i += 1
                            continue
                        is_left = image_id == "39"
                        prefix = "FIRST" if obstacle_id == 1 else "SECOND"
                        direction = prefix + "LEFT" if is_left else prefix + "RIGHT"
                        if prefix == "SECOND":
                            self.RPiMain.PC.second_arrow = 'L' if is_left else 'R'
                        # Adjust the direction command into STM commands
                        adj_commands = self.adjust_commands([direction])
                        # Insert and immediately execute the adjusted commands
                        for adj_cmd in adj_commands:
                            print(f"[STM] Sending command to STM: {adj_cmd}")
                            self.write_to_stm(adj_cmd)
                        i += 1  # Move past the SNAP to the next command
                        continue
                    elif command == "FIN":
                        # Handle FIN
                        self.RPiMain.PC.handle_fin()
                    else:
                        print(f"[STM] Sending command to STM: {command}")
                        self.write_to_stm(command)
                    i += 1

            else:
                print("[STM] Unknown message type:", message_type)

    def send_path_to_android(self, message):
        # Forward path to Android if available
        if "path" in message["data"] and message["data"]["path"]:
            self.RPiMain.Android.msg_queue.put(self.create_path_message(message["data"]["path"]))

    def is_valid_command(self, command):
        return re.match(STM_NAV_COMMAND_FORMAT, command)

    def is_turn_command(self, command):
        return self.is_valid_command(command) and re.match(r"^[FB][RL][0-9]{3}$", command)

    def is_obstacle_routing_command(self, command):
        return command in STM_OBS_ROUTING_MAP

    def is_straight_command(self, command):
        return self.is_valid_command(command) and re.match("^[FB][WS][0-9]{3}$", command)

    def is_validturn_command(self, command):
        return self.is_valid_command(command) and (command.startswith("F") or command.startswith("B"))

    def combine_straight_commands(self, straight_commands):
        dir_dict = {"FW": 1, "BW": -1, "FS": 1, "BS": -1}
        total = 0
        is_slow = any(c.startswith("FS") or c.startswith("BS") for c in straight_commands)
        for c in straight_commands:
            dir = c[:2]
            val = int(c[2:])
            total += dir_dict.get(dir, 0) * val

        if total > 0:
            prefix = "FS" if is_slow else "FW"
            return f"{prefix}{abs(total):03d}"
        elif total < 0:
            prefix = "BS" if is_slow else "BW"
            return f"{prefix}{abs(total):03d}"
        else:
            return None

    def add_command(self, final, new):
        if self.is_straight_command(new) and (len(final) > 0 and self.is_straight_command(final[-1])):
            prev = final.pop(-1)
            combined = self.combine_straight_commands([prev, new])
            if combined is not None:
                final.append(combined)
            else:
                final.append(prev)
                final.append(new)
        else:
            final.append(new)
        return final

    def adjust_commands(self, commands):
        def adjust_turn_command(command):
            # Adjust turn commands based on location
            return STM_COMMAND_ADJUSTMENT_MAP.get(command, [command])

        def adjust_obstacle_routing_command(command):
            # Adjust obstacle routing commands
            routing = STM_OBS_ROUTING_MAP.get(command, [command])
            return routing

        final_commands = []
        for i in range(len(commands)):
            command = commands[i].upper()
            adj_commands = []
            if self.is_turn_command(command):
                adj_commands = adjust_turn_command(command)
            elif self.is_obstacle_routing_command(command):
                adj_commands = adjust_obstacle_routing_command(command)
            else:
                final_commands = self.add_command(final_commands, command)
            for c in adj_commands:
                final_commands = self.add_command(final_commands, c)
        return final_commands

    def create_path_message(self, path):
        # Create a JSON-encoded message for path information
        message = {
            "type": "PATH",
            "data": {
                "path": path
            }
        }
        return json.dumps(message).encode("utf-8")

    def write_to_stm(self, command):
        if re.match(STM_YDIST_COMMAND_FORMAT, command):
            self.clean_buffers()  # Clear buffers before sending
            self.serial.write((command + '\n').encode())
            #time.sleep(3)  # Increased buffer time to allow STM to respond and measure
            dist = self.wait_for_dist()
            if dist >= 0:
                self.RPiMain.PC.handle_y_dist(dist)
            else:
                print(f"[STM] Invalid distance received for {command}: {dist}")
            return
        elif command in ["IRL", "IRR"]:
            self.clean_buffers()  # Clear buffers before sending
            self.serial.write((command + '\n').encode())
            #time.sleep(3)  # Increased buffer time to allow STM to respond and measure
            dist = self.wait_for_xdist()
            if dist >= 0:
                self.RPiMain.PC.handle_x_dist(dist)
            else:
                print(f"[STM] Invalid x distance received for {command}: {dist}")
            return

        self.serial.write((command + '\n').encode())
        if not re.match(STM_YDIST_COMMAND_FORMAT, command) and not re.match(STM_XDIST_COMMAND_FORMAT, command):
            self.wait_for_ack()
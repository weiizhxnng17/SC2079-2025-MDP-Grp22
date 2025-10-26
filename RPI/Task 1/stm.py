import json
from queue import Queue
import re
import threading
import time
import serial
import glob
from Camera import get_image  # Import Camera module
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
        # Task 2: return to carpark
        self.second_arrow = None
        self.xdist = 0
        self.ydist = 0
        self.move_counter = 0
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
        # Reset input and output buffers of the serial connection
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
        # Wait for ACK message from STM with a timeout
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.listen()
            if message == STM_ACK_MSG:
                print("[STM] Received ACK from STM")
                return True
            time.sleep(0.1)
        print("[STM] ERROR: No ACK received from STM within timeout.")
        return False

    def wait_for_dist(self):
        # Wait for distance value from STM
        message = self.listen()
        try:
            dist = int(message)
            return dist
        except ValueError:
            print("[STM] ERROR: Invalid distance received -", message)
            return -1

    def send(self):
        # Send commands to STM based on the received messages from Android
        self.second_arrow = None
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
                # Send path to Android
                self.send_path_to_android(message)

                # Convert/adjust turn or obstacle routing commands
                commands: list[str] = self.adjust_commands(message["data"]["commands"])

                for idx, command in enumerate(commands):
                    if command.startswith("SNAP"):
                        # Extract obstacle ID from SNAP command (e.g., "SNAP1_C" -> 1)
                        sleep(3)
                        snap_parts = command.split('_')
                        obstacle_id = int(snap_parts[0][4:])  # 'SNAP1' -> 1
                        # Capture and send image to PC
                        capture_and_send_image_thread = threading.Thread(
                            target=self.send_image_to_pc,
                            kwargs={"final_image": False, "obstacle_id": obstacle_id},
                            daemon=True
                        )
                        capture_and_send_image_thread.start()
                        capture_and_send_image_thread.join()  # Wait for image processing
                    elif command == "FIN":
                        # End of command list, signal completion
                        print("[STM] Received FIN command. Ending navigation.")
                        completion_message = {
                            "type": "NAVIGATION_COMPLETE",
                            "data": {}
                        }
                        if hasattr(self.RPiMain, "Android"):
                            self.RPiMain.Android.msg_queue.put(json.dumps(completion_message).encode("utf-8"))
                        break  # Exit the command loop
                    else:
                        print("[STM] Writing to STM:", command)
                        self.write_to_stm(command)

                if self.second_arrow is not None:
                    self.return_to_carpark()
                    # Capture final image for Task 2
                    capture_and_send_image_thread = threading.Thread(
                        target=self.send_image_to_pc,
                        kwargs={"final_image": True, "obstacle_id": None},
                        daemon=True
                    )
                    capture_and_send_image_thread.start()
                    capture_and_send_image_thread.join()  # Wait for final image
                    print("[STM] DONE")
                    return

    def send_image_to_pc(self, final_image=False, obstacle_id=None):
        # Capture image and send to PC
        try:
            image_message = get_image(final_image=final_image, obstacle_id=obstacle_id)
            if hasattr(self.RPiMain, "PC"):
                # Directly call handler instead of queue (bypass socket)
                parsed_msg = json.loads(image_message.decode("utf-8"))
                self.RPiMain.PC.handle_image(parsed_msg)
                print("[STM] Directly handled IMAGE_TAKEN via PC handler, final_image =", final_image)
        except Exception as e:
            print("[STM] ERROR: Failed to capture or send image to PC -", str(e))

    def write_to_stm(self, command):
        # Hybrid: Write command to STM with original distance handling and ACK retry
        if self.serial is None or not self.serial.is_open:
            print("[STM] ERROR: No serial connection. Skipping write.")
            return

        exception = True
        while exception:
            try:
                self.clean_buffers()
                print("[STM] Sending command", command)
                # Append newline and encode
                command_with_newline = command + "\n"
                self.serial.write(command_with_newline.encode("utf-8"))
            except Exception as e:
                print("[STM] ERROR: Failed to write to STM -", str(e))
                self.reconnect()
            else:
                exception = False

        # Handle special cases after successful write
        if command == STM_GYRO_RESET_COMMAND:
            print("[STM] Waiting %ss for reset" % STM_GYRO_RESET_DELAY)
            time.sleep(STM_GYRO_RESET_DELAY)
        elif re.match(STM_XDIST_COMMAND_FORMAT, command):
            dist = self.wait_for_dist()
            if dist >= 0:
                if self.second_arrow == 'L':
                    if self.move_counter >= 0 and self.move_counter < 2:
                        self.xdist -= dist
                        print("[STM] updated XDIST =", self.xdist)
                        self.move_counter += 1
                    else:
                        self.xdist += dist
                        print("[STM] updated XDIST =", self.xdist)
                if self.second_arrow == 'R':
                    if self.move_counter == 1:
                        self.xdist -= dist
                    else:
                        self.xdist += dist
                        print("[STM] updated XDIST =", self.xdist)
                    self.move_counter += 1
            else:
                print("[STM] ERROR: failed to update XBDIST, received invalid value:", dist)
        elif re.match(STM_YDIST_COMMAND_FORMAT, command):
            dist = self.wait_for_dist()
            if dist >= 0:
                self.ydist += dist
                print("[STM] updated YDIST =", self.ydist)
            else:
                print("[STM] ERROR: failed to update YDIST, received invalid value:", dist)
        else:
            if not self.wait_for_ack():
                print("[STM] Retrying due to missing ACK...")
                self.reconnect()

        # Increment move counter for gyro reset (outside special cases)
        self.move_counter += 1
        if self.move_counter % STM_GYRO_RESET_FREQ == 0:
            self.write_to_stm(STM_GYRO_RESET_COMMAND)

    def is_valid_command(self, command):
        # Validate command format
        return re.match(STM_NAV_COMMAND_FORMAT, command)

    def send_path_to_android(self, message):
        # Send path to Android if present in message
        if "path" in message["data"]:
            path_message = self.create_path_message(message["data"]["path"])
            if hasattr(self.RPiMain, "Android"):
                self.RPiMain.Android.msg_queue.put(path_message)

    def adjust_commands(self, commands):
        # Adjust commands for turns and obstacle routing for smoother execution
        def is_turn_command(command):
            return self.is_valid_command(command) and re.match("^[FB][RL]090$", command)

        def adjust_turn_command(turn_command):
            return STM_COMMAND_ADJUSTMENT_MAP.get(turn_command, [turn_command])

        def is_obstacle_routing_command(command):
            return command in STM_OBS_ROUTING_MAP.keys()

        def adjust_obstacle_routing_command(obs_routing_command):
            if obs_routing_command.startswith("SECOND"):
                self.second_arrow = obs_routing_command[len("SECOND")]
                print("[STM] Saving second arrow as", self.second_arrow)
            return STM_OBS_ROUTING_MAP[obs_routing_command]

        def is_straight_command(command):
            return self.is_valid_command(command) and re.match("^[FB][WS][0-9]{3}$", command)

        def is_validturn_command(command):
            return self.is_valid_command(command) and (command.startswith("F") or command.startswith("B"))

        def combine_straight_commands(straight_commands):
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

        def add_command(final, new):
            if is_straight_command(new) and (len(final) > 0 and is_straight_command(final[-1])):
                prev = final.pop(-1)
                combined = combine_straight_commands([prev, new])
                if combined is not None:
                    final.append(combined)
                else:
                    final.append(prev)
                    final.append(new)
            else:
                final.append(new)
            return final

        final_commands = []
        for i in range(len(commands)):
            command = commands[i].upper()
            if not self.task2:
                final_commands = add_command(final_commands, command)
            else:
                adj_commands = []
                if is_turn_command(command):
                    adj_commands = adjust_turn_command(command)
                elif is_obstacle_routing_command(command):
                    adj_commands = adjust_obstacle_routing_command(command)
                else:
                    final_commands = add_command(final_commands, command)
                for c in adj_commands:
                    final_commands = add_command(final_commands, c)
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

    def return_to_carpark(self):
        # Execute the return to carpark procedure based on the obtained information
        print(f"[STM] Initiating return to carpark: XDIST = {self.xdist}, YDIST = {self.ydist}, ARROW = {self.second_arrow}")
        commands = self.get_commands_to_carpark()
        for command in commands:
            self.write_to_stm(command)

    def get_commands_to_carpark(self):
        # Calculate the path to return to the carpark based on the obtained information
        print(f"[STM] Calculating path to carpark...")
        movement_list = []

        if self.second_arrow == 'L':
            x_adjustment = self.xdist - 10
            y_adjustment = self.ydist + 107
        elif self.second_arrow == 'R':
            x_adjustment = self.xdist - 52
            y_adjustment = self.ydist + 45

        movement_list.append(f"FW{y_adjustment:03d}")
        if self.second_arrow == 'R':
            movement_list.append("FL090")
            if x_adjustment > 0:
                movement_list.append(f"FW{x_adjustment:03d}")
            else:
                movement_list.append(f"BW{abs(x_adjustment):03d}")
            movement_list.append("FR090")
            movement_list.append("VF100")
        elif self.second_arrow == 'L':
            movement_list.append("FR090")
            if x_adjustment > 0:
                movement_list.append(f"FW{x_adjustment:03d}")
            else:
                movement_list.append(f"BW{abs(x_adjustment):03d}")
            movement_list.append("FL090")
            movement_list.append("VF100")
        else:
            print("[STM] ERROR getting path to carpark, second arrow invalid -", self.second_arrow)

        print("[STM] Final path to carpark:", movement_list)
        return movement_list
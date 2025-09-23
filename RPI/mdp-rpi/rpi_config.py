# Configuration constants
# LOCATION = "OUT" # IN (indoors) / OUT (outdoors) / NONE (disable turn adjustment)
LOCATION = "NONE"

RPI_IP = "192.168.22.22"
MSG_LOG_MAX_SIZE = 150 # characters

# PC Interface
PC_PORT = 8888
PC_BUFFER_SIZE = 2048

# Camera Interface
NUM_IMAGES = 1

# Android Interface
BT_UUID = "00001101-0000-1000-8000-00805f9b34fb"
BT_BUFFER_SIZE = 2048

# STM Interface
STM_BAUDRATE = 115200
STM_ACK_MSG = "A"
STM_NAV_COMMAND_FORMAT = '^([FB][WS][0-9]{3}|[FB][RL]090)$' # Task 1: Updated for FR090, FL090, BR090, BL090
STM_NAV_COMMAND_FORMAT = '^(([FB][WS])|([UYV]F)|([FB][RL]090))[0-9]{3}$' # Task 2: Updated for FR090, FL090, BR090, BL090
STM_GYRO_RESET_COMMAND = "GYROR"
STM_GYRO_RESET_DELAY = 4 # time to wait for gyro reset
STM_GYRO_RESET_FREQ = 3 # number of obstacles before GRYO RESET command is sent

# Task 1: adjust commands for turns to correct turning radius to 30cm, as expected by PC-algo
STM_COMMAND_ADJUSTMENT_DICT = {
    "OUT": {
        # 90 degree turns: manually calibrated (updated to 3 digits)
        "FR090": ["FW007", "FR090", "BW008"],
        "FL090": ["FW007", "FL090", "BW011"],
        "BR090": ["FW009", "BR090", "BW009"],
        "BL090": ["FW009", "BL090", "BW006"],
        # 180 degree turns: manually calibrated (if needed; add similar)
        # "FR180": ["FW008", "FR180", "BW008"],  # Example, uncomment and adjust if supporting 180°
    },
    "IN": {
        # 90 degree turns: manually calibrated (updated to 3 digits)
        "FR090": ["FW008", "FR090", "BW008"],
        "FL090": ["FW006", "FL090", "BW011"],
        "BR090": ["FW009", "BR090", "BW009"],
        "BL090": ["FW010", "BL090", "BW006"],
        # 180 degree turns: manually calibrated (if needed)
    },
    "NONE": {}
}
STM_COMMAND_ADJUSTMENT_MAP = STM_COMMAND_ADJUSTMENT_DICT[LOCATION]

# Task 2: translate PC commands for moving around obstacles to STM_NAV_COMMAND_FORMAT
STM_OBS_ROUTING_MAP = {
    "FIRSTLEFT": ["FL090", "FR090", "FW005", "FR090", "FL090"],  # Updated to 90-degree turns
    "FIRSTRIGHT": ["FR090", "FL090", "FW005", "FL090", "FR090"],  # Updated to 90-degree turns
    "SECONDLEFT": ["FR090", "FW100", "BW100", "FW015",  # Updated to FR090, BL090, BR090
                   "BL090", "BW010", "BL090", "FW100",  # Updated to BL090
                   "BW200", "FW020",  # Unchanged
                   "BR090"],  # Updated to BR090
    "SECONDRIGHT": ["FR090", "FW100", "BW100",  # Updated to FR090
                    "FL090", "FL090", "FW100",  # Updated to FL090
                    "BW200",  # Unchanged
                    "FL090"]  # Updated to FL090
}
STM_XDIST_COMMAND_FORMAT = "^[IX][FB][0-9]{3}$"  # Updated from [IJ][FB] to [IX][FB] (assuming I/X for X-dist)
STM_YDIST_COMMAND_FORMAT = "^YF[0-9]{3}$"
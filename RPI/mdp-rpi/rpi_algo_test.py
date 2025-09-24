#!/usr/bin/env python3
"""
RPi <-> Algo bridge  (fully commented)

What this file does, at a glance:
- Talks to your **Algo (path planning) server** on the PC (HTTP):
    - POST /path (or /path_debug) with robot + obstacles
    - Receives "commands" and "path" and rebroadcasts as a NAVIGATION message

- Talks to your **Image Recognition server** (HTTP) on a DIFFERENT host:
    - On every `SNAP*` command from the Algo, load a LOCAL image on the RPi
      (/home/pi/mdp-rpi/camera/images/image.jpg), rename it so that the
      obstacle_id is in the 2nd underscore token, and upload it to either:
        - POST /imageyolo   (default), or
        - POST /imageopencv (if IMG_BACKEND=opencv)
    - Receives {"obstacle_id": "...", "image_id": "..."} and rebroadcasts
      as an IMAGE_RESULTS message (mapping image_id -> img_id as your RPi expects)

- **Preserves your legacy message shapes** so the rest of your RPi stack (Android/STM)
  can be integrated without changing their expectations:
    NAVIGATION: {"type":"NAVIGATION","data":{"commands":[...],"path":[...]}}
    IMAGE_RESULTS: {"type":"IMAGE_RESULTS","data":{"obstacle_id":"..","img_id":".."}}

- Provides clear TODOs where to:
    - send movement commands to STM (UART),
    - receive ACK/odometry back from STM,
    - publish updates to Android (MQTT/websocket/etc),
    - forward to PCInterface.msg_queue if you want to reuse your existing TCP sender.
"""

import os
import re
import time
import json
import shutil
import logging
from typing import Dict, Any, List, Optional
from threading import Thread
from flask import Flask, request, jsonify

import requests  # pip install requests

# ==============================================================================
# Configuration (can be overridden by environment variables)
# ==============================================================================

A5_MODE = os.getenv("A5_MODE", "0") == "1"

# -- Algo (path planning) server on the PC --
ALGO_HOST = os.getenv("ALGO_HOST", "192.168.22.13")  # PC IP running Flask algo
ALGO_PORT = int(os.getenv("ALGO_PORT", "5050"))
ALGO_BASE = f"http://{ALGO_HOST}:{ALGO_PORT}"

# -- Image-recognition server (separate host) --
IMG_HOST = os.getenv("IMG_HOST", "192.168.22.21")
IMG_PORT = int(os.getenv("IMG_PORT", "5000"))
IMG_BASE = f"http://{IMG_HOST}:{IMG_PORT}"
IMG_BACKEND = os.getenv("IMG_BACKEND", "yolo").lower()  # "yolo" or "opencv"

# -- HTTP client behavior (both servers use these) --
HTTP_TIMEOUT_S = float(os.getenv("HTTP_TIMEOUT_S", "8"))
HTTP_RETRIES   = int(os.getenv("HTTP_RETRIES", "3"))
RETRY_SLEEP_S  = float(os.getenv("HTTP_RETRY_SLEEP", "1.0"))

# -- Optional: forward to your existing PCInterface queue (length-prefixed TCP sender) --
# Set FORWARD_TO_PCINTERFACE=1 and set QUEUE_TARGET below to your RPiMain.PC.msg_queue
FORWARD_TO_PCINTERFACE = os.getenv("FORWARD_TO_PCINTERFACE", "0") == "1"
QUEUE_TARGET = None  # e.g., RPiMain.PC.msg_queue (bytes queue); see TODO below.

# -- Where the RPi camera writes images (we read this file for inference) --
LOCAL_IMAGE_PATH = os.getenv("LOCAL_IMAGE_PATH", "/home/pi/mdp-rpi/camera/images/image.jpg")

# Basic logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


# ==============================================================================
# Shared HTTP client with keep-alive + retries
# - Both the AlgoHTTP and ImageHTTP inherit from this.
# ==============================================================================
class HttpClient:
    """
    A thin helper that:
      - Maintains an HTTP session (keep-alive, so generally a single TCP connection).
      - Retries transient failures (timeouts, 5xx) a few times before giving up.
    """
    def __init__(self, base: str, timeout: float, retries: int, backoff_sleep: float):
        self.base = base.rstrip("/")
        self.s = requests.Session()
        self.timeout = timeout
        self.retries = retries
        self.sleep = backoff_sleep

    def _req(self, method: str, path: str, **kw) -> requests.Response:
        """
        Perform an HTTP request with retries.
        Raises on final failure; caller handles.
        """
        url = f"{self.base}{path}"
        for i in range(self.retries):
            try:
                r = self.s.request(method, url, timeout=self.timeout, **kw)
                r.raise_for_status()
                return r
            except Exception as e:
                logging.warning("HTTP %s %s failed (attempt %d/%d): %s",
                                method, path, i+1, self.retries, e)
                if i + 1 == self.retries:
                    raise
                time.sleep(self.sleep)


# ==============================================================================
# Algo (path planning) HTTP client
# - Talks to your PC Flask app's /health, /path, /path_debug, (optional) /stitch.
# ==============================================================================
class AlgoHTTP(HttpClient):
    def health(self) -> bool:
        """
        GET /health -> "OK" on success (your Flask algo's liveness).
        """
        try:
            r = self._req("GET", "/health")
            return (r.text or "").strip() == "OK"
        except Exception:
            return False

    def path(self, payload: Dict[str, Any], debug: bool = False) -> Dict[str, Any]:
        """
        POST /path or /path_debug with:
            {
              "obstacles": [{"x":..,"y":..,"d":..,"id":..}, ...],
              "retrying": bool,
              "robot_x": int,
              "robot_y": int,
              "robot_dir": "0|1|2|3"  (string; server casts to int)
            }
        Returns JSON with "data": {"commands":[...], "path":[...], "distance":...}
        """
        ep = "/path_debug" if debug else "/path"
        return self._req("POST", ep, json=payload).json()

    def stitch(self) -> Dict[str, Any]:
        """
        Optional: GET /stitch on the algo server (not used by default now).
        """
        return self._req("GET", "/stitch").json()


# ==============================================================================
# Image-recognition HTTP client
# - Talks to your image server's /status, /imageyolo or /imageopencv (and optional /stitch).
# ==============================================================================
class ImageHTTP(HttpClient):
    """
    POST /imageyolo   with form-data field 'file' -> {"obstacle_id":"..","image_id":".."}
    POST /imageopencv with form-data field 'file' -> same shape
    """
    def status(self) -> Dict[str, Any]:
        """
        GET /status -> basic {"result":"okk"} from your image server, used for sanity check.
        """
        return self._req("GET", "/status").json()

    def infer(self, file_path: str, backend: str = "yolo") -> Dict[str, Any]:
        """
        Uploads the *file_path* under multipart field 'file' to either:
          - /imageyolo  (backend="yolo")
          - /imageopencv (backend="opencv")
        The server responds with:
          {"obstacle_id": "<from filename 2nd token>", "image_id": "<model output>"}
        """
        ep = "/imageyolo" if backend == "yolo" else "/imageopencv"
        fn = os.path.basename(file_path)
        with open(file_path, "rb") as f:
            files = {"file": (fn, f, "image/jpeg")}  # content-type typical for JPG
            return self._req("POST", ep, files=files).json()

    def stitch(self) -> Dict[str, Any]:
        """
        Optional: GET /stitch on the image server to create stitched strips.
        """
        return self._req("GET", "/stitch").json()


# ==============================================================================
# Translators: normalize server responses into your legacy RPi message shapes
# - NAVIGATION: used by your existing routing to STM/Android.
# - IMAGE_RESULTS: matches your RPi handler (expects img_id, not image_id).
# ==============================================================================
def wrap_nav_from_algo(path_json: Dict[str, Any]) -> Dict[str, Any]:
    data = path_json.get("data", {})
    return {
        "type": "NAVIGATION",
        "data": {
            "commands": data.get("commands", []),
            "path": data.get("path", [])
        }
    }

def wrap_image_result(img_json: Dict[str, Any]) -> Dict[str, Any]:
    # Map "image_id" (server) -> "img_id" (your RPi/Android consumer expects)
    return {
        "type": "IMAGE_RESULTS",
        "data": {
            "obstacle_id": img_json.get("obstacle_id"),
            "img_id": img_json.get("image_id")
        }
    }

#android and stm sendng bytes over 
def _json_to_bytes(d: Dict[str, Any]) -> bytes:
    """Utility: JSON-dump `d` to UTF-8 bytes."""
    import json as _json
    return _json.dumps(d).encode("utf-8")

def encode_len_prefixed(msg_dict: Dict[str, Any]) -> bytes:
    """
    If you decide to forward via your existing PCInterface (length-prefixed TCP),
    convert dict -> utf-8 bytes. The 4-byte length prefix is added in PCInterface.send().
    """
    payload = json.dumps(msg_dict)
    return payload.encode("utf-8")


# ==============================================================================
# Bridge Orchestrator
# - Owns the high-level flow:
#     1) Build /path request (robot + obstacles)
#     2) Broadcast NAVIGATION message
#     3) Execute commands; on SNAP*:
#           - find obstacle id (if embedded in token),
#           - upload local image to image server,
#           - broadcast IMAGE_RESULTS
#     4) (Optional) call image server /stitch at FIN if you want.
# ==============================================================================
class AlgoBridge:
    """
    Replace the dummy providers & stubs with your integrations when ready.
    TODO markers indicate where to plug-in Android + STM.
    """

    def __init__(self, algo: AlgoHTTP, img: ImageHTTP, forward: bool = False, rpimain: Optional[object] = None, android: Optional[object] = None, stm: Optional[object] = None):
        self.algo = algo
        self.img = img
        self.forward = forward

        # Wire up queues to existing RPiMain/Android/STM interfaces if available
        self.rpimain = rpimain

        if android is not None:
            self.android_queue = getattr(android, "msg_queue", None)
        elif rpimain is not None and hasattr(rpimain, "Android"):
            self.android_queue = getattr(getattr(rpimain, "Android"), "msg_queue", None)
        else:
            self.android_queue = None

        if stm is not None:
            self.stm_queue = getattr(stm, "msg_queue", None)
        elif rpimain is not None and hasattr(rpimain, "STM"):
            self.stm_queue = getattr(getattr(rpimain, "STM"), "msg_queue", None)
        else:
            self.stm_queue = None

    # --------------------------------------------------------------------------
    # INPUT PROVIDERS (replace with your real data sources)
    # --------------------------------------------------------------------------
    def get_robot_state(self) -> Dict[str, Any]:
        """
        TODO[Android?]: If Android is the source of robot position/heading,
                        read it here and ensure robot_dir is a STRING "0|1|2|3".
        """
        a = getattr(getattr(self.rpimain, "Android", None), "last_pose", None)
        return a if a else {"x": 1, "y": 1, "dir": "0"}
       # return {"x": 1, "y": 1, "dir": "0"}  # <-- placeholder

    def get_obstacles(self) -> List[Dict[str, Any]]:
        """
        TODO[Android]: If Android provides obstacle list, fetch and return here.
        Must be a list of dicts with keys: x, y, d, id (all ints).
        """
        obs = getattr(getattr(self.rpimain, "Android", None), "last_obstacles", None)
        return obs if obs else []
        #return [
        #    {"x": 5, "y": 7, "d": 2, "id": 1},
        #    {"x": 10, "y": 12, "d": 0, "id": 2}
        #]

    # --------------------------------------------------------------------------
    # IMAGE PREP: Ensure filename encodes obstacle_id in the 2nd underscore token
    # --------------------------------------------------------------------------
    def _prepare_upload_file(self, obstacle_id: int) -> str:
        """
        The image server extracts obstacle_id from the *filename* like:
           "<anything>_<OBSTACLE_ID>_<anything>.jpg"
        We copy the local camera image to /tmp/rpi_captures/task1_<id>_image.jpg
        to satisfy that convention.
        """
        src = LOCAL_IMAGE_PATH
        if not os.path.exists(src):
            logging.warning("Local image not found at %s; creating a dummy JPEG.", src)
            os.makedirs(os.path.dirname(src), exist_ok=True)
            with open(src, "wb") as f:
                f.write(b"\xFF\xD8\xFF\xD9")  # minimal JPEG so upload doesn't fail

        os.makedirs("/tmp/rpi_captures", exist_ok=True)
        dst = f"/tmp/rpi_captures/task1_{obstacle_id}_image.jpg" #TODO change this logic
        shutil.copyfile(src, dst)
        return dst

    # --------------------------------------------------------------------------
    # STM + ANDROID STUBS (replace when integrating)
    # --------------------------------------------------------------------------
    def send_to_stm(self, cmd: str) -> None:
        """Forward a single movement command to STM as NAVIGATION JSON over its msg_queue."""
        try:
            if hasattr(self, "stm_queue") and self.stm_queue is not None:
                nav_msg = {"type": "NAVIGATION", "data": {"commands": [cmd]}}
                self.stm_queue.put(_json_to_bytes(nav_msg))
                logging.info("[STM] Enqueued command: %s", cmd)
            else:
                logging.warning("STM queue not wired; using dummy delay for cmd: %s", cmd)
                time.sleep(0.05)
        except Exception as e:
            logging.error("Failed to enqueue STM command %s: %s", cmd, e)

    def publish_android_update(self, msg_dict: Dict[str, Any]) -> None:
        """Publish `msg_dict` to Android via RPiMain.Android.msg_queue (bytes)."""
        try:
            if hasattr(self, "android_queue") and self.android_queue is not None:
                self.android_queue.put(_json_to_bytes(msg_dict))
                logging.info("[Android] Published: type=%s", msg_dict.get("type"))
            else:
                logging.warning("Android queue not wired; skipping publish.")
        except Exception as e:
            logging.error("Failed to publish to Android: %s", e)

    def forward_to_pcinterface(self, msg_dict: Dict[str, Any]) -> None:
        """
        TODO[PCInterface]:
          - If you want to reuse your existing TCP pipeline (length-prefixed),
            set FORWARD_TO_PCINTERFACE=1 and set QUEUE_TARGET to your
            RPiMain.PC.msg_queue before calling run_demo().
        """
        if not (self.forward and QUEUE_TARGET is not None):
            return
        QUEUE_TARGET.put(encode_len_prefixed(msg_dict))


    def hardcoded_algo_response(self) -> Dict[str, Any]:
        return {
            "data": {
                "commands": [
                    # "FR090","FW030","FL090","BW010","SNAP3_C","BW010","FR090","FW020","FL090","FL090",
                    # "BW010","SNAP2_C","BW010","FL090","BW020","BR090","BL090","BW010","SNAP1_C",
                    # "FW010","FR090","BR090","FW020","FL090","BW010","SNAP4_C","FIN"

                    "FR090","SNAP1_C", "BW30", "FL090", "FR090", "BW050", "FR090", "SNAP2_C", "BW030", "FL090", "FR090", "BW050", "FR090",
                    "SNAP3_C" "BW030", "FL090", "FR090", "BW050", "FR090", "SNAP4_C", "FIN"
                ],
                "distance": 54.19999999999999
                # "path": [
                #     {"d": 0, "s": -1, "x": 1,  "y": 1},
                #     {"d": 2, "s": -1, "x": 4,  "y": 2},
                #     {"d": 2, "s": -1, "x": 7,  "y": 2},
                #     {"d": 0, "s": -1, "x": 8,  "y": 5},
                #     {"d": 0, "s":  3, "x": 8,  "y": 4},
                #     {"d": 0, "s": -1, "x": 8,  "y": 3},
                #     {"d": 2, "s": -1, "x": 11, "y": 4},
                #     {"d": 2, "s": -1, "x": 13, "y": 4},
                #     {"d": 0, "s": -1, "x": 14, "y": 7},
                #     {"d": 6, "s": -1, "x": 11, "y": 8},
                #     {"d": 6, "s":  2, "x": 12, "y": 8},
                #     {"d": 6, "s": -1, "x": 13, "y": 8},
                #     {"d": 4, "s": -1, "x": 12, "y": 5},
                #     {"d": 4, "s": -1, "x": 12, "y": 7},
                #     {"d": 2, "s": -1, "x": 11, "y":10},
                #     {"d": 4, "s": -1, "x": 8,  "y":11},
                #     {"d": 4, "s":  1, "x": 8,  "y":12},
                #     {"d": 4, "s": -1, "x": 8,  "y":11},
                #     {"d": 6, "s": -1, "x": 5,  "y":10},
                #     {"d": 4, "s": -1, "x": 2,  "y":11},
                #     {"d": 4, "s": -1, "x": 2,  "y": 9},
                #     {"d": 2, "s": -1, "x": 5,  "y": 8},
                #     {"d": 2, "s":  4, "x": 4,  "y": 8}
                # ]
            },
            "error": None
        }
    # --------------------------------------------------------------------------
    # MAIN FLOWS
    # --------------------------------------------------------------------------
    def request_path_and_broadcast(self, retrying: bool = False, debug: bool = False) -> Dict[str, Any]:
        """
        1) Build the /path request from robot state + obstacles.
        2) Send to Algo server.
        3) Translate to NAVIGATION and broadcast to Android/PCInterface.
        """

        if A5_MODE : 
            logging.info("Using HARDCODED algo response (skipping /path call).")
            resp = self.hardcoded_algo_response()
        else : 
            robot = self.get_robot_state()
            obstacles = self.get_obstacles()

            # -- IMPORTANT: robot_dir must be a STRING; server does int(content['robot_dir']) --
            path_req = {
                "obstacles": obstacles,
                "retrying": retrying,
                "robot_x": robot["x"],
                "robot_y": robot["y"],
                "robot_dir": robot["dir"],  # string "0|1|2|3"
            }
            logging.info("Calling /path%s ...", "_debug" if debug else "")
            resp = self.algo.path(path_req, debug=debug)


        # Wrap into your legacy NAVIGATION message
        nav_msg = wrap_nav_from_algo(resp)
        logging.info("NAVIGATION commands (first few): %s", nav_msg["data"]["commands"][:6])
        self.publish_android_update(nav_msg)     # TODO[Android]
        self.forward_to_pcinterface(nav_msg)     # TODO[PCInterface]
        return resp

    def execute_commands_and_do_snaps(self, commands: List[str]) -> None:
        """
        Iterate through Algo commands:
          - Movement: send to STM (or skip if it's a no-op turn).
          - SNAP*: upload local image to Image server, then broadcast IMAGE_RESULTS.
          - FIN: end-of-run hook (optionally call image server /stitch).
        """
        took_any_photos = False

        for cmd in commands:
            # ---- Photo commands: match SNAP, SNAP1_C, SNAP12_R, etc. ----
            if cmd.startswith("SNAP"):
                took_any_photos = True
                # Try to parse obstacle id embedded as SNAP<id>...
                m = re.match(r"^SNAP(\d+)", cmd)
                obstacle_id = int(m.group(1)) if m else 1  # default to 1 if absent

                # Prepare local file with correct naming convention (2nd token = obstacle_id)
                upload_path = self._prepare_upload_file(obstacle_id)

                # Send to Image server (NOT Algo)
                img_json = self.img.infer(upload_path, backend=IMG_BACKEND)

                # Map to legacy IMAGE_RESULTS and notify
                img_msg = wrap_image_result(img_json)
                logging.info("IMAGE_RESULTS: obstacle=%s, img_id=%s",
                             img_msg["data"].get("obstacle_id"),
                             img_msg["data"].get("img_id"))
                self.publish_android_update(img_msg)   # TODO[Android]
                self.forward_to_pcinterface(img_msg)   # TODO[PCInterface]
                continue

            # ---- Ignore no-op turns like FR00/FL00/BR00/BL00 ----
            if re.match(r"^(FR|FL|BR|BL)0+$", cmd):
                logging.info("[STM] (skip no-op): %s", cmd)
                continue

            # ---- End-of-run ----
            if cmd == "FIN":
                logging.info("FIN received.")
                # Optional: stitch on the image server if you want visual summaries.
                # try:
                #     if took_any_photos:
                #         stitch_resp = self.img.stitch()
                #         logging.info("Image server /stitch: %s", stitch_resp)
                # except Exception as e:
                #     logging.warning("Image /stitch failed (non-fatal): %s", e)
                continue

            # ---- Regular movement: forward to STM ----
            self.send_to_stm(cmd)  # TODO[STM]

    def run_demo(self) -> None:
        """
        Demo flow:
          - Check Algo liveness.
          - Request a path.
          - Execute commands and do SNAP uploads to the image server.
        """
        ok = self.algo.health()
        logging.info("Algo /health: %s", ok)

        path_json = self.request_path_and_broadcast(retrying=False, debug=False)
        cmds = path_json.get("data", {}).get("commands", [])
        logging.info("Total commands: %d", len(cmds))
        self.execute_commands_and_do_snaps(cmds)

    
    



# ==============================================================================
# Entrypoint
# ==============================================================================
def main():
    from threading import Thread
    from rpi_main import RPiMain, TASK_2

    rpimain = RPiMain(TASK_2)

    # Let RPiMain manage connect/start/join logic
    Thread(target=rpimain.run, name="RPiMain_thread", daemon=True).start()


    # Create HTTP clients for both servers
    algo = AlgoHTTP(ALGO_BASE, HTTP_TIMEOUT_S, HTTP_RETRIES, RETRY_SLEEP_S)
    img  = ImageHTTP(IMG_BASE, HTTP_TIMEOUT_S, HTTP_RETRIES, RETRY_SLEEP_S)

    # Optional quick sanity check to confirm the image server is reachable
    try:
        st = img.status()
        logging.info("Image server /status: %s", st)
    except Exception as e:
        logging.warning("Image server not reachable yet: %s", e)


    # Build the bridge
    bridge = AlgoBridge(algo=algo, img=img, forward=FORWARD_TO_PCINTERFACE, rpimain=rpimain)


    if A5_MODE == "push" :
        logging.info("A5_MODE = push-> starting RPI reciever on port %d; waiting for PC posts...", BRIDGE_PORT)
        start_receiver_http(bridge,port = BRIDGE_PORT)
    else :
        logging.info("ALGO_MODE=pull → calling Algo server ourselves (existing behavior).")
        bridge.run_demo()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception("Bridge crashed: %s", e)



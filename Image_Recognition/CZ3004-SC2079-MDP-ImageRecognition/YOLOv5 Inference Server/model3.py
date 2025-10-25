import os
import shutil
import time
import glob
import cv2
import torch
import random
import string
import numpy as np
from PIL import Image
from ultralytics import YOLO   # ✅ New way to load YOLO models

# ------------------------------- perf knobs (CPU-friendly) ----------------------------
for k in ["OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS","VECLIB_MAXIMUM_THREADS","NUMEXPR_NUM_THREADS"]:
    os.environ.setdefault(k, "2")
torch.set_num_threads(2)
try:
    cv2.setNumThreads(0)
    cv2.setUseOptimized(True)
except Exception:
    pass

YOLO_IMG_SIZE = int(os.getenv("MDP_YOLO_IMG_SIZE", "416"))  # 320/416/512
YOLO_CONF     = float(os.getenv("MDP_YOLO_CONF", "0.35"))
YOLO_IOU      = float(os.getenv("MDP_YOLO_IOU",  "0.45"))
YOLO_MAXDET   = int(os.getenv("MDP_YOLO_MAXDET", "5"))

# ------------------------------------ templates --------------------------------------
TEMPLATE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "opencv_templates"))
TEMPLATES = {}  # label -> grayscale template

def load_opencv_templates():
    """Load grayscale templates from TEMPLATE_DIR into TEMPLATES dict."""
    if not os.path.isdir(TEMPLATE_DIR):
        print(f"[OpenCV] Template dir not found: {TEMPLATE_DIR}")
        return
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    loaded = 0
    for fname in os.listdir(TEMPLATE_DIR):
        if fname.lower().endswith(exts):
            label = os.path.splitext(fname)[0]
            path = os.path.join(TEMPLATE_DIR, fname)
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            TEMPLATES[label] = img
            loaded += 1
    print(f"[OpenCV] Loaded {loaded} templates from {TEMPLATE_DIR}")

# ------------------------------------ ID mapping -------------------------------------
NAME_TO_ID = {
    "NA": 'NA', "Bullseye": 10,
    "One": 11, "Two": 12, "Three": 13, "Four": 14, "Five": 15,
    "Six": 16, "Seven": 17, "Eight": 18, "Nine": 19,
    "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
    "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
    "Up": 36, "Down": 37, "Right": 38, "Left": 39,
    "Up Arrow": 36, "Down Arrow": 37, "Right Arrow": 38, "Left Arrow": 39,
    "Stop": 40
}
FALLBACK_ACCEPT = float(os.getenv("MDP_FALLBACK_ACCEPT", "0.50"))



# raw class-name (as stored in your trained weights) -> human-friendly label used by the app
RAW_TO_HUMAN = {
    "10": "Bullseye",
    "11": "One",  "12": "Two",  "13": "Three", "14": "Four", "15": "Five",
    "16": "Six",  "17": "Seven","18": "Eight", "19": "Nine",
    "20": "A", "21": "B", "22": "C", "23": "D", "24": "E", "25": "F", "26": "G", "27": "H",
    "28": "S", "29": "T", "30": "U", "31": "V", "32": "W", "33": "X", "34": "Y", "35": "Z",
    "36": "Up", "37": "Down", "38": "Right", "39": "Left",
    "40": "Stop", "marker": "Bullseye"
}

def to_human_label(raw_label):
    # raw_label could be an int or string; normalize to string
    s = str(raw_label)
    return RAW_TO_HUMAN.get(s, s)


def get_random_string(length):
    return ''.join(random.choice(string.ascii_letters) for _ in range(length))

# ------------------------------------ YOLO loader ------------------------------------
def load_model():
    """Load YOLO model with Ultralytics API."""
    base_dir = os.path.dirname(__file__)
    weights_path = os.path.abspath(os.path.join(base_dir, "..", "Weights", "best(125epochs).pt"))

    if not os.path.exists(weights_path):
        raise FileNotFoundError(f"Weights not found at: {weights_path}")

    model = YOLO(weights_path)
    model.overrides['conf'] = YOLO_CONF
    model.overrides['iou'] = YOLO_IOU
    model.overrides['max_det'] = YOLO_MAXDET
    return model

# -------------------------------- drawing / saving -----------------------------------
def draw_own_bbox(img, x1, y1, x2, y2, label, color=(36,255,12), text_color=(0,0,0)):
    """Draw bbox + label. Saves raw and annotated images under own_results/"""
    os.makedirs("own_results", exist_ok=True)
    label = label + "-" + str(NAME_TO_ID.get(label, 'NA'))
    rand = str(int(time.time()))

    # Save raw
    cv2.imwrite(f"own_results/raw_image_{label}_{rand}.jpg", img)

    # Draw
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), color, -1)
    cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 1)
    cv2.imwrite(f"own_results/annotated_image_{label}_{rand}.jpg", img)

# ----------------------------- YOLO-only predictor (fast) -----------------------------
# def predict_image_yolo(image_filename: str, model) -> str:
#     """YOLO-only classification. No OpenCV fallback."""
#     uploads_dir = 'uploads'
#     img_path = os.path.join(uploads_dir, image_filename)
#     bgr = cv2.imread(img_path)
#     if bgr is None:
#         print("Failed to read image:", img_path)
#         return 'NA'

#     results = model.predict(
#         bgr,
#         imgsz=YOLO_IMG_SIZE,
#         conf=YOLO_CONF,
#         iou=YOLO_IOU,
#         verbose=False
#     )

#     if not results or results[0].boxes is None or len(results[0].boxes) == 0:
#         return 'NA'

#     boxes = results[0].boxes
#     names = results[0].names

#     xyxy = boxes.xyxy.cpu().numpy().astype(int)
#     confs = boxes.conf.cpu().numpy()
#     clses = boxes.cls.cpu().numpy().astype(int)

#     best = None
#     best_area = 0

#     for i in range(len(xyxy)):
#         x1, y1, x2, y2 = xyxy[i]
#         conf = confs[i]
#         cls_id = clses[i]

#         raw_label = names[cls_id]        # e.g. "28"
#         label = to_human_label(raw_label)  # convert -> "S"

#         if conf < YOLO_CONF :
#             continue

#         area = (x2 - x1) * (y2 - y1)
#         if area > best_area:
#             best_area = area
#             best = (x1, y1, x2, y2, label)

#     if best is None:
#         return 'NA'

#     x1, y1, x2, y2, label = best
#     draw_own_bbox(bgr, x1, y1, x2, y2, label)

#     return str(NAME_TO_ID.get(label, 'NA'))


def predict_image_yolo(image_filename: str, model) -> str:
    """YOLO-only classification. No OpenCV fallback."""
    uploads_dir = 'uploads'
    img_path = os.path.join(uploads_dir, image_filename)
    bgr = cv2.imread(img_path)
    if bgr is None:
        print("Failed to read image:", img_path)
        return 'NA'

    results = model.predict(
        bgr,
        imgsz=YOLO_IMG_SIZE,
        conf=YOLO_CONF,
        iou=YOLO_IOU,
        verbose=False
    )

    if not results or results[0].boxes is None or len(results[0].boxes) == 0:
        return 'NA'

    boxes = results[0].boxes
    names = results[0].names

    xyxy = boxes.xyxy.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()
    clses = boxes.cls.cpu().numpy().astype(int)

    detections = []
    for i in range(len(xyxy)):
        x1, y1, x2, y2 = xyxy[i]
        conf = confs[i]
        cls_id = clses[i]
        raw_label = names[cls_id]
        label = to_human_label(raw_label)
        if conf < YOLO_CONF:
            continue
        area = (x2 - x1) * (y2 - y1)
        detections.append((x1, y1, x2, y2, label, area))

    if not detections:
        return 'NA'

    # 🧩 separate bullseyes from others
    bullseye_dets = [d for d in detections if d[4].lower() in ("bullseye", "marker")]
    other_dets    = [d for d in detections if d[4].lower() not in ("bullseye", "marker")]

    print("image:", bullseye_dets, other_dets)

    # 🧠 Heuristic: if other valid classes exist, ignore bullseyes
    if other_dets:
        best = max(other_dets, key=lambda d: d[5])   # largest area among non-bullseye
        #best = max(other_dets, key=lambda d: confs[i])
        #best = max(other_dets, key=lambda d: (d[conf_index]*0.7 + d[area_index]*0.3))


    else:
        best = max(detections, key=lambda d: d[5])   # only bullseye present

    x1, y1, x2, y2, label, area = best
    draw_own_bbox(bgr, x1, y1, x2, y2, label)

    return str(NAME_TO_ID.get(label, 'NA'))

# --------------------------- OpenCV-only predictor (thorough) -------------------------
def _prep_gray(g):
    g = cv2.GaussianBlur(g, (3, 3), 0)
    g = cv2.equalizeHist(g)
    return g

def _rotate_gray(g, angle_deg):
    if angle_deg == 0:
        return g
    h, w = g.shape[:2]
    M = cv2.getRotationMatrix2D((w/2.0, h/2.0), angle_deg, 1.0)
    return cv2.warpAffine(g, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

def _best_template_match_with_loc(roi_bgr, method=cv2.TM_CCOEFF_NORMED):
    if not TEMPLATES:
        return "NA", -1.0, None
    roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    MAX_SIDE = 480
    h, w = roi_gray.shape[:2]
    s_roi = 1.0
    if max(h, w) > MAX_SIDE:
        s_roi = MAX_SIDE / float(max(h, w))
        roi_gray = cv2.resize(roi_gray, (int(w*s_roi), int(h*s_roi)), interpolation=cv2.INTER_AREA)

    roi_variants = []
    for invert in (False, True):
        rg = roi_gray if not invert else cv2.bitwise_not(roi_gray)
        rg = _prep_gray(rg)
        for ang in (-8, -4, 0, 4, 8):
            roi_variants.append(_rotate_gray(rg, ang))

    scales = [round(x, 2) for x in np.linspace(0.6, 1.4, 9)]
    best_label, best_score, best_box = "NA", -1.0, None
    for label, tmpl in TEMPLATES.items():
        for invert in (False, True):
            tg = tmpl if not invert else cv2.bitwise_not(tmpl)
            tg = _prep_gray(tg)
            for s in scales:
                th = max(8, int(tg.shape[0] * s))
                tw = max(8, int(tg.shape[1] * s))
                t_resized = cv2.resize(tg, (tw, th), interpolation=cv2.INTER_AREA)
                for rg in roi_variants:
                    if th >= rg.shape[0] or tw >= rg.shape[1]:
                        continue
                    res = cv2.matchTemplate(rg, t_resized, method)
                    _, max_val, _, max_loc = cv2.minMaxLoc(res)
                    if max_val > best_score:
                        best_score = float(max_val)
                        best_label = label
                        x, y = max_loc
                        best_box = (x, y, tw, th)

    if best_box is not None and s_roi != 1.0:
        x, y, tw, th = best_box
        inv = 1.0 / s_roi
        best_box = (int(x*inv), int(y*inv), int(tw*inv), int(th*inv))
    return best_label, best_score, best_box

def predict_image_opencv(image_filename: str) -> str:
    uploads_dir = 'uploads'
    img_path = os.path.join(uploads_dir, image_filename)
    bgr = cv2.imread(img_path)
    if bgr is None:
        print("Failed to read image:", img_path)
        return 'NA'

    label, score, box = _best_template_match_with_loc(bgr)
    os.makedirs("runs/opencv", exist_ok=True)
    vis = bgr.copy()
    if box is not None and label != "NA":
        x, y, w, h = box
        cv2.rectangle(vis, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(vis, f"{label} {score:.2f}", (x, max(0, y-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
    out_name = f"{int(time.time())}_{label if label!='NA' else 'NA'}.jpg"
    cv2.imwrite(os.path.join("runs/opencv", out_name), vis)
    if label in NAME_TO_ID and score >= FALLBACK_ACCEPT:
        return str(NAME_TO_ID[label])
    return 'NA'

# ----------------------------------- stitching ---------------------------------------
def stitch_image():
    imgFolder = 'runs'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')
    imgPaths = glob.glob(os.path.join(imgFolder + "/detect/*/", "*.jpg"))
    images = [Image.open(x) for x in imgPaths]
    if not images:
        return None
    width, height = zip(*(i.size for i in images))
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0
    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    stitchedImg.save(stitchedPath)
    for img in imgPaths:
        os.makedirs(os.path.join("runs", "originals"), exist_ok=True)
        shutil.move(img, os.path.join("runs", "originals", os.path.basename(img)))
    return stitchedImg

def stitch_image_own():
    imgFolder = 'own_results'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')
    imgPaths = glob.glob(os.path.join(imgFolder + "/annotated_image_*.jpg"))
    if not imgPaths:
        return None
    imgTimestamps = [imgPath.split("_")[-1][:-4] for imgPath in imgPaths]
    sortedByTimeStampImages = sorted(zip(imgPaths, imgTimestamps), key=lambda x: x[1])
    images = [Image.open(x[0]) for x in sortedByTimeStampImages]
    width, height = zip(*(i.size for i in images))
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0
    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    stitchedImg.save(stitchedPath)
    return stitchedImg

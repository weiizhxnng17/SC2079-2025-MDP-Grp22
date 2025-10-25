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

# ---------------- PyTorch 2.6 safe-unpickler fix for YOLOv5 checkpoints ----------------
from torch.serialization import add_safe_globals

try:
    from models.yolo import DetectionModel
    add_safe_globals([DetectionModel])
    try:
        from models.yolo import Model  # some forks use Model
        add_safe_globals([Model])
    except Exception:
        pass
except Exception as e:
    print("[WARN] Could not register YOLO classes for safe unpickler:", e)

_orig_torch_load = torch.load
def _patched_torch_load(*args, **kwargs):
    kwargs.setdefault("weights_only", False)  # critical for older YOLOv5 .pt
    return _orig_torch_load(*args, **kwargs)
torch.load = _patched_torch_load
# --------------------------------------------------------------------------------------

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
# --------------------------------------------------------------------------------------

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
# --------------------------------------------------------------------------------------

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
# --------------------------------------------------------------------------------------

def get_random_string(length):
    return ''.join(random.choice(string.ascii_letters) for _ in range(length))

# ------------------------------------ YOLO loader ------------------------------------
def load_model():
    """Load YOLOv5 model from local repo (hubconf.py) + your weights."""
    base_dir     = os.path.dirname(__file__)
    repo_dir     = base_dir
    #weights_path = os.path.abspath(os.path.join(base_dir, "..", "Weights", "Week_9.pt"))
    weights_path = os.path.abspath(os.path.join(base_dir, "..", "Weights", "best(125epochs).pt"))
    if not os.path.exists(weights_path):
        raise FileNotFoundError(f"Weights not found at: {weights_path}")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = torch.hub.load(
        repo_dir, 'custom', path=weights_path, source='local', force_reload=False
    ).to(device).eval()

    try:
        model.conf = YOLO_CONF
        model.iou  = YOLO_IOU
        model.max_det = YOLO_MAXDET
        if device.type == 'cuda':
            model.half()
    except Exception:
        pass

    # one-time warmup
    with torch.no_grad():
        dummy = torch.zeros(1, 3, YOLO_IMG_SIZE, YOLO_IMG_SIZE).to(device)
        if device.type == 'cuda' and next(model.parameters()).dtype == torch.float16:
            dummy = dummy.half()
        _ = model(dummy)

    return model
# --------------------------------------------------------------------------------------

# -------------------------------- drawing / saving -----------------------------------
def draw_own_bbox(img, x1, y1, x2, y2, label, color=(36,255,12), text_color=(0,0,0)):
    """Draw bbox + label. Saves raw and annotated images under own_results/"""
    os.makedirs("own_results", exist_ok=True)

    name_to_id = NAME_TO_ID
    label = label + "-" + str(name_to_id.get(label, 'NA'))

    x1 = int(x1); y1 = int(y1); x2 = int(x2); y2 = int(y2)
    rand = str(int(time.time()))

    # Save raw
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"own_results/raw_image_{label}_{rand}.jpg", img_rgb)

    # Draw
    cv2.rectangle(img_rgb, (x1, y1), (x2, y2), color, 2)
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(img_rgb, (x1, y1 - 20), (x1 + w, y1), color, -1)
    cv2.putText(img_rgb, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 1)
    cv2.imwrite(f"own_results/annotated_image_{label}_{rand}.jpg", img_rgb)
# --------------------------------------------------------------------------------------

# -------------------- Thorough OpenCV matching (with location) -----------------------
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
    """Returns (label, score, (x,y,w,h)) in original ROI coords."""
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

    # map back to original coordinates if scaled
    if best_box is not None and s_roi != 1.0:
        x, y, tw, th = best_box
        inv = 1.0 / s_roi
        best_box = (int(x*inv), int(y*inv), int(tw*inv), int(th*inv))

    return best_label, best_score, best_box
# --------------------------------------------------------------------------------------

# ----------------------------- YOLO-only predictor (fast) -----------------------------
def predict_image_yolo(image_filename: str, model) -> str:
    """YOLO-only classification. No OpenCV fallback."""
    uploads_dir = 'uploads'
    img_path = os.path.join(uploads_dir, image_filename)
    bgr = cv2.imread(img_path)
    if bgr is None:
        print("Failed to read image:", img_path)
        return 'NA'

    results = model(bgr, size=YOLO_IMG_SIZE)
    try:
        results.save('runs')
    except Exception as e:
        print("[warn] results.save failed:", e)

    pred = results.pred[0]   # [N,6] => x1,y1,x2,y2,conf,cls
    names = results.names
    if pred is None or pred.shape[0] == 0:
        return 'NA'

    boxes = pred.cpu().numpy()
    areas = (boxes[:,2] - boxes[:,0]) * (boxes[:,3] - boxes[:,1])
    order = np.argsort(-areas)

    final_label, final_box = None, None
    for i in order:
        x1, y1, x2, y2, conf, cls = boxes[i]
        if conf < 0.50:
            continue
        label = names[int(cls)]
        if label == "Bullseye":
            continue
        final_label, final_box = label, (int(x1), int(y1), int(x2), int(y2))
        break

    if final_label is None:
        return 'NA'

    x1, y1, x2, y2 = final_box
    draw_own_bbox(bgr, x1, y1, x2, y2, final_label)
    return str(NAME_TO_ID.get(final_label, 'NA'))
# --------------------------------------------------------------------------------------

# --------------------------- OpenCV-only predictor (thorough) -------------------------
def predict_image_opencv(image_filename: str) -> str:
    """OpenCV-only classification (thorough). Saves annotated image to runs/opencv/."""
    uploads_dir = 'uploads'
    img_path = os.path.join(uploads_dir, image_filename)
    bgr = cv2.imread(img_path)
    if bgr is None:
        print("Failed to read image:", img_path)
        return 'NA'

    label, score, box = _best_template_match_with_loc(bgr)
    print(f"[opencv] best={label} score={score:.3f}")

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
# --------------------------------------------------------------------------------------

# ----------------------------------- stitching ---------------------------------------
def stitch_image():
    """Stitches YOLO-detect jpgs into one strip: saves to runs/stitched-*.jpeg"""
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
    """Stitches images saved by draw_own_bbox into one strip: saves to own_results/"""
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
# --------------------------------------------------------------------------------------

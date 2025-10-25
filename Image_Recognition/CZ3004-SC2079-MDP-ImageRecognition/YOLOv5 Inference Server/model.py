import os
import shutil
import time
import glob
import torch
from PIL import Image
import cv2
import random
import string
import numpy as np
import random
import torch

# --- PyTorch 2.6 safe-unpickler fix for YOLOv5 checkpoints ---
import torch
from torch.serialization import add_safe_globals


# Allow YOLOv5 classes contained in old-style pickled checkpoints
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

# Monkey-patch torch.load to default weights_only=False (needed by older YOLO .pt)
_orig_torch_load = torch.load
def _patched_torch_load(*args, **kwargs):
    kwargs.setdefault("weights_only", False)   # <-- critical
    return _orig_torch_load(*args, **kwargs)
torch.load = _patched_torch_load
# --- end fix ---

# --- perf knobs: keep CPU predictable and fast ---
import os, cv2, torch

# Limit BLAS/OMP threads (Windows CPU often speeds up with 1–2 threads)
for k in ["OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS","VECLIB_MAXIMUM_THREADS","NUMEXPR_NUM_THREADS"]:
    os.environ.setdefault(k, "2")
torch.set_num_threads(2)
try:
    cv2.setNumThreads(0)  # let torch own the threads
    cv2.setUseOptimized(True)
except Exception:
    pass

# Use a smaller inference size on CPU; keep quality reasonable
YOLO_IMG_SIZE = int(os.getenv("MDP_YOLO_IMG_SIZE", "416"))  # 320/416/512
YOLO_CONF = float(os.getenv("MDP_YOLO_CONF", "0.35"))       # confidence threshold
YOLO_IOU  = float(os.getenv("MDP_YOLO_IOU",  "0.45"))       # NMS IoU
YOLO_MAXDET = int(os.getenv("MDP_YOLO_MAXDET", "5"))        # keep a few boxes only


TEMPLATE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "opencv_templates"))
TEMPLATES = {}  # label -> gray template

def load_opencv_templates():
    """
    Loads grayscale templates from TEMPLATE_DIR into TEMPLATES dict.
    File names (without extension) become labels, e.g., 'A.png' -> 'A'.
    """
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


# --- Thorough OpenCV fallback (multi-scale, invert, light rotation) ---

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

def _best_template_match(roi_bgr, method=cv2.TM_CCOEFF_NORMED):
    """
    Thorough template search:
      - downscale huge ROIs to keep compute bounded
      - try both original and inverted polarity
      - try small rotations: -8, -4, 0, +4, +8 deg
      - multi-scale: 0.6 → 1.4 (step 0.1)
    Only called when YOLO fails, so OK to be heavier.
    """
    if not TEMPLATES:
        return "NA", -1.0

    # Prepare ROI
    roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    # cap max side for compute; increase to 600 if you want even more robustness
    MAX_SIDE = 480
    h, w = roi_gray.shape[:2]
    max_side = max(h, w)
    if max_side > MAX_SIDE:
        s = MAX_SIDE / float(max_side)
        roi_gray = cv2.resize(roi_gray, (int(w*s), int(h*s)), interpolation=cv2.INTER_AREA)

    # Build ROI variants (polarity x rotation)
    roi_variants = []
    for invert in (False, True):
        rg = roi_gray if not invert else cv2.bitwise_not(roi_gray)
        rg = _prep_gray(rg)
        for ang in (-8, -4, 0, 4, 8):
            roi_variants.append(_rotate_gray(rg, ang))

    # Multi-scales
    scales = [round(x, 2) for x in np.linspace(0.6, 1.4, 9)]  # 0.6,0.7,...,1.4

    best_label, best_score = "NA", -1.0
    for label, tmpl in TEMPLATES.items():
        # template polarity variants
        for invert in (False, True):
            tg = tmpl if not invert else cv2.bitwise_not(tmpl)
            tg = _prep_gray(tg)

            # scan scales
            for s in scales:
                th = max(8, int(tg.shape[0] * s))
                tw = max(8, int(tg.shape[1] * s))
                t_resized = cv2.resize(tg, (tw, th), interpolation=cv2.INTER_AREA)

                for rg in roi_variants:
                    if th >= rg.shape[0] or tw >= rg.shape[1]:
                        continue
                    res = cv2.matchTemplate(rg, t_resized, method)
                    _, max_val, _, _ = cv2.minMaxLoc(res)
                    if max_val > best_score:
                        best_score = float(max_val)
                        best_label = label

    return best_label, best_score

def opencv_fallback_whole_image(image_path):
    """
    Thorough whole-image scan used ONLY if YOLO returns no boxes.
    """
    img = cv2.imread(image_path)
    if img is None:
        return "NA", -1.0
    label, score = _best_template_match(img)
    print(f"[fallback/thorough] template={label} score={score:.3f}")
    return label, score



def get_random_string(length):
    """
    Generate a random string of fixed length 

    Inputs
    ------
    length: int - length of the string to be generated

    Returns
    -------
    str - random string

    """
    result_str = ''.join(random.choice(string.ascii_letters) for i in range(length))
    return result_str

# def load_model():
#     """
#     Load the model from the local directory
#     """
#     #model = torch.hub.load('./', 'custom', path='YOLOv5_new.pt', source='local')
#     model = torch.hub.load('./', 'custom', path='Week_9.pt', source='local')
#     return model


# def load_model():
#     """
#     Load the YOLOv5 model from Weights/Week_9.pt
#     """
#     # Path to this file's folder (YOLOv5 Inference Server)
#     base_dir = os.path.dirname(__file__)
#     # Go up one level into Weights folder
#     weights_path = os.path.join(base_dir, "..", "Weights", "Week_9.pt")

#     # Convert to absolute path (Windows safe)
#     weights_path = os.path.abspath(weights_path)

#     # Load YOLOv5 custom model using local repo
#     model = torch.hub.load(
#         'ultralytics/yolov5',  # can also be local path if you cloned yolov5/
#         'custom',
#         path=weights_path,
#         source='github'        # or 'local' if you have yolov5 repo inside project
#     )
#     return model

# import torch
# import os

# def load_model():
#     """
#     Load YOLOv5 model using local model definitions and weights
#     """
#     base_dir = os.path.dirname(__file__)
#     weights_path = os.path.join(base_dir, "..", "Weights", "Week_9.pt")
#     weights_path = os.path.abspath(weights_path)

#     # Load model directly
#     model = torch.load(weights_path, map_location=torch.device('cpu'))  # or 'cuda' if GPU available
#     model.eval()
#     return model

# model.py




# def load_model():
#     base_dir     = os.path.dirname(__file__)
#     repo_dir     = base_dir
#     weights_path = os.path.abspath(os.path.join(base_dir, "..", "Weights", "Week_9.pt"))
#     device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#     model = torch.hub.load(repo_dir, 'custom', path=weights_path, source='local').to(device).eval()
#     return model

def load_model():
    base_dir     = os.path.dirname(__file__)
    repo_dir     = base_dir
    weights_path = os.path.abspath(os.path.join(base_dir, "..", "Weights", "Week_9.pt"))
    if not os.path.exists(weights_path):
        raise FileNotFoundError(f"Weights not found at: {weights_path}")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = torch.hub.load(
        repo_dir, 'custom', path=weights_path, source='local', force_reload=False
    ).to(device).eval()

    # speed knobs
    try:
        model.conf = YOLO_CONF
        model.iou  = YOLO_IOU
        model.max_det = YOLO_MAXDET
        # If you have CUDA, fp16 is a big speedup
        if device.type == 'cuda':
            model.half()
    except Exception:
        pass

    # warmup (once) – compiles kernels and allocates buffers
    with torch.no_grad():
        dummy = torch.zeros(1, 3, YOLO_IMG_SIZE, YOLO_IMG_SIZE).to(device)
        if device.type == 'cuda' and next(model.parameters()).dtype == torch.float16:
            dummy = dummy.half()
        _ = model(dummy)

    return model






def draw_own_bbox(img,x1,y1,x2,y2,label,color=(36,255,12),text_color=(0,0,0)):
    """
    Draw bounding box on the image with text label and save both the raw and annotated image in the 'own_results' folder

    Inputs
    ------
    img: numpy.ndarray - image on which the bounding box is to be drawn

    x1: int - x coordinate of the top left corner of the bounding box

    y1: int - y coordinate of the top left corner of the bounding box

    x2: int - x coordinate of the bottom right corner of the bounding box

    y2: int - y coordinate of the bottom right corner of the bounding box

    label: str - label to be written on the bounding box

    color: tuple - color of the bounding box

    text_color: tuple - color of the text label

    Returns
    -------
    None

    """
    name_to_id = {
        "NA": 'NA',
        "Bullseye": 10,
        "One": 11,
        "Two": 12,
        "Three": 13,
        "Four": 14,
        "Five": 15,
        "Six": 16,
        "Seven": 17,
        "Eight": 18,
        "Nine": 19,
        "A": 20,
        "B": 21,
        "C": 22,
        "D": 23,
        "E": 24,
        "F": 25,
        "G": 26,
        "H": 27,
        "S": 28,
        "T": 29,
        "U": 30,
        "V": 31,
        "W": 32,
        "X": 33,
        "Y": 34,
        "Z": 35,
        "Up": 36,
        "Down": 37,
        "Right": 38,
        "Left": 39,
        "Up Arrow": 36,
        "Down Arrow": 37,
        "Right Arrow": 38,
        "Left Arrow": 39,
        "Stop": 40
    }
    # Reformat the label to {label name}-{label id}
    label = label + "-" + str(name_to_id[label])
    # Convert the coordinates to int
    x1 = int(x1)
    x2 = int(x2)
    y1 = int(y1)
    y2 = int(y2)
    # Create a random string to be used as the suffix for the image name, just in case the same name is accidentally used
    rand = str(int(time.time()))

    # Save the raw image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"own_results/raw_image_{label}_{rand}.jpg", img)

    # Draw the bounding box
    img = cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    # For the text background, find space required by the text so that we can put a background with that amount of width.
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    # Print the text  
    img = cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), color, -1)
    img = cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 1)
    # Save the annotated image
    cv2.imwrite(f"own_results/annotated_image_{label}_{rand}.jpg", img)


def predict_image(image, model, signal):
    """
    Predict the image using the model and save the results in the 'runs' folder
    
    Inputs
    ------
    image: str - name of the image file

    model: torch.hub.load - model to be used for prediction

    signal: str - signal to be used for filtering the predictions

    Returns
    -------
    str - predicted label
    """
    try:
        # Load the image
        img = Image.open(os.path.join('uploads', image))

        # Predict the image using the model
        results = model(img)

        # Images with predicted bounding boxes are saved in the runs folder
        results.save('runs')

        # Convert the results to a pandas dataframe and calculate the height and width of the bounding box and the area of the bounding box
        df_results = results.pandas().xyxy[0]
        df_results['bboxHt'] = df_results['ymax'] - df_results['ymin']
        df_results['bboxWt'] = df_results['xmax'] - df_results['xmin']
        df_results['bboxArea'] = df_results['bboxHt'] * df_results['bboxWt']

        # Label with largest bbox height will be last
        df_results = df_results.sort_values('bboxArea', ascending=False)

        # Filter out Bullseye
        pred_list = df_results 
        pred_list = pred_list[pred_list['name'] != 'Bullseye']
        
        # Initialize prediction to NA
        pred = 'NA'

        # Ignore Bullseye unless they are the only image detected and select the last label in the list (the last label will be the one with the largest bbox height)
        if len(pred_list) == 1:
            if pred_list.iloc[0]['name'] != 'Bullseye':
                pred = pred_list.iloc[0]

        # If more than 1 label is detected
        elif len(pred_list) > 1:

            # More than 1 Symbol detected, filter by confidence and area
            pred_shortlist = []
            current_area = pred_list.iloc[0]['bboxArea']
            # For each prediction, check if the confidence is greater than 0.5 and if the area is greater than 80% of the current area or 60% if the prediction is 'One'
            for _, row in pred_list.iterrows():
                if row['name'] != 'Bullseye' and row['confidence'] > 0.5 and ((current_area * 0.8 <= row['bboxArea']) or (row['name'] == 'One' and current_area * 0.6 <= row['bboxArea'])):
                    # Add the prediction to the shortlist
                    pred_shortlist.append(row)
                    # Update the current area to the area of the prediction
                    current_area = row['bboxArea']
            
            # If only 1 prediction remains after filtering by confidence and area
            if len(pred_shortlist) == 1:
                # Choose that prediction
                pred = pred_shortlist[0]

            # If multiple predictions remain after filtering by confidence and area
            else:
                # Use signal of {signal} to filter further 
                
                # Sort the predictions by xmin
                pred_shortlist.sort(key=lambda x: x['xmin'])

                # If signal is 'L', choose the first prediction in the list, i.e. leftmost in the image
                if signal == 'L':
                    pred = pred_shortlist[0]
                
                # If signal is 'R', choose the last prediction in the list, i.e. rightmost in the image
                elif signal == 'R':
                    pred = pred_shortlist[-1]
                
                # If signal is 'C', choose the prediction that is central in the image
                else:
                    # Loop through the predictions shortlist
                    for i in range(len(pred_shortlist)):
                        # If the xmin of the prediction is between 250 and 774, i.e. the center of the image, choose that prediction
                        if pred_shortlist[i]['xmin'] > 250 and pred_shortlist[i]['xmin'] < 774:
                            pred = pred_shortlist[i]
                            break
                    
                    # If no prediction is central, choose the one with the largest area
                    if isinstance(pred,str):
                        # Choosing one with largest area if none are central
                        pred_shortlist.sort(key=lambda x: x['bboxArea']) 
                        pred = pred_shortlist[-1]
        
        # Draw the bounding box on the image
        if not isinstance(pred,str):
            draw_own_bbox(np.array(img), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])

        name_to_id = {
            "NA": 'NA',
            "Bullseye": 10,
            "One": 11,
            "Two": 12,
            "Three": 13,
            "Four": 14,
            "Five": 15,
            "Six": 16,
            "Seven": 17,
            "Eight": 18,
            "Nine": 19,
            "A": 20,
            "B": 21,
            "C": 22,
            "D": 23,
            "E": 24,
            "F": 25,
            "G": 26,
            "H": 27,
            "S": 28,
            "T": 29,
            "U": 30,
            "V": 31,
            "W": 32,
            "X": 33,
            "Y": 34,
            "Z": 35,
            "Up": 36,
            "Down": 37,
            "Right": 38,
            "Left": 39,
            "Up Arrow": 36,
            "Down Arrow": 37,
            "Right Arrow": 38,
            "Left Arrow": 39,
            "Stop": 40
        }
        # If pred is not a string, i.e. a prediction was made and pred is not 'NA'
        if not isinstance(pred,str):
            image_id = str(name_to_id[pred['name']])
        else:
            image_id = 'NA'
        print(f"Final result: {image_id}")
        return image_id
    # If some error happened, we just return 'NA' so that the inference loop is closed
    except:
        print(f"Final result: NA")
        return 'NA'

# def predict_image_week_9(image, model):
#     # Load the image
#     img = Image.open(os.path.join('uploads', image))
#     # Run inference
#     results = model(img)
#     # Save the results
#     results.save('runs')
#     # Convert the results to a dataframe
#     df_results = results.pandas().xyxy[0]
#     # Calculate the height and width of the bounding box and the area of the bounding box
#     df_results['bboxHt'] = df_results['ymax'] - df_results['ymin']
#     df_results['bboxWt'] = df_results['xmax'] - df_results['xmin']
#     df_results['bboxArea'] = df_results['bboxHt'] * df_results['bboxWt']

#     # Label with largest bbox height will be last
#     df_results = df_results.sort_values('bboxArea', ascending=False)
#     pred_list = df_results 
#     pred = 'NA'
#     # If prediction list is not empty
#     if pred_list.size != 0:
#         # Go through the predictions, and choose the first one with confidence > 0.5
#         for _, row in pred_list.iterrows():
#             if row['name'] != 'Bullseye' and row['confidence'] > 0.5:
#                 pred = row    
#                 break

#         # Draw the bounding box on the image 
#         if not isinstance(pred,str):
#             draw_own_bbox(np.array(img), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])
        
#     # Dictionary is shorter as only two symbols, left and right are needed
#     name_to_id = {
#         "NA": 'NA',
#         "Bullseye": 10,
#         "Right": 38,
#         "Left": 39,
#         "Right Arrow": 38,
#         "Left Arrow": 39,
#     }
#     # Return the image id
#     if not isinstance(pred,str):
#         image_id = str(name_to_id[pred['name']])
#     else:
#         image_id = 'NA'
#     return image_id


# def predict_image_week_9(image, model):
#     uploads_dir = 'uploads'
#     img_path = os.path.join(uploads_dir, image)
#     img_pil = Image.open(img_path)
#     img_bgr = cv2.imread(img_path)

#     # YOLO inference
#     results = model(img_pil)
#     results.save('runs')
#     df = results.pandas().xyxy[0]

#     # Common mapping
#     name_to_id = {
#         "NA": 'NA',
#         "Bullseye": 10,
#         "Right": 38,
#         "Left": 39,
#         "Right Arrow": 38,
#         "Left Arrow": 39,
#         # Add letters if you’re detecting letters too:
#         "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
#         "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
#     }

#     # If YOLO returned nothing, do OpenCV fallback on whole image
#     if df is None or len(df) == 0:
#         label, score = opencv_fallback_whole_image(img_path)
#         if label in name_to_id and score >= 0.6:
#             draw_own_bbox(np.array(img_pil), 0, 0, img_pil.width-1, img_pil.height-1, label)  # rough box
#             return str(name_to_id[label])
#         return 'NA'

#     # YOLO found something: pick first non-Bullseye with conf > 0.5
#     df['bboxHt'] = df['ymax'] - df['ymin']
#     df['bboxWt'] = df['xmax'] - df['xmin']
#     df['bboxArea'] = df['bboxHt'] * df['bboxWt']
#     df = df.sort_values('bboxArea', ascending=False)

#     pred_row = None
#     for _, row in df.iterrows():
#         if row['name'] != 'Bullseye' and row['confidence'] > 0.5:
#             pred_row = row
#             break

#     # If still nothing confident → OpenCV fallback on whole image
#     if pred_row is None:
#         label, score = opencv_fallback_whole_image(img_path)
#         if label in name_to_id and score >= 0.6:
#             draw_own_bbox(np.array(img_pil), 0, 0, img_pil.width-1, img_pil.height-1, label)
#             return str(name_to_id[label])
#         return 'NA'

#     # Cross-check the YOLO ROI with OpenCV (template match)
#     x1, y1, x2, y2 = map(int, [pred_row['xmin'], pred_row['ymin'], pred_row['xmax'], pred_row['ymax']])
#     x1 = max(0, x1); y1 = max(0, y1); x2 = min(img_bgr.shape[1]-1, x2); y2 = min(img_bgr.shape[0]-1, y2)
#     roi_bgr = img_bgr[y1:y2, x1:x2].copy()

#     cv_label, cv_score = _best_template_match(roi_bgr)

#     # Decision logic:
#     # - If OpenCV strongly agrees or YOLO’s label isn’t in our map, use OpenCV.
#     # - Else keep YOLO.
#     yolo_label = pred_row['name']
#     yolo_conf = float(pred_row['confidence'])

#     # You can tune these thresholds
#     CV_STRONG = 0.70
#     CV_OK = 0.60

#     final_label = yolo_label
#     if cv_label in name_to_id:
#         if cv_label == yolo_label and cv_score >= CV_OK:
#             final_label = yolo_label  # agreement
#         elif cv_score >= CV_STRONG:
#             final_label = cv_label    # override on strong OpenCV evidence
#         # else keep YOLO

#     # Draw
#     if final_label in name_to_id and final_label != "Bullseye":
#         draw_own_bbox(np.array(img_pil), x1, y1, x2, y2, final_label)

#     return str(name_to_id.get(final_label, 'NA'))

def predict_image_week_9(image, model):
    uploads_dir = 'uploads'
    img_path = os.path.join(uploads_dir, image)

    bgr = cv2.imread(img_path)
    if bgr is None:
        print("Failed to read image:", img_path)
        return 'NA'

    # Inference (numpy BGR is supported by YOLOv5 hub model)
    results = model(bgr, size=YOLO_IMG_SIZE)
    # Save annotated frame to `runs/` (kept as requested)
    try:
        results.save('runs')
    except Exception as e:
        print("[warn] results.save failed:", e)

    # Fast parse without pandas
    pred = results.pred[0]           # tensor [N,6] => (x1,y1,x2,y2,conf,cls)
    names = results.names            # id -> label dict

    # If YOLO returned nothing → fallback
    if pred is None or pred.shape[0] == 0:
        label, score = opencv_fallback_whole_image(img_path)
        name_to_id = {
            "NA": 'NA', "Bullseye": 10,
            "Right": 38, "Left": 39, "Right Arrow": 38, "Left Arrow": 39,
            "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
            "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
            "One": 11, "Two": 12, "Three": 13, "Four": 14, "Five": 15,
            "Six": 16, "Seven": 17, "Eight": 18, "Nine": 19, "Stop": 40
        }
        if label in name_to_id and score >= 0.60:
            # rough box (full frame) for visualization
            draw_own_bbox(bgr, 0, 0, bgr.shape[1] - 1, bgr.shape[0] - 1, label)
            return str(name_to_id[label])
        return 'NA'

    # Choose best (largest area) valid detection with conf>0.5 and not Bullseye
    boxes = pred.cpu().numpy()
    # sort by area desc
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

    # If nothing confident → fallback (fast)
    if final_label is None:
        label, score = opencv_fallback_whole_image(img_path)
        name_to_id = {
            "NA": 'NA', "Bullseye": 10,
            "Right": 38, "Left": 39, "Right Arrow": 38, "Left Arrow": 39,
            "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
            "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
            "One": 11, "Two": 12, "Three": 13, "Four": 14, "Five": 15,
            "Six": 16, "Seven": 17, "Eight": 18, "Nine": 19, "Stop": 40
        }
        if label in name_to_id and score >= 0.60:
            draw_own_bbox(bgr, 0, 0, bgr.shape[1] - 1, bgr.shape[0] - 1, label)
            return str(name_to_id[label])
        return 'NA'

    # Draw and return ID
    x1, y1, x2, y2 = final_box
    draw_own_bbox(bgr, x1, y1, x2, y2, final_label)

    name_to_id = {
        "NA": 'NA', "Bullseye": 10,
        "One": 11, "Two": 12, "Three": 13, "Four": 14, "Five": 15,
        "Six": 16, "Seven": 17, "Eight": 18, "Nine": 19,
        "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
        "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
        "Up": 36, "Down": 37, "Right": 38, "Left": 39,
        "Up Arrow": 36, "Down Arrow": 37, "Right Arrow": 38, "Left Arrow": 39,
        "Stop": 40
    }
    return str(name_to_id.get(final_label, 'NA'))


def stitch_image():
    """
    Stitches the images in the folder together and saves it into runs/stitched folder
    """
    # Initialize path to save stitched image
    imgFolder = 'runs'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    # Find all files that ends with ".jpg" (this won't match the stitched images as we name them ".jpeg")
    imgPaths = glob.glob(os.path.join(imgFolder+"/detect/*/", "*.jpg"))
    # Open all images
    images = [Image.open(x) for x in imgPaths]
    # Get the width and height of each image
    width, height = zip(*(i.size for i in images))
    # Calculate the total width and max height of the stitched image, as we are stitching horizontally
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0

    # Stitch the images together
    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    # Save the stitched image to the path
    stitchedImg.save(stitchedPath)

    # Move original images to "originals" subdirectory
    for img in imgPaths:
        shutil.move(img, os.path.join(
            "runs", "originals", os.path.basename(img)))

    return stitchedImg

def stitch_image_own():
    """
    Stitches the images in the folder together and saves it into own_results folder

    Basically similar to stitch_image() but with different folder names and slightly different drawing of bounding boxes and text
    """
    imgFolder = 'own_results'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    imgPaths = glob.glob(os.path.join(imgFolder+"/annotated_image_*.jpg"))
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


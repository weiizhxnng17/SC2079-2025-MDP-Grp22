



# import os
# import time
# from flask import Flask, request, jsonify
# from flask_cors import CORS
# from werkzeug.utils import secure_filename

# # import only what we use from model1.py
# from model3 import (
#     load_model,
#     load_opencv_templates,
#     predict_image_yolo,
#     predict_image_opencv,
#     stitch_image,
#     stitch_image_own,
# )

# app = Flask(__name__)
# CORS(app)

# # ensure uploads dir exists
# os.makedirs("uploads", exist_ok=True)

# # init once at startup
# model = load_model()
# load_opencv_templates()



# def _extract_obstacle_id(filename: str) -> str:
#     """
#     expected filename format: "<timestamp>_<obstacle_id>_<...>.jpg"
#     if format is unexpected, return "NA" instead of crashing.
#     """
#     parts = filename.split("_")
#     return parts[1] if len(parts) >= 2 else "NA"


# @app.route('/status', methods=['GET'])
# def status():
#     return jsonify({"result": "okk"})


# @app.route('/imageyolo', methods=['POST'])
# def image_yolo():
#     """
#     YOLO-only prediction. Fast path; no OpenCV fallback.
#     Returns: {"obstacle_id": "...", "image_id": "..."}
#     """
    
#     if 'file' not in request.files:
#         return jsonify({"error": "missing file"}), 400

#     file = request.files['file']
#     if not file or file.filename == '':
#         return jsonify({"error": "empty filename"}), 400

#     filename = secure_filename(file.filename)
#     save_path = os.path.join('uploads', filename)
#     file.save(save_path)

#     obstacle_id = _extract_obstacle_id(filename)
#     image_id = predict_image_yolo(filename, model)

#     return jsonify({"obstacle_id": 1, "image_id": image_id})


# @app.route('/imageopencv', methods=['POST'])
# def image_opencv():
#     """
#     OpenCV-only prediction. Thorough template matching.
#     Returns: {"obstacle_id": "...", "image_id": "..."}
#     """
#     if 'file' not in request.files:
#         return jsonify({"error": "missing file"}), 400

#     file = request.files['file']
#     if not file or file.filename == '':
#         return jsonify({"error": "empty filename"}), 400

#     filename = secure_filename(file.filename)
#     save_path = os.path.join('uploads', filename)
#     file.save(save_path)

#     obstacle_id = _extract_obstacle_id(filename)
#     image_id = predict_image_opencv(filename)

#     # return jsonify({"obstacle_id": obstacle_id, "image_id": image_id})
#     return jsonify({"obstacle_id": 1, "image_id": image_id})
    


# @app.route('/stitch', methods=['GET'])
# def stitch():
#     """
#     Stitches images from YOLO and own_results into two separate strips.
#     """
#     img1 = stitch_image()
#     img2 = stitch_image_own()
#     # no GUI .show() in server context
#     return jsonify({"result": "ok"})


# # (optional) legacy notice if someone still calls /image
# @app.route('/image', methods=['POST'])
# def legacy_image():
#     return jsonify({"error": "endpoint moved. use /imageyolo or /imageopencv"}), 410


# if __name__ == '__main__':
#     # debug=True is handy; flip to False in production
#     app.run(host='0.0.0.0', port=5000, debug=False)



import os
import time
from flask import Flask, request, jsonify
from flask_cors import CORS
from werkzeug.utils import secure_filename
import imghdr

from model3 import (
    load_model,
    load_opencv_templates,
    predict_image_yolo,
    predict_image_opencv,
    stitch_image,
    stitch_image_own,
)

app = Flask(__name__)
CORS(app)

os.makedirs("uploads", exist_ok=True)

model = load_model()
load_opencv_templates()

# -------------------- NEW: constants and counter --------------------
STITCH_TRIGGER_COUNT = 2
IMAGEYOLO_CALL_COUNT = 0


def _extract_obstacle_id(filename: str) -> str:
    parts = filename.split("_")
    return parts[1] if len(parts) >= 2 else "NA"


@app.route('/status', methods=['GET'])
def status():
    return jsonify({"result": "ok"})


@app.route('/imageyolo', methods=['POST'])
def image_yolo():
    """
    YOLO-only prediction. Fast path; no OpenCV fallback.
    Automatically stitches every 8 calls.
    """
    global IMAGEYOLO_CALL_COUNT  # <--- track counter

    if 'file' not in request.files:
        return jsonify({"error": "missing file"}), 400

    file = request.files['file']
    if not file or file.filename == '':
        return jsonify({"error": "empty filename"}), 400

    # --- Save the uploaded file temporarily ---
    raw_filename = secure_filename(file.filename or f"{int(time.time())}")
    save_path = os.path.join('uploads', raw_filename)
    file.save(save_path)

    # --- Detect image type ---
    detected_type = imghdr.what(save_path)
    if detected_type:
        # Rename with detected extension
        new_filename = f"{raw_filename}.{detected_type}"
        new_path = os.path.join('uploads', new_filename)
        os.rename(save_path, new_path)
        save_path = new_path
    else:
        print(f"[WARN] Could not detect image type for {raw_filename}, keeping original")

    print(f"[DEBUG] Saved image to {save_path}")

    # --- Normal YOLO flow ---
    obstacle_id = _extract_obstacle_id(raw_filename)
    image_id = predict_image_yolo(os.path.basename(save_path), model)

    # increment counter
    IMAGEYOLO_CALL_COUNT += 1
    print(f"[INFO] /imageyolo called {IMAGEYOLO_CALL_COUNT} times")

    # trigger stitching when threshold reached
    stitched_path = None
    if IMAGEYOLO_CALL_COUNT >= STITCH_TRIGGER_COUNT:
        print("[INFO] Threshold reached — stitching images now...")
        stitched_img = stitch_image_own()
        if stitched_img:
            stitched_path = os.path.join("own_results", f"stitched-{int(time.time())}.jpeg")
            stitched_img.save(stitched_path)
            print(f"[INFO] Stitched image saved: {stitched_path}")
        else:
            print("[WARN] No images found to stitch.")
        IMAGEYOLO_CALL_COUNT = 0  # reset counter

    print(f"[INFO] image id and obstacle id are {image_id} and {obstacle_id} ")

    result = {
        "obstacle_id": obstacle_id,
        "image_id": image_id,
    }
    return jsonify(result)


@app.route('/imageopencv', methods=['POST'])
def image_opencv():
    if 'file' not in request.files:
        return jsonify({"error": "missing file"}), 400

    file = request.files['file']
    if not file or file.filename == '':
        return jsonify({"error": "empty filename"}), 400

    filename = secure_filename(file.filename)
    save_path = os.path.join('uploads', filename)
    file.save(save_path)

    obstacle_id = _extract_obstacle_id(filename)
    image_id = predict_image_opencv(filename)

    return jsonify({"obstacle_id": obstacle_id, "image_id": image_id})


@app.route('/stitch', methods=['GET'])
def stitch():
    img1 = stitch_image()
    img2 = stitch_image_own()
    return jsonify({"result": "ok"})


@app.route('/image', methods=['POST'])
def legacy_image():
    return jsonify({"error": "endpoint moved. use /imageyolo or /imageopencv"}), 410


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)


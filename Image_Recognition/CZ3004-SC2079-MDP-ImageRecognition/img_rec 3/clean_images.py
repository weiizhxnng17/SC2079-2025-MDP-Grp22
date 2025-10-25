import cv2
import numpy as np
from pathlib import Path

SRC = Path("../data/cropped")
DST = Path("../data/final")
DST.mkdir(parents=True, exist_ok=True)

# Tweakables (increase kernel if specks are larger)
TARGET_SIZE = (1024, 1024)
KERNEL = np.ones((10, 10), np.uint8)    # try (5,5) if reflections are strong

def load_as_gray(p):
    img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    # Drop alpha if present, convert to gray
    if img.ndim == 3:
        if img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

def binarize_clean(gray):
    # Light denoise but keep edges
    gray = cv2.medianBlur(gray, 3)

    # Otsu threshold: white background (255), black ink (0)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Invert so "ink" is foreground (255) for morphology
    inv = 255 - bw

    # 1) Closing fills tiny holes (white specks in letters)
    inv_closed = cv2.morphologyEx(inv, cv2.MORPH_CLOSE, KERNEL, iterations=1)

    # 2) Opening removes tiny isolated bits (noise/dust)
    inv_clean = cv2.morphologyEx(inv_closed, cv2.MORPH_OPEN, KERNEL, iterations=1)

    # Back to black text (0) on white background (255)
    cleaned = 255 - inv_clean

    # Force strictly 0/255
    cleaned = np.where(cleaned > 127, 255, 0).astype(np.uint8)
    return cleaned

def process_one(src_path, dst_path):
    gray = load_as_gray(src_path)
    if gray is None:
        print(f"Skipping unreadable file: {src_path}")
        return

    # Stretch to 1024x1024 (no crop)
    resized = cv2.resize(gray, TARGET_SIZE, interpolation=cv2.INTER_LINEAR)

    # Binarize + clean reflections/noise
    final = binarize_clean(resized)

    # Save (use PNG to avoid JPEG artifacts)
    out_path = dst_path.with_suffix(".png")
    cv2.imwrite(str(out_path), final)

def main():
    files = [p for p in SRC.iterdir() if p.suffix.lower() == '.jpg' and p.is_file()]
    if not files:
        print("No images found in /cropped.")
        return

    for p in files:
        out = DST / p.name
        process_one(p, out)
    print(f"Done. Cleaned images saved to: {DST.resolve()}")

if __name__ == "__main__":
    main()

# run_stitch.py

from model3 import stitch_image, stitch_image_own

def main():
    print("[INFO] Running stitching manually...")

    # Option 1: YOLO detection results
    stitched1 = stitch_image()
    if stitched1:
        print("[INFO] YOLO stitched image saved successfully.")
    else:
        print("[WARN] No YOLO images found to stitch.")

    # Option 2: Your own annotated images (from draw_own_bbox)
    stitched2 = stitch_image_own()
    if stitched2:
        print("[INFO] Own annotated stitched image saved successfully.")
    else:
        print("[WARN] No annotated images found to stitch.")

if __name__ == "__main__":
    main()

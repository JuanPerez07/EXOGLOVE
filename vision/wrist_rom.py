"""
This module implements the main vision system for measuring wrist range of motion (ROM) using a single camera. 
It detects colored markers on reference points, calculates angles, and provides real-time feedback.
"""
import cv2 as cv
import numpy as np
import os
from datetime import datetime
# Source directory for intrinsic and extrinsic parameters
PARAM_DIR = "params/"

# Select active colors
SELECTED_ORIGIN_COLOR = 'green'
SELECTED_MOBILE_COLOR = 'yellow'

# Directory to save images
IMG_DIR = "TIMESTAMP_IMG"

# HSV Color Ranges configuration
HSV_RANGES = {
    'origin': {
        'yellow': {'min': np.array([20, 100, 100]), 'max': np.array([40, 255, 255])},
        'red': {'min': np.array([0, 100, 100]), 'max': np.array([10, 255, 255])},
        'green': {'min': np.array([40, 100, 100]), 'max': np.array([80, 255, 255])},
        'blue': {'min': np.array([100, 150, 0]), 'max': np.array([140, 255, 255])}
    },
    'mobile': {
        'yellow': {'min': np.array([20, 100, 100]), 'max': np.array([40, 255, 255])},
        'red': {'min': np.array([0, 100, 100]), 'max': np.array([10, 255, 255])},
        'green': {'min': np.array([40, 100, 100]), 'max': np.array([80, 255, 255])},
        'blue': {'min': np.array([100, 150, 0]), 'max': np.array([140, 255, 255])}
    }
}



def get_centroid(mask):
    """ Calculates the centroid of the largest contour in a binary mask. """
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Get the largest contour
    c_largest = max(contours, key=cv.contourArea)
    M = cv.moments(c_largest)
    
    # Avoid division by zero
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    else:
        return None

def load_params(param_dir):
    """ Loads camera matrix and distortion coefficients. """
    try:
        K = np.load(param_dir + "camera_matrix.npy")
        D = np.load(param_dir + "dist_coeffs.npy")
        return K, D
    except FileNotFoundError:
        print("Calibration parameters not found. Please run camera_calib.py first.")
        return None, None

def image_processing(frame):
    """ Applies pre-processing: blur and color conversion. """
    # 1. Noise Reduction
    frame_blur = cv.GaussianBlur(frame, (5, 5), 0)
    # 2. Color Space Conversion
    frame_hsv = cv.cvtColor(frame_blur, cv.COLOR_BGR2HSV)
    return frame_hsv

def segment_hsv(frame_hsv, lower, upper, erode=False):
    """ Segments the image based on color range and finds the centroid. """
    mask = cv.inRange(frame_hsv, lower, upper)
    if erode:
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask = cv.erode(mask, kernel, iterations=1)
    return get_centroid(mask), mask

def calculate_angle(pt_origin, pt_mobile, K, D):
    """ Undistorts points and calculates the angle between them. """
    # 5. Lens Correction (Undistort Points)
    pts_src = np.array([[pt_origin], [pt_mobile]], dtype=np.float32)
    pts_corr = cv.undistortPoints(pts_src, K, D, P=K)
    
    pt_origen_corr = pts_corr[0][0]
    pt_movil_corr  = pts_corr[1][0]

    # 6. Calculate Deltas
    dx = pt_movil_corr[0] - pt_origen_corr[0]
    # Invert Y because in images (0,0) is top-left
    dy = (pt_origen_corr[1] - pt_movil_corr[1])

    # 7. Strict Trigonometric Calculation
    angulo_rad = np.arctan2(dy, dx)
    return np.degrees(angulo_rad)

def main():
    # --- BLOCK 1: CONFIGURATION AND INITIALIZATION (SETUP) ---
    
    # Load Calibration Parameters (Obtained previously with Chessboard)
    K, D = load_params(PARAM_DIR)
    if K is None:
        return

    # Global Variables
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return
    # Initial angle for delta calculation
    last_angle = None
    angle_delta = None
    # --- BLOCK 3: MAIN LOOP ---

    while True:
        ret, frame_raw = cap.read()
        if not ret:
            break

        # --- PHASE A & B: PROCESSING & SEGMENTATION ---
        frame_hsv = image_processing(frame_raw)

        pt_origen_raw, mask_origin = segment_hsv(frame_hsv, HSV_RANGES['origin'][SELECTED_ORIGIN_COLOR]['min'], HSV_RANGES['origin'][SELECTED_ORIGIN_COLOR]['max'])
        pt_movil_raw, mask_mobile  = segment_hsv(frame_hsv, HSV_RANGES['mobile'][SELECTED_MOBILE_COLOR]['min'], HSV_RANGES['mobile'][SELECTED_MOBILE_COLOR]['max'], erode=True)

        # --- PHASE C: LOGIC AND CORRECTION (KEY OPTIMIZATION) ---

        current_angle = None
        

        if pt_origen_raw is not None and pt_movil_raw is not None:
            # --- PHASE D: MATHEMATICAL CALCULATION ---
            angulo_deg = calculate_angle(pt_origen_raw, pt_movil_raw, K, D)
            current_angle = angulo_deg

            # --- PHASE E: VISUALIZATION (UI) ---
            
            # Draw on the original frame for visual feedback
            line_color = (0,0,255) # red line
            cv.line(frame_raw, pt_origen_raw, pt_movil_raw, line_color, 2)
            cv.putText(frame_raw, f"Angulo: {angulo_deg:.2f}", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if angle_delta is not None:
                cv.putText(frame_raw, f"Delta: {angle_delta:.2f}", (10, 80), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        else:
            cv.putText(frame_raw, "Puntos no detectados", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # --- FINALIZATION ---
        cv.imshow("Sistema de Vision", frame_raw)
        cv.imshow("Mask Origin detects " + SELECTED_ORIGIN_COLOR, mask_origin)
        cv.imshow("Mask Mobile detects " + SELECTED_MOBILE_COLOR, mask_mobile)

        k = cv.waitKey(1)
        if k == ord('q'):
            break
        elif k == 13: # Enter key to calculate delta 
            # Calculate angle delta if we have a previous angle
            if last_angle is not None:
                angle_delta = abs(current_angle - last_angle)
            # Update last angle for next delta calculation
            last_angle = current_angle 

            # Save image
            os.makedirs(IMG_DIR, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(IMG_DIR, f"img_{timestamp}.png")
            cv.imwrite(filename, frame_raw)
            print(f"Imagen guardada: {filename}")

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
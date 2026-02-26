"""
This module implements the main vision system for measuring finger range of motion (ROM) using a single camera
"""
"""
This module implements a simplified vision system for measuring finger range of motion (ROM) 
from a static image. It detects colored markers, calculates the angle, and displays the result.
"""
import cv2 as cv
import numpy as np
import argparse

# Select active colors
SELECTED_ORIGIN_COLOR = 'yellow'
SELECTED_MOBILE_COLOR = 'green'

# HSV Color Ranges configuration
HSV_RANGES = {
    'origin': {
        'yellow': {'min': np.array([20, 100, 100]), 'max': np.array([40, 255, 255])},
        'red': {'min': np.array([0, 100, 100]), 'max': np.array([10, 255, 255])},
        'green': {'min': np.array([50, 100, 50]), 'max': np.array([80, 255, 255])},
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
        kernel = cv.getStructuringElement(cv.MORPH_CROSS, (6, 6))
        mask = cv.erode(mask, kernel, iterations=3)
    return get_centroid(mask), mask

def calculate_angle(pt_origin, pt_mobile):
    """ Calculates the angle between two raw points directly. """
    # Calculate Deltas directly from raw pixel coordinates
    dx = pt_mobile[0] - pt_origin[0]
    # Invert Y because in images (0,0) is top-left
    dy = (pt_origin[1] - pt_mobile[1])

    # Strict Trigonometric Calculation
    angulo_rad = np.arctan2(dy, dx)
    return np.degrees(angulo_rad)

def main(image_path):
    # --- BLOCK 1: INITIALIZATION ---
    # Load the static image
    frame_raw = cv.imread(image_path)
    if frame_raw is None:
        print(f"Error: No se pudo cargar la imagen en la ruta '{image_path}'. Verifique que el archivo exista.")
        return

    # --- BLOCK 2: PROCESSING & SEGMENTATION ---
    frame_hsv = image_processing(frame_raw)

    pt_origen_raw, mask_origin = segment_hsv(
        frame_hsv, 
        HSV_RANGES['origin'][SELECTED_ORIGIN_COLOR]['min'], 
        HSV_RANGES['origin'][SELECTED_ORIGIN_COLOR]['max']
    )
    
    pt_movil_raw, mask_mobile  = segment_hsv(
        frame_hsv, 
        HSV_RANGES['mobile'][SELECTED_MOBILE_COLOR]['min'], 
        HSV_RANGES['mobile'][SELECTED_MOBILE_COLOR]['max'], 
        erode=True
    )

    # --- BLOCK 3: LOGIC AND CALCULATION ---
    if pt_origen_raw is not None and pt_movil_raw is not None:
        # Calculate angle
        angulo_deg = calculate_angle(pt_origen_raw, pt_movil_raw)

        # --- BLOCK 4: VISUALIZATION (UI) ---
        # Draw line and angle text
        line_color = (0, 130, 255) # Orange line (BGR format)
        text_color = (180, 105, 255) # Magenta text (BGR format)
        cv.line(frame_raw, pt_origen_raw, pt_movil_raw, line_color, 2)
        cv.putText(frame_raw, f"Angulo: {angulo_deg:.2f}", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, text_color, 2)
    else:
        cv.putText(frame_raw, "Puntos no detectados", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, text_color, 2)

    # --- BLOCK 5: FINALIZATION ---
    cv.imshow("Sistema de Vision - Finger ROM", frame_raw)
    cv.imshow("Mask Origin (" + SELECTED_ORIGIN_COLOR + ")", mask_origin)
    cv.imshow("Mask Mobile (" + SELECTED_MOBILE_COLOR + ")", mask_mobile)

    print("Presiona cualquier tecla en las ventanas de imagen para salir...")
    cv.waitKey(0) # Waits indefinitely until a key is pressed
    cv.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mide el ROM del dedo a partir de una imagen.")
    parser.add_argument('-i', '--image', type=str, required=True, help='Ruta a la imagen que se va a procesar')
    args = parser.parse_args()
    
    main(args.image)
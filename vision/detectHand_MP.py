"""
This module implements a vision system for measuring finger angles 
using MediaPipe hand tracking on a static image.
It calculates the angles between MCP-PIP and PIP-TIP segments for each finger.
"""
import cv2 as cv
import mediapipe as mp
import os
import numpy as np
import argparse

# MediaPipe dictionary mapping fingers to their MCP, PIP and TIP landmark indices
FINGERS_LANDMARKS = {
    'Thumb':  {'mcp': 2,  'pip': 3,  'tip': 4},
    'Index':  {'mcp': 5,  'pip': 6,  'tip': 8},
    'Middle': {'mcp': 9,  'pip': 10, 'tip': 12},
    'Ring':   {'mcp': 13, 'pip': 14, 'tip': 16},
    'Pinky':  {'mcp': 17, 'pip': 18, 'tip': 20}
}

# Colors for visualization (BGR)
COLORS = {
    'Thumb':  (0, 0, 255),    # Red
    'Index':  (0, 255, 255),  # Yellow
    'Middle': (0, 255, 0),    # Green
    'Ring':   (255, 0, 0),    # Blue
    'Pinky':  (255, 0, 255)   # Magenta
}

def calculate_angle(pt_origin, pt_mobile):
    """ Calculates the angle between two points directly (in degrees). """
    dx = pt_mobile[0] - pt_origin[0]
    # Invert Y because in images (0,0) is top-left
    dy = (pt_origin[1] - pt_mobile[1])

    # Strict Trigonometric Calculation
    angulo_rad = np.arctan2(dy, dx)
    return np.degrees(angulo_rad)

def get_pixel_coords(landmark, img_width, img_height):
    """ Converts normalized MediaPipe coordinates (0-1) to pixel coordinates. """
    return int(landmark.x * img_width), int(landmark.y * img_height)

def main(image_path):
    # --- BLOCK 1: INITIALIZATION ---
    # Load the static image
    frame_raw = cv.imread(image_path)
    if frame_raw is None:
        print(f"Error: No se pudo cargar la imagen en la ruta '{image_path}'.")
        return

    height, width, _ = frame_raw.shape
    
    # Initialize MediaPipe Hands
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    
    # Usamos static_image_mode=True porque es una sola imagen, no un stream de video
    with mp_hands.Hands(
        static_image_mode=True, 
        max_num_hands=2, 
        min_detection_confidence=0.5
    ) as hands:
        
        # --- BLOCK 2: PROCESSING ---
        # MediaPipe requires RGB images
        frame_rgb = cv.cvtColor(frame_raw, cv.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        # --- BLOCK 3: LOGIC AND VISUALIZATION ---
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                
                # Opcional: Dibujar todas las conexiones de la mano de fondo
                mp_drawing.draw_landmarks(
                    frame_raw, 
                    hand_landmarks, 
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(200, 200, 200), thickness=1, circle_radius=1),
                    mp_drawing.DrawingSpec(color=(150, 150, 150), thickness=1)
                )

                # Iterate through each finger to calculate and draw our specific angles
                for finger_name, indices in FINGERS_LANDMARKS.items():
                    # Extract normalized landmarks
                    mcp_norm = hand_landmarks.landmark[indices['mcp']]
                    pip_norm = hand_landmarks.landmark[indices['pip']]
                    tip_norm = hand_landmarks.landmark[indices['tip']]

                    # Convert to pixel coordinates for math and drawing
                    pt_mcp = get_pixel_coords(mcp_norm, width, height)
                    pt_pip = get_pixel_coords(pip_norm, width, height)
                    pt_tip = get_pixel_coords(tip_norm, width, height)

                    # Calculate the angles
                    angle_mcp_pip = calculate_angle(pt_mcp, pt_pip)
                    angle_pip_tip = calculate_angle(pt_pip, pt_tip)

                    # Draw the specific lines for MCP->PIP and PIP->TIP
                    color = COLORS[finger_name]
                    cv.line(frame_raw, pt_mcp, pt_pip, color, 3)
                    cv.line(frame_raw, pt_pip, pt_tip, color, 3)
                    cv.circle(frame_raw, pt_mcp, 5, color, -1)
                    cv.circle(frame_raw, pt_pip, 5, color, -1)
                    cv.circle(frame_raw, pt_tip, 5, color, -1)

                    # Put text with the angles near the joints
                    text_position_pip = (pt_pip[0] - 20, pt_pip[1] - 15)
                    text_position_tip = (pt_tip[0] - 20, pt_tip[1] - 15)
                    cv.putText(frame_raw, f"{angle_mcp_pip:.1f}", text_position_pip, 
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv.putText(frame_raw, f"{angle_pip_tip:.1f}", text_position_tip, 
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    print(f"{finger_name} MCP-PIP Angle: {angle_mcp_pip:.2f} grados")
                    print(f"{finger_name} PIP-TIP Angle: {angle_pip_tip:.2f} grados")
        else:
            cv.putText(frame_raw, "No se detectaron manos", (20, 50), 
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            print("No se detectaron manos en la imagen.")

        # --- BLOCK 4: FINALIZATION ---
        save_segmented_image(image_path, frame_raw)

def save_segmented_image(input_image_path, image, output_dir_name="segmented_img"):
    """Save the processed image in a segmented_img folder adjacent to the input image."""
    input_dir = os.path.dirname(os.path.abspath(input_image_path))
    output_dir = os.path.join(input_dir, output_dir_name)
    os.makedirs(output_dir, exist_ok=True)

    base_name = os.path.basename(input_image_path)
    output_path = os.path.join(output_dir, base_name)

    success = cv.imwrite(output_path, image)
    if success:
        print(f"Imagen guardada en: {output_path}")
    else:
        print(f"Error: No se pudo guardar la imagen en '{output_path}'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calcula los ángulos MCP-PIP y PIP-TIP de los dedos usando MediaPipe.")
    parser.add_argument('-i', '--image', type=str, required=True, help='Ruta a la imagen que se va a procesar')
    args = parser.parse_args()
    
    main(args.image)
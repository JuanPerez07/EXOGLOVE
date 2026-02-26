"""
This module implements a vision system for measuring finger angles 
using MediaPipe hand tracking on a static image.
It calculates the angle between the MCP joint and the Tip for each finger.
"""
import cv2 as cv
import mediapipe as mp
import numpy as np
import argparse

# MediaPipe dictionary mapping fingers to their MCP and TIP landmark indices
FINGERS_LANDMARKS = {
    'Thumb':  {'mcp': 2,  'tip': 4},
    'Index':  {'mcp': 5,  'tip': 8},
    'Middle': {'mcp': 9,  'tip': 12},
    'Ring':   {'mcp': 13, 'tip': 16},
    'Pinky':  {'mcp': 17, 'tip': 20}
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
                    tip_norm = hand_landmarks.landmark[indices['tip']]

                    # Convert to pixel coordinates for math and drawing
                    pt_mcp = get_pixel_coords(mcp_norm, width, height)
                    pt_tip = get_pixel_coords(tip_norm, width, height)

                    # Calculate the angle
                    angle_deg = calculate_angle(pt_mcp, pt_tip)

                    # Draw the specific line between MCP and TIP
                    color = COLORS[finger_name]
                    cv.line(frame_raw, pt_mcp, pt_tip, color, 3)
                    cv.circle(frame_raw, pt_mcp, 5, color, -1)
                    cv.circle(frame_raw, pt_tip, 5, color, -1)

                    # Put text with the angle near the tip of the finger
                    text_position = (pt_tip[0] - 20, pt_tip[1] - 15)
                    cv.putText(frame_raw, f"{angle_deg:.1f}", text_position, 
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    print(f"{finger_name} Angle: {angle_deg:.2f} grados")
        else:
            cv.putText(frame_raw, "No se detectaron manos", (20, 50), 
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            print("No se detectaron manos en la imagen.")

        # --- BLOCK 4: FINALIZATION ---
        cv.imshow("MediaPipe - Deteccion de Mano y Angulos", frame_raw)
        print("\nPresiona cualquier tecla en la ventana de la imagen para salir...")
        cv.waitKey(0) # Wait indefinitely until a key is pressed
        cv.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calcula los ángulos MCP-TIP de los dedos usando MediaPipe.")
    parser.add_argument('-i', '--image', type=str, required=True, help='Ruta a la imagen que se va a procesar')
    args = parser.parse_args()
    
    main(args.image)
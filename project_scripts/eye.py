import cv2 
import mediapipe as mp
import numpy as np
import time
import transmitter
mode = "EYE"
# -----------------------
# Helper Functions
# -----------------------
# Flag to track if "STOP" has been sent
last_stop_direction = True
right_left_check = True

def Direction_Map_func(detected_dir):
    global right_left_check
    global last_stop_direction  
    if (detected_dir == "Looking Right") and last_stop_direction:
        right_left_check = False
        return "RIGHT"
    elif (detected_dir == "Looking Left") and last_stop_direction:
        right_left_check = False
        return "LEFT"
    elif (detected_dir == "Center") and last_stop_direction  and right_left_check:
        return "FORWARD"
    elif detected_dir == "Looking Up":
        right_left_check = True
        last_stop_direction = True
        return "FORWARD"
    else:
        last_stop_direction = False
        return "STOP"
    
def calculate_distance(point1, point2):
    """Compute Euclidean distance between two (x,y) points."""
    return np.linalg.norm(np.array(point1) - np.array(point2))

def get_eye_info(face_landmarks, eye_side, h, w):

    if eye_side == "right":
        pupil_idx, inner_idx, outer_idx, top_idx, bottom_idx = 473, 362, 263, 386, 374
    elif eye_side == "left":
        pupil_idx, inner_idx, outer_idx, top_idx, bottom_idx = 468, 133, 33, 159, 145
    else:
        raise ValueError("eye_side must be 'right' or 'left'")
    
    # Convert normalized landmarks to pixel coordinates.
    pupil = face_landmarks.landmark[pupil_idx]
    inner_corner = face_landmarks.landmark[inner_idx]
    outer_corner = face_landmarks.landmark[outer_idx]
    top = face_landmarks.landmark[top_idx]
    bottom = face_landmarks.landmark[bottom_idx]
    
    pupil_coords = (int(pupil.x * w), int(pupil.y * h))
    inner_coords = (int(inner_corner.x * w), int(inner_corner.y * h))
    outer_coords = (int(outer_corner.x * w), int(outer_corner.y * h))
    top_coords = (int(top.x * w), int(top.y * h))
    bottom_coords = (int(bottom.x * w), int(bottom.y * h))
    
    # Compute horizontal ratio.
    d_inner = calculate_distance(pupil_coords, inner_coords)
    d_outer = calculate_distance(pupil_coords, outer_coords)
    if eye_side == "right":
        hor_ratio = d_inner / (d_inner + d_outer + 1e-6)
    else:
        hor_ratio = 1 - (d_inner / (d_inner + d_outer + 1e-6))
    
    # Compute vertical ratio (normalized position within the eye).
    vert_ratio = (pupil_coords[1] - top_coords[1]) / (bottom_coords[1] - top_coords[1] + 1e-6)
    
    return {
        'pupil': pupil_coords,
        'inner': inner_coords,
        'outer': outer_coords,
        'top': top_coords,
        'bottom': bottom_coords,
        'hor_ratio': hor_ratio,
        'vert_ratio': vert_ratio
    }

# -----------------------
# Calibration Variables
# -----------------------
calibration_mode = False
calib_states = ["Center", "Left", "Right"]
calib_index = 0  # 0: Center, 1: Left, 2: Right
calib_data = {"Center": [], "Left": [], "Right": []}
NUM_SAMPLES = 20  # Number of samples per state

# Default thresholds (these will be updated after calibration)
left_threshold = 0.4
right_threshold = 0.6

# Threshold to determine if the eye is closed (eye opening ratio).
# (This ratio is computed as vertical eye opening divided by horizontal eye width.)
EYE_CLOSED_THRESHOLD = 0.25

# -----------------------
# Main Detection Function
# -----------------------
def detect_eye_direction():
    global calibration_mode, calib_index, calib_data, left_threshold, right_threshold

    # Initialize MediaPipe Face Mesh.
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    cap = cv2.VideoCapture(0)  # Use your preferred camera index
    
    # Set resolution to 1920x1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)     # Set width
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)    # Set height

    stable_direction = "No face detected"
    last_direction = ""
    consistent_frames = 0

    while cap.isOpened():
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb_frame)

        current_direction = "No face"
        chosen_ratio = None  # Will hold the selected horizontal ratio

        if results.multi_face_landmarks:
            # Process the first detected face.
            face_landmarks = results.multi_face_landmarks[0]
            right_eye = get_eye_info(face_landmarks, "right", h, w)
            left_eye = get_eye_info(face_landmarks, "left", h, w)

            # --- Added: Eye Closure Detection ---
            # Compute the vertical eye opening (distance between top and bottom landmarks)
            # and the horizontal eye width (distance between inner and outer corners)
            right_eye_opening = calculate_distance(right_eye['top'], right_eye['bottom'])
            right_eye_width = calculate_distance(right_eye['inner'], right_eye['outer'])
            right_EAR = right_eye_opening / (right_eye_width + 1e-6)

            left_eye_opening = calculate_distance(left_eye['top'], left_eye['bottom'])
            left_eye_width = calculate_distance(left_eye['inner'], left_eye['outer'])
            left_EAR = left_eye_opening / (left_eye_width + 1e-6)

            avg_EAR = (right_EAR + left_EAR) / 2.0

            # If the average eye opening ratio is below the threshold, consider the eyes closed.
            if avg_EAR < EYE_CLOSED_THRESHOLD:
                current_direction = "Eye Closed"
            else:
                # --- Continue with the rest of the code if the eyes are open ---
                # Select the eye with the more extreme horizontal ratio (further from 0.5).
                if abs(right_eye['hor_ratio'] - 0.5) >= abs(left_eye['hor_ratio'] - 0.5):
                    chosen_ratio = right_eye['hor_ratio']
                else:
                    chosen_ratio = left_eye['hor_ratio']

                # For vertical direction, average the two eyes’ vertical ratios.
                avg_vert = (right_eye['vert_ratio'] + left_eye['vert_ratio']) / 2.0

                if calibration_mode:
                    state = calib_states[calib_index]
                    cv2.putText(frame, f"CALIBRATION: Look {state}", (30, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(frame, f"Samples: {len(calib_data[state])}/{NUM_SAMPLES}",
                                (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    current_direction = f"Calibrating: {state}"
                else:
                    # Use calibrated (or default) thresholds to decide horizontal direction.
                    if chosen_ratio is not None:
                        if chosen_ratio < left_threshold:
                            hor_dir = "Left"
                        elif chosen_ratio > right_threshold:
                            hor_dir = "Right"
                        else:
                            hor_dir = "Center"
                    else:
                        hor_dir = "Center"
                    
                    # Vertical direction (optional).
                    if avg_vert < 0.4:                                                                     ##Ahmed_Elsafy_Elgewily##
                        vert_dir = "Up"
                    elif avg_vert > 0.65:                                                                   ##Ahmed_Elsafy_Elgewily##
                        vert_dir = "Down"
                    else:
                        vert_dir = "Center"
                    
                    # Combine horizontal and vertical directions.
                    
                    if hor_dir == "Center" and vert_dir == "Center":
                        current_direction = "Center"
                    elif vert_dir == "Center":
                        current_direction = f"Looking {hor_dir}"
                    else :
                        current_direction = f"Looking {vert_dir}"

            # Draw landmarks for both eyes.
            for eye in (right_eye, left_eye):
                for pt in [eye['pupil'], eye['inner'], eye['outer'], eye['top'], eye['bottom']]:
                    cv2.circle(frame, pt, 3, (0, 255, 0), -1)
                cv2.line(frame, eye['inner'], eye['outer'], (255, 0, 0), 1)
                cv2.line(frame, eye['top'], eye['bottom'], (255, 0, 0), 1)

        # Temporal smoothing: update only after several consistent frames.
        if current_direction == last_direction:
            consistent_frames += 1
        else:
            consistent_frames = 0
        if consistent_frames > 10:                                                                          ##Ahmed_Elsafy_Elgewily##
            stable_direction = current_direction
        last_direction = current_direction

        processing_time = (time.time() - start_time) * 1000

        cv2.putText(frame, f"Mapped Direction: {Direction_Map_func(stable_direction)}", (30, h - 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Direction: {stable_direction}", (30, h - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Processing: {processing_time:.1f} ms", (30, h - 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Consistent Frames: {consistent_frames}", (w-300, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.imshow("Eye Direction Detection", frame)
        # SEND command to ESP32
        transmitter.send_command(f"{Direction_Map_func(stable_direction)}|{mode}")
        print(Direction_Map_func(stable_direction))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # Toggle calibration mode with 'c'
        if key == ord('c'):
            calibration_mode = not calibration_mode
            if calibration_mode:
                calib_index = 0
                calib_data = {"Center": [], "Left": [], "Right": []}
                print("Calibration mode activated.")
            else:
                print("Calibration mode deactivated.")

        # --- Calibration Mode Controls ---
        if calibration_mode:
            state = calib_states[calib_index]
            # Press SPACE to record a sample.
            if key == ord(' '):
                if chosen_ratio is not None:
                    calib_data[state].append(chosen_ratio)
                    print(f"Recorded {state} sample: {chosen_ratio:.3f} "
                          f"({len(calib_data[state])}/{NUM_SAMPLES})")
            # Press 'n' to move to the next calibration state.
            if key == ord('n'):
                if len(calib_data[state]) >= NUM_SAMPLES:
                    if calib_index < len(calib_states) - 1:
                        calib_index += 1
                        print(f"Moving to calibration step: {calib_states[calib_index]}")
                    else:
                        print("Already at the final calibration step.")
                else:
                    print(f"Not enough samples for {state}. Need {NUM_SAMPLES}.")
            # Press 'f' to finish calibration and compute new thresholds.
            if key == ord('f'):
                if all(len(calib_data[s]) >= NUM_SAMPLES for s in calib_states):
                    center_avg = np.mean(calib_data["Center"])
                    left_avg = np.mean(calib_data["Left"])
                    right_avg = np.mean(calib_data["Right"])
                    left_threshold = (center_avg + left_avg) / 2.0
                    right_threshold = (center_avg + right_avg) / 2.0
                    calibration_mode = False
                    print("Calibration complete!")
                   # print(f"Center: {center_avg:.3f} | Left: {left_avg:.3f} | Right: {right_avg:.3f}")
                   # print(f"New thresholds -> Left: {left_threshold:.3f}, Right: {right_threshold:.3f}")
                else:
                    print("Insufficient calibration data for all states.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_eye_direction()

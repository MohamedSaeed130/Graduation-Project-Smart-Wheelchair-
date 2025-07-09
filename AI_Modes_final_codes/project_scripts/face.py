import cv2  # For video capture and image processing
import mediapipe as mp  # For face landmark detection
import numpy as np  # For numerical operations
import time  # For measuring FPS
from face_lib import get_largest_face
import transmitter

# Data to send
message = None
mode = "FACE"  # Define the mode
try:
    from mediapipe.python.solutions.face_mesh_connections import FACE_CONNECTIONS
except ImportError:
    FACE_CONNECTIONS = None

# PARAMETERS
target_fps = 20      # Define target FPS
threshold_frames = 13  # Number of frames to confirm direction change
pitch_threshold = 10
yaw_threshold = 13

# Initialize the MediaPipe Face Mesh module
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Initialize MediaPipe drawing utilities and specify drawing styles
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

# Open the default webcam for video capture
cap = cv2.VideoCapture(0)

frame_time = 1.0 / target_fps  # Time for each frame in seconds

# Initialize direction variables
last_direction = None  # To store the last printed direction
current_direction = None  # Current detected direction
direction_counter = 0  # Counter to confirm direction change

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
    elif (detected_dir == "Forward") and last_stop_direction and right_left_check:
        return "FORWARD"
    elif detected_dir == "Looking Up":
        right_left_check = True
        last_stop_direction = True
        return "FORWARD"
    elif detected_dir == "Looking Down":
        last_stop_direction = False
        return "STOP"
    else:
        return "STOP"

# Main loop to process each frame
while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    start_time = time.time()

    # Preprocess the image for MediaPipe Face Mesh
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = face_mesh.process(image)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    img_h, img_w, img_c = image.shape
    largest_face = None
    if results.multi_face_landmarks:
        largest_face, face_2d, face_3d = get_largest_face(results.multi_face_landmarks, img_w, img_h)
    
    if largest_face: 
        focal_length = img_w
        cam_matrix = np.array([[focal_length, 0, img_w / 2],
                               [0, focal_length, img_h / 2],
                               [0, 0, 1]])

        dist_matrix = np.zeros((4, 1), dtype=np.float64)

        success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)
        rmat, _ = cv2.Rodrigues(rot_vec)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
        pitch = angles[0] * 360
        yaw = angles[1] * 360
        roll = angles[2] * 360

        # Determine the current direction
        detected_direction = None
        if pitch > pitch_threshold:
            detected_direction = "Looking Up"
        elif pitch < -pitch_threshold:
            detected_direction = "Looking Down"
        elif yaw > yaw_threshold:
            detected_direction = "Looking Right"
        elif yaw < -yaw_threshold:
            detected_direction = "Looking Left"
        else:
            detected_direction = "Forward"

        # Draw landmarks and connections on the largest face
        mp_drawing.draw_landmarks(
            image=image,
            landmark_list=largest_face,
            connections=FACE_CONNECTIONS if FACE_CONNECTIONS else [],
            landmark_drawing_spec=drawing_spec,
            connection_drawing_spec=drawing_spec
        )
    else:
        detected_direction = "Stop"

    # Update the direction after confirming consistency
    if detected_direction == current_direction:
        direction_counter += 1
    else:
        current_direction = detected_direction
        direction_counter = 1

    if direction_counter >= threshold_frames: #and current_direction != last_direction
        mapped_detected_direction = Direction_Map_func(detected_direction)
        print(f"Detected direction: {mapped_detected_direction}")
        message = f"{mapped_detected_direction}|{mode}"  # Combine direction and mode
        direction_counter=0
        try:
            # Send data to ESP32
            transmitter.send_command(message)
            print(f"Data sent: {message}")
        except Exception as e:
            print(f"An error occurred while sending data: {e}")

        last_direction = current_direction  # Update the last direction

    # Display the direction and counter on the frame
    cv2.putText(image, mapped_detected_direction if last_direction else "Calculating...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
    cv2.putText(image, f'Counter: {direction_counter}', (18, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 2)

    # Display the processed frame in a window
    cv2.imshow('Head Pose Estimation', image)

    # Calculate elapsed time and control frame rate
    elapsed_time = time.time() - start_time
    sleep_time = max(0, frame_time - elapsed_time)  # Ensure non-negative sleep time
    time.sleep(sleep_time)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
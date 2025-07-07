import numpy as np

def get_largest_face(multi_face_landmarks, img_w, img_h):
    """
    Extract the largest face based on the bounding box area(for mediapipe).

    Parameters:
        results (mediapipe.python.solution.face_mesh.FaceMesh.process): The result of the face mesh detection.
        img_w (int): Width of the input image.
        img_h (int): Height of the input image.

    Returns:
        tuple: A tuple containing face_2d points, and face_3d points.
    """
    largest_face = None
    largest_area = 0
    face_2d = []
    face_3d = []

    # Return if there is not any face
    if not multi_face_landmarks: return

    for face_landmarks in multi_face_landmarks:
        # Calculate the bounding box for each face
        x_coords = [int(lm.x * img_w) for lm in face_landmarks.landmark]
        y_coords = [int(lm.y * img_h) for lm in face_landmarks.landmark]
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        area = (x_max - x_min) * (y_max - y_min)

        # Keep the face with the largest bounding box
        if area > largest_area:
            largest_area = area
            largest_face = face_landmarks

    if largest_face:
        for idx, lm in enumerate(largest_face.landmark):
            if idx in [33, 263, 1, 61, 291, 199]:  # Key landmarks
                x, y = int(lm.x * img_w), int(lm.y * img_h)
                face_2d.append([x, y])
                face_3d.append([x, y, lm.z])

    return largest_face, np.array(face_2d, dtype=np.float64), np.array(face_3d, dtype=np.float64)

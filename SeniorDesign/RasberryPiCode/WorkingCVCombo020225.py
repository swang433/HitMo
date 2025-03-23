import cv2
import mediapipe as mp
from collections import deque

mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

WINDOW_SIZE = 10            # Number of frames to track movement
JAB_THRESHOLD = 0.15        # Z-axis change threshold for a jab
CROSS_THRESHOLD = 0.15      # Z-axis change threshold for a cross
HOOK_X_THRESHOLD = 0.10     # X-axis change threshold for a lead hook
HOOK_Z_FORWARD_THRESHOLD = 0.05  # Minimal forward movement for hook
READY_POSITION_THRESHOLD = 0.1   # Threshold to consider hand near torso
COMBO_RESET_DELAY = 2            # Reset combo and stance after 2 seconds of inactivity
FPS = 30                         

landmark_history = deque(maxlen=WINDOW_SIZE)
state = "Ready"  # State machine: "Ready", "Strike", "Return"
combo = []       # Track the punch sequence
pause_counter = 0  # Count frames with no significant movement
stance = None      # Fighter's stance, detected at the beginning

def detect_stance(landmarks):
    """
    Detect the fighter's stance based on shoulder depth (z-coordinates).
    If left_shoulder.z < right_shoulder.z, we assume the left shoulder is closer.
    """
    left_shoulder = landmarks[11]
    right_shoulder = landmarks[12]
    if left_shoulder.z < right_shoulder.z:
        return "Orthodox"
    elif right_shoulder.z < left_shoulder.z:
        return "Southpaw"
    return "Unknown"

def in_ready_position(wrist_z, shoulder_z):
    """
    Checks if the wrist is close enough to the torso (shoulder) to be considered "ready".
    """
    return abs(wrist_z - shoulder_z) < READY_POSITION_THRESHOLD

def detect_jab(landmarks, stance):
    """
    Detect a jab motion for the LEAD arm:
      - Orthodox: left arm (wrist=15, elbow=13, shoulder=11)
      - Southpaw: right arm (wrist=16, elbow=14, shoulder=12)
    Returns "Jab" if detected, else None.
    """
    if stance == "Orthodox":
        wrist = landmarks[15]
        elbow = landmarks[13]
        shoulder = landmarks[11]
    else:  # Southpaw
        wrist = landmarks[16]
        elbow = landmarks[14]
        shoulder = landmarks[12]

    wrist_z = wrist.z
    elbow_z = elbow.z
    shoulder_z = shoulder.z

    global state
    if state == "Ready":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Strike"
    elif state == "Strike":
        if (elbow_z - wrist_z) > JAB_THRESHOLD:
            state = "Return"
            return "Jab"
    elif state == "Return":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Ready"
    return None

def detect_cross(landmarks, stance):
    """
    Detect a cross motion for the REAR arm:
      - Orthodox: right arm (wrist=16, elbow=14, shoulder=12)
      - Southpaw: left arm (wrist=15, elbow=13, shoulder=11)
    Returns "Cross" if detected, else None.
    """
    if stance == "Orthodox":
        wrist = landmarks[16]
        elbow = landmarks[14]
        shoulder = landmarks[12]
    else:  # Southpaw
        wrist = landmarks[15]
        elbow = landmarks[13]
        shoulder = landmarks[11]

    wrist_z = wrist.z
    elbow_z = elbow.z
    shoulder_z = shoulder.z

    global state
    if state == "Ready":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Strike"
    elif state == "Strike":
        if (elbow_z - wrist_z) > CROSS_THRESHOLD:
            state = "Return"
            return "Cross"
    elif state == "Return":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Ready"
    return None

def detect_lead_hook(landmarks, stance):
    """
    Detect a LEAD hook:
      - Orthodox: left arm (wrist=15, elbow=13, shoulder=11)
      - Southpaw: right arm (wrist=16, elbow=14, shoulder=12)
    """
    if stance == "Orthodox":
        wrist = landmarks[15]
        elbow = landmarks[13]
        shoulder = landmarks[11]
    else:  # Southpaw
        wrist = landmarks[16]
        elbow = landmarks[14]
        shoulder = landmarks[12]

    wrist_x = wrist.x
    elbow_x = elbow.x
    shoulder_x = shoulder.x
    wrist_z = wrist.z
    shoulder_z = shoulder.z

    global state
    if state == "Ready":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Strike"
    elif state == "Strike":
        x_diff = abs(elbow_x - wrist_x)
        z_diff = (shoulder_z - wrist_z)
        if x_diff > HOOK_X_THRESHOLD and z_diff > HOOK_Z_FORWARD_THRESHOLD:
            state = "Return"
            return "Hook"
    elif state == "Return":
        if in_ready_position(wrist_z, shoulder_z):
            state = "Ready"
    return None

def detect_punch(landmark_history, stance):
    """
    Check for jab, cross, or hook in the latest frame.
    """
    if len(landmark_history) < 2:
        return None
    landmarks = landmark_history[-1]
    jab = detect_jab(landmarks, stance)
    if jab:
        return jab
    cross = detect_cross(landmarks, stance)
    if cross:
        return cross
    hook = detect_lead_hook(landmarks, stance)
    if hook:
        return hook
    return None

cap = cv2.VideoCapture('video.mp4')
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS)) or FPS

out = cv2.VideoWriter(
    'processedVideo.mp4',
    cv2.VideoWriter_fourcc(*'mp4v'),
    fps,
    (frame_width, frame_height)
)

frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("End of video or cannot read frame.")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)
    annotated_frame = frame.copy()

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        landmark_history.append(landmarks)

        if stance is None:
            stance = detect_stance(landmarks)
            if stance != "Unknown":
                print(f"Stance detected: {stance}")
            pause_counter = 0
            combo = []

        if stance != "Unknown":
            punch_type = detect_punch(landmark_history, stance)
            if punch_type:
                combo.append(punch_type)
                print(f"Frame {frame_count}: {punch_type} detected! Current combo: {combo}")
                pause_counter = 0
            else:
                pause_counter += 1

            if pause_counter >= COMBO_RESET_DELAY * fps:
                combo = []
                stance = None
                print("Combo and stance reset due to inactivity.")
                pause_counter = 0

        mp.solutions.drawing_utils.draw_landmarks(
            annotated_frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
        )

        cv2.putText(
            annotated_frame,
            f"Stance: {stance or 'Undetermined'}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )
        cv2.putText(
            annotated_frame,
            f"Combo: {' -> '.join(combo) if combo else 'None'}",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 0, 0),
            2
        )
    else:
        cv2.putText(
            annotated_frame,
            "No landmarks detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    out.write(annotated_frame)
    frame_count += 1

cap.release()
out.release()
cv2.destroyAllWindows()

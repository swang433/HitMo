import cv2
import mediapipe as mp

# Initialize Mediapipe Pose for both views
mp_pose = mp.solutions.pose
pose_top = mp_pose.Pose()
pose_side = mp_pose.Pose()

# Open the two prerecorded videos
cap_side = cv2.VideoCapture("top_video.mp4")   # Main camera (top view)
cap_top = cv2.VideoCapture("bottom_video.mp4")   # Side camera

# Get properties from the main video (assumed to match side video)
frame_width  = int(cap_top.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap_top.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap_top.get(cv2.CAP_PROP_FPS)) or 30

# Define video writer for annotated output
out = cv2.VideoWriter("adjusted_output.mp4",
                      cv2.VideoWriter_fourcc(*"mp4v"),
                      fps, (frame_width, frame_height))

# Scaling factor to adjust Z using the X coordinate from the side view
Z_SCALE_FACTOR = 0.2  # Adjust this based on your setup

while cap_top.isOpened() and cap_side.isOpened():
    ret_top, frame_top = cap_top.read()
    ret_side, frame_side = cap_side.read()
    
    if not ret_top or not ret_side:
        print("End of one of the videos or frame read error.")
        break

    # Process the main (top) frame for pose landmarks
    frame_top_rgb = cv2.cvtColor(frame_top, cv2.COLOR_BGR2RGB)
    results_top = pose_top.process(frame_top_rgb)
    
    # Process the side frame for pose landmarks
    frame_side_rgb = cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB)
    results_side = pose_side.process(frame_side_rgb)
    
    # Make a copy for annotation
    annotated_frame = frame_top.copy()
    
    if results_top.pose_landmarks and results_side.pose_landmarks:
        # Get landmarks from both views
        landmarks_top = results_top.pose_landmarks.landmark
        landmarks_side = results_side.pose_landmarks.landmark

        # Key landmark indices
        landmark_indices = {
            "L_Shoulder": 11,
            "R_Shoulder": 12,
            "L_Elbow": 13,
            "R_Elbow": 14,
            "L_Wrist": 15,
            "R_Wrist": 16
        }

        # Store text for overlay
        text_lines = []

        for name, idx in landmark_indices.items():
            lm_top = landmarks_top[idx]
            lm_side = landmarks_side[idx]

            # Convert normalized coordinates to pixel values
            x_main, y_main = int(lm_top.x * frame_width), int(lm_top.y * frame_height)

            # Adjust Z using X from side view
            corrected_z = lm_top.z - (lm_side.x * Z_SCALE_FACTOR)

            # Store coordinate text
            text_lines.append(f"{name}: X={x_main}, Y={y_main}, Z={corrected_z:.2f}")

        # Overlay text in top-left corner
        y_offset = 30
        for text in text_lines:
            cv2.putText(annotated_frame, text, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 20  # Move down for next line

    # Show the adjusted video with text overlay
    cv2.imshow("Adjusted Pose", annotated_frame)
    out.write(annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources
cap_top.release()
cap_side.release()
out.release()
cv2.destroyAllWindows()

print("Annotated video saved as 'adjusted_output.mp4'")

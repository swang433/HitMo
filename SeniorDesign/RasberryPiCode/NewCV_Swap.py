
import mediapipe as mp
from collections import deque
import csv
import pandas as pd
import numpy as np
import os
import argparse
import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk
import cv2

# --------------------------------------------
# Parameters, Initialization, and CSV Setup
# --------------------------------------------
mp_pose = mp.solutions.pose
pose_A = mp_pose.Pose()  # For main (bottom) video
pose_B = mp_pose.Pose()  # For secondary (top) video

# Detection thresholds and parameters
WINDOW_SIZE = 10            # Number of frames to track movement
JAB_THRESHOLD = 0.15        # Threshold for jab detection (difference in replaced z)
CROSS_THRESHOLD = 0.15      # Threshold for cross detection
HOOK_X_THRESHOLD = 0.10     # X-axis change threshold for hook
HOOK_Z_FORWARD_THRESHOLD = 0.05  # Minimal forward movement for hook
UPPERCUT_Z_THRESHOLD = 50   # Forward movement for uppercut
UPPERCUT_Y_THRESHOLD = 0.10  # Upward movement for uppercut
COMBO_RESET_DELAY = 2        # Seconds of inactivity to reset combo
FPS_DEFAULT = 30            
STANCE_DETECTION_DURATION = 2  # Determine stance only in the first 2 seconds

# For storing landmark history and punch sequence
landmark_history = deque(maxlen=WINDOW_SIZE)
state = "Ready"
combo = []       
pause_counter = 0
stance = None    

# Define frame_count (must be defined before main loop)
frame_count = 0

# Define output directory
OUTPUT_DIR = "output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Setup argument parser
parser = argparse.ArgumentParser(description='Process video files for boxing analysis.')
parser.add_argument('--bottom_video', type=str, default=os.path.join(OUTPUT_DIR, r"bottom_video.mp4"),
                    help='Path to bottom camera video file')
parser.add_argument('--top_video', type=str, default=os.path.join(OUTPUT_DIR, r"top_video.mp4"),
                    help='Path to top camera video file')
args = parser.parse_args()

# Open CSV file for logging (one row per frame)
csv_file = open(os.path.join(OUTPUT_DIR, "camera.csv"), mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "Timestamp", "Stance", "Punch",
    "L_Wrist_X", "L_Wrist_Y", "L_Wrist_Z",
    "L_Elbow_X", "L_Elbow_Y", "L_Elbow_Z",
    "L_Shoulder_X", "L_Shoulder_Y", "L_Shoulder_Z",
    "R_Wrist_X", "R_Wrist_Y", "R_Wrist_Z",
    "R_Elbow_X", "R_Elbow_Y", "R_Elbow_Z",
    "R_Shoulder_X", "R_Shoulder_Y", "R_Shoulder_Z"
])

# --------------------------------------------
# Helper Functions for Detection
# --------------------------------------------
def detect_stance(landmarks):
    """Detect fighter's stance using shoulder landmarks (indices 11 and 12)."""
    left_shoulder = landmarks[11]
    right_shoulder = landmarks[12]
    if left_shoulder.z > right_shoulder.z:
        return "Orthodox"
    elif right_shoulder.z > left_shoulder.z:
        return "Southpaw"
    return "Unknown"

def in_ready_position(wrist_y, elbow_y):
    """Check if an arm is in the ready position using y-coordinates."""
    global frame_height
    return abs(wrist_y * frame_height - elbow_y * frame_height) < 100

# --------------------------------------------
# Punch Detection Functions (Dynamic)
# --------------------------------------------
def detect_jab_dynamic(landmark_history, stance):
    """Detect jab dynamically over 2-5 frame gaps."""
    if len(landmark_history) < 2:
        return ""
    wrist_idx = 15 if stance == "Orthodox" else 16
    for gap in range(2, min(6, len(landmark_history)+1)):
        diff = landmark_history[-1][wrist_idx].z - landmark_history[-gap][wrist_idx].z
        if diff > JAB_THRESHOLD:
            return "Jab"
    return ""

def detect_cross_dynamic(landmark_history, stance):
    """Detect cross dynamically over 2-5 frame gaps."""
    if len(landmark_history) < 2:
        return ""
    wrist_idx = 16 if stance == "Orthodox" else 15
    for gap in range(2, min(6, len(landmark_history)+1)):
        diff = landmark_history[-1][wrist_idx].z - landmark_history[-gap][wrist_idx].z
        if diff > CROSS_THRESHOLD:
            return "Cross"
    return ""

def detect_hook_dynamic(landmark_history, stance, min_x_change=150, min_frame_gap=3):
    """
    Detects a lead or rear hook using ONLY X-movement with reduced sensitivity.
    """
    if len(landmark_history) < min_frame_gap:
        return ""

    # Indices for lead and rear hands
    if stance == "Orthodox":
        lead_wrist_idx, lead_elbow_idx = 15, 13  # Left hand
        rear_wrist_idx, rear_elbow_idx = 16, 14  # Right hand
    else:
        lead_wrist_idx, lead_elbow_idx = 16, 14  # Right hand
        rear_wrist_idx, rear_elbow_idx = 15, 13  # Left hand

    for gap in range(min_frame_gap, min(8, len(landmark_history)+1)):  # Check over 3-7 frames
        earliest = landmark_history[-gap]
        middle = landmark_history[-(gap//2)]
        latest = landmark_history[-1]

        global frame_width

        # Convert X coordinates to pixel values
        past_lead_x = earliest[lead_wrist_idx].x * frame_width
        mid_lead_x = middle[lead_wrist_idx].x * frame_width
        current_lead_x = latest[lead_wrist_idx].x * frame_width

        past_rear_x = earliest[rear_wrist_idx].x * frame_width
        mid_rear_x = middle[rear_wrist_idx].x * frame_width
        current_rear_x = latest[rear_wrist_idx].x * frame_width

        # Calculate total X movement
        lead_x_change = abs(current_lead_x - past_lead_x)
        rear_x_change = abs(current_rear_x - past_rear_x)

        # Hook motion: Outward then inward (min X movement required)
        lead_x_sweep = (mid_lead_x > past_lead_x and current_lead_x < mid_lead_x and lead_x_change > min_x_change)
        rear_x_sweep = (mid_rear_x < past_rear_x and current_rear_x > mid_rear_x and rear_x_change > min_x_change)

        # Detect Lead Hook (Left hand sweeping inward)
        if lead_x_sweep:
            return "Lead Hook"

        # Detect Rear Hook (Right hand sweeping inward)
        if rear_x_sweep:
            return "Rear Hook"

    return ""

def detect_uppercut_dynamic(landmark_history, stance, min_y_change=100, max_x_change=50, min_frame_gap=3):
    """
    Detects a lead or rear uppercut using ONLY Y-movement, while ignoring movements with excessive X change.
    """
    if len(landmark_history) < min_frame_gap:
        return ""

    # Indices for lead and rear hands
    if stance == "Orthodox":
        lead_wrist_idx, lead_elbow_idx = 15, 13  # Left hand
        rear_wrist_idx, rear_elbow_idx = 16, 14  # Right hand
    else:
        lead_wrist_idx, lead_elbow_idx = 16, 14  # Right hand
        rear_wrist_idx, rear_elbow_idx = 15, 13  # Left hand

    for gap in range(min_frame_gap, min(8, len(landmark_history)+1)):  # Check over 3-7 frames
        earliest = landmark_history[-gap]
        middle = landmark_history[-(gap//2)]
        latest = landmark_history[-1]

        global frame_width, frame_height

        # Convert Y & X coordinates to pixel values
        past_lead_y = earliest[lead_wrist_idx].y * frame_height
        mid_lead_y = middle[lead_wrist_idx].y * frame_height
        current_lead_y = latest[lead_wrist_idx].y * frame_height

        past_rear_y = earliest[rear_wrist_idx].y * frame_height
        mid_rear_y = middle[rear_wrist_idx].y * frame_height
        current_rear_y = latest[rear_wrist_idx].y * frame_height

        past_lead_x = earliest[lead_wrist_idx].x * frame_width
        current_lead_x = latest[lead_wrist_idx].x * frame_width

        past_rear_x = earliest[rear_wrist_idx].x * frame_width
        current_rear_x = latest[rear_wrist_idx].x * frame_width

        # Calculate total Y movement
        lead_y_change = abs(current_lead_y - past_lead_y)
        rear_y_change = abs(current_rear_y - past_rear_y)

        # Calculate total X movement
        lead_x_change = abs(current_lead_x - past_lead_x)
        rear_x_change = abs(current_rear_x - past_rear_x)

        # Uppercut motion: Moving up significantly, but NOT too much sideways motion
        lead_y_sweep = (current_lead_y < mid_lead_y and mid_lead_y < past_lead_y and 
                        lead_y_change > min_y_change and lead_x_change < max_x_change)

        rear_y_sweep = (current_rear_y < mid_rear_y and mid_rear_y < past_rear_y and 
                        rear_y_change > min_y_change and rear_x_change < max_x_change)

        # Detect Lead Uppercut (Left hand moving upwards, but not sideways)
        if lead_y_sweep:
            return "Lead Uppercut"

        # Detect Rear Uppercut (Right hand moving upwards, but not sideways)
        if rear_y_sweep:
            return "Rear Uppercut"

    return ""

def detect_punch(landmark_history, stance):
    """
    Detect multiple punches in the same frame.
    Returns a list of detected punches instead of stopping at the first one.
    """
    detected_punches = []  # Store all detected punches in this list

    jab = detect_jab_dynamic(landmark_history, stance)
    if jab:
        detected_punches.append(jab)

    cross = detect_cross_dynamic(landmark_history, stance)
    if cross:
        detected_punches.append(cross)

    hook = detect_hook_dynamic(landmark_history, stance)
    if hook:
        detected_punches.append(hook)

    uppercut = detect_uppercut_dynamic(landmark_history, stance)
    if uppercut:
        detected_punches.append(uppercut)

    return detected_punches if detected_punches else ["None"]  # Return all detected punches or "None"

def evaluate_boxing_routine(arm_csv='arm.csv', camera_csv='camera.csv', routine_csv='Routine.csv', time_threshold=0.5, routine_number=None):
    """
    Evaluate a boxing routine by comparing detected punches with expected punches.
    
    Args:
        arm_csv: Path to CSV file with physical sensor data
        camera_csv: Path to CSV file with camera detection data
        routine_csv: Path to CSV file with expected punch routine
        time_threshold: Maximum allowed time difference between arm and camera events
        routine_number: Optional routine number to use (otherwise extracted from arm data)
        
    Returns:
        Dictionary with evaluation results
    """
    # Load CSV files
    arm_df = pd.read_csv(arm_csv)
    camera_df = pd.read_csv(camera_csv)
    routine_df = pd.read_csv(routine_csv)
    
    # Convert the Punch column to string to ensure it can be split
    camera_df['Punch'] = camera_df['Punch'].astype(str)
    
    # Extract routine number from arm data if not provided
    if routine_number is None and 'Routine' in arm_df.columns:
        # Get the most common routine number (in case there are any inconsistencies)
        routine_number = arm_df['Routine'].mode()[0]
        print(f"Detected routine number {routine_number} from arm data")
    
    # Filter to just the TRUE events from arm data
    # This handles both the original format and new format where STRIKE values are converted to Bool
    true_events = arm_df[arm_df['Bool'] == True].reset_index(drop=True)
    
    # Initialize tracking variables
    results = []
    correct_count = 0
    routine_index = 0  # Track current position in routine
    
    # Process each TRUE event
    for _, event in true_events.iterrows():
        # Get the expected punch from routine (if available)
        if routine_index >= len(routine_df):
            expected_punch = "No more expected punches"
        else:
            expected_punch = routine_df.iloc[routine_index]['Punch']
            
        # Find closest camera timestamp to arm event
        arm_time = event['Timestamp']
        camera_idx = (camera_df['Timestamp'] - arm_time).abs().idxmin()
        camera_row = camera_df.loc[camera_idx]
        camera_time = camera_row['Timestamp']
        detected_punch = camera_row['Punch']
        time_diff = abs(camera_time - arm_time)
        
        # Only count as valid if timestamps are within threshold
        if time_diff <= time_threshold:
            # Check if detected punch includes expected punch
            detected_punch_list = detected_punch.split(' + ')
            is_match = (routine_index < len(routine_df)) and (expected_punch in detected_punch_list) and ("None" not in detected_punch_list)
            
            if is_match:
                correct_count += 1
        else:
            is_match = False
            detected_punch = f"{detected_punch} (Time diff too large: {time_diff:.2f}s)"
                
        # Store details for this punch
        results.append({
            'Event': routine_index + 1,
            'Arm Time': arm_time,
            'Camera Time': camera_time,
            'Time Diff': time_diff,
            'Expected': expected_punch,
            'Detected': detected_punch,
            'Correct': is_match
        })
        
        # Move to next expected punch after each TRUE event
        routine_index += 1
    
    # Check if routine had more punches than detected events
    remaining_punches = len(routine_df) - routine_index
    if remaining_punches > 0:
        for i in range(remaining_punches):
            idx = routine_index + i
            results.append({
                'Event': idx + 1,
                'Arm Time': None,
                'Camera Time': None,
                'Time Diff': None,
                'Expected': routine_df.iloc[idx]['Punch'],
                'Detected': "No sensor event detected",
                'Correct': False
            })
    
    # Calculate overall accuracy
    total_events = min(len(true_events), len(routine_df))
    accuracy = (correct_count / total_events) * 100 if total_events > 0 else 0
    
    # Calculate completion time (time from first to last TRUE event)
    if len(true_events) >= 2:
        start_time = true_events.iloc[0]['Timestamp']
        end_time = true_events.iloc[-1]['Timestamp']
        completion_time = end_time - start_time
    else:
        completion_time = None
    
    # Define expected completion times for each routine (in seconds)
    expected_times = {
        1: 15.0,  # Routine 1: 15 seconds
        2: 22.0,  # Routine 2: 22 seconds
        3: 30.0,  # Routine 3: 30 seconds
    }
    
    # Calculate time score
    time_score = 0
    if completion_time is not None and routine_number in expected_times:
        expected_time = expected_times[routine_number]
        # Calculate time score (100% if exactly on time, bonus for faster, penalty for slower)
        time_ratio = expected_time / completion_time
        # Cap time score between 0% and 120%
        time_score = min(120, max(0, 100 * time_ratio))
    
    # Calculate total score (50% accuracy, 50% time)
    total_score = (accuracy * 0.5) + (time_score * 0.5)
    
    return {
        'accuracy': accuracy,
        'accuracy_score': accuracy,  # Same as accuracy but explicitly labeled as score
        'correct': correct_count,
        'total': total_events,
        'completion_time': completion_time,
        'expected_time': expected_times.get(routine_number),
        'time_score': time_score,
        'total_score': total_score,
        'routine_number': routine_number,
        'details': results
    }

class VideoPlayer:
        def __init__(self, parent, video_path, width=640, height=480):
            self.parent = parent
            self.video_path = video_path
            self.cap = cv2.VideoCapture(video_path)

            # Get actual video width and height
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if width is None else width
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if height is None else height

            # Create a Label in the existing window to display the video
            self.video_label = Label(parent)
            self.video_label.pack(expand=True, fill="both")  # Allow expansion

            # Start updating the video
            self.update_video()

        def update_video(self):
            ret, frame = self.cap.read()
            if ret:
                # Resize frame to match Tkinter window
                frame = cv2.resize(frame, (self.width, self.height))

                # Convert frame from BGR to RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = Image.fromarray(frame)
                frame = ImageTk.PhotoImage(frame)

                # Update the label with the new frame
                self.video_label.config(image=frame)
                self.video_label.image = frame

                # Repeat every 20 ms
                self.parent.after(20, self.update_video)
            else:
                self.cap.release()  # Release video when finished

def display_summary():
    print("summary running")
    df = pd.read_csv("evaluation_summary.csv") #PLACEHOLDER PATH
    root = tk.Tk()
    root.title("Evaluation Summary")

    # Create a frame for the video and text
    video_frame = tk.Frame(root)
    video_frame.pack(side="left", expand=True, fill="both")  # Expand video frame fully
    text_frame = tk.Frame(root, width=250)  # Adjust width for better text display
    text_frame.pack(side="right", fill="y")

    # Create a label for the video
    instruction_label = Label(video_frame, text="Video Playback:")
    instruction_label.pack()

    # Integrate VideoPlayer into the existing window (Ensure width and height match)
    video_path = r"output/bottom_video.mp4"  # **Adjust path here**
    player = VideoPlayer(video_frame, video_path, width=800, height=600)  # **Adjust size here**

    # Create a text widget for the summaryf
    text_widget = tk.Text(text_frame, wrap="word", height=15)
    text_widget.insert("1.0", df.to_string())
    text_widget.pack(expand=True, fill="both")

    root.mainloop()

# Main function to separate concerns
def main(bottom_video=None, top_video=None):
    global frame_count, frame_width, frame_height, fps, landmark_history, stance, combo, pause_counter
    
    # Use function parameters if provided, otherwise use command line args
    bottom_video_path = bottom_video if bottom_video else args.bottom_video
    top_video_path = top_video if top_video else args.top_video
    
    print(f"Processing videos:")
    print(f"  - Bottom video: {bottom_video_path}")
    print(f"  - Top video: {top_video_path}")
    
    # Reset global variables
    frame_count = 0
    landmark_history = deque(maxlen=WINDOW_SIZE)
    stance = None
    combo = []
    pause_counter = 0
    
    # Open input videos
    cap_A = cv2.VideoCapture(bottom_video_path)  # Main (bottom) video
    cap_B = cv2.VideoCapture(top_video_path)     # Secondary (top) video
    
    if not cap_A.isOpened():
        print(f"Error: Could not open bottom video at {bottom_video_path}")
        return 0
        
    if not cap_B.isOpened():
        print(f"Error: Could not open top video at {top_video_path}")
        return 0
    

    
    

    
    # Get video properties
    frame_width = int(cap_A.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap_A.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap_A.get(cv2.CAP_PROP_FPS)) or FPS_DEFAULT
    
    # Set up video writer for output
    processed_video_path = os.path.join(OUTPUT_DIR, "processedVideo.mp4")
    out = cv2.VideoWriter(processed_video_path,
                          cv2.VideoWriter_fourcc(*'mp4v'),
                          fps,
                          (frame_width, frame_height))
    
    print(f"Started processing... Output will be saved to {processed_video_path}")
    
    # Main processing loop
    while cap_A.isOpened() and cap_B.isOpened():
        ret_A, frame_A = cap_A.read()
        ret_B, frame_B = cap_B.read()
        if not ret_A or not ret_B:
            print("End of video or frame read error.")
            break
    
        # Convert frames from BGR to RGB for pose processing
        frame_A_rgb = cv2.cvtColor(frame_A, cv2.COLOR_BGR2RGB)
        frame_B_rgb = cv2.cvtColor(frame_B, cv2.COLOR_BGR2RGB)
        results_A = pose_A.process(frame_A_rgb)
        results_B = pose_B.process(frame_B_rgb)
    
        annotated_frame = frame_A.copy()
    
        if results_A.pose_landmarks and results_B.pose_landmarks:
            landmarks_A = results_A.pose_landmarks.landmark
            landmarks_B = results_B.pose_landmarks.landmark
    
            # Replace each landmark's z in the main video with the x value from the secondary video
            for i in range(len(landmarks_A)):
                landmarks_A[i].z = landmarks_B[i].x
    
            # Save the modified landmarks in our history for punch detection
            landmark_history.append(landmarks_A)
    
            # Determine stance only during the first 2 seconds of the video
            if stance is None and frame_count < STANCE_DETECTION_DURATION * fps:
                stance = detect_stance(landmarks_A)
                if stance != "Unknown":
                    print(f"Stance detected: {stance}")
                pause_counter = 0
                combo = []
    
            # Detect multiple punches per frame
            punches = detect_punch(landmark_history, stance)
    
            if punches != ["None"]:  # If any punches detected
                combo.extend(punches)  # Append multiple detected punches
                punch_text = " + ".join(punches)  # Join punches for display
                print(f"Frame {frame_count}: {punch_text} detected! Current combo: {combo}")
                pause_counter = 0
            else:
                punch_text = "None"
                pause_counter += 1
    
            # Reset combo after inactivity
            if pause_counter >= COMBO_RESET_DELAY * fps:
                combo = []
                print("Combo reset due to inactivity.")
                pause_counter = 0
    
            # -------------------------------
            # Annotate the Processed Frame
            # -------------------------------
            mp.solutions.drawing_utils.draw_landmarks(
                annotated_frame, results_A.pose_landmarks, mp_pose.POSE_CONNECTIONS
            )
    
            # Display Stance, Combo, and Punches
            cv2.putText(annotated_frame, f"Stance: {stance or 'Undetermined'}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Combo: {' -> '.join(combo) if combo else 'None'}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(annotated_frame, f"Punch: {punch_text}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
            # Extract Key Landmarks for CSV Logging
            l_wrist, l_elbow, l_shoulder = landmarks_A[15], landmarks_A[13], landmarks_A[11]
            r_wrist, r_elbow, r_shoulder = landmarks_A[16], landmarks_A[14], landmarks_A[12]
    
            # Log CSV row with multiple punches
            timestamp = frame_count / fps
            csv_writer.writerow([
                timestamp, stance, punch_text,
                f"{l_wrist.x:.4f}", f"{l_wrist.y:.4f}", f"{l_wrist.z:.4f}",
                f"{l_elbow.x:.4f}", f"{l_elbow.y:.4f}", f"{l_elbow.z:.4f}",
                f"{l_shoulder.x:.4f}", f"{l_shoulder.y:.4f}", f"{l_shoulder.z:.4f}",
                f"{r_wrist.x:.4f}", f"{r_wrist.y:.4f}", f"{r_wrist.z:.4f}",
                f"{r_elbow.x:.4f}", f"{r_elbow.y:.4f}", f"{r_elbow.z:.4f}",
                f"{r_shoulder.x:.4f}", f"{r_shoulder.y:.4f}", f"{r_shoulder.z:.4f}"
            ])
    
        else:
            cv2.putText(annotated_frame, "No landmarks detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
        out.write(annotated_frame)
        frame_count += 1
    
        cv2.imshow("Processed Pose", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    # Clean up Resources
    cap_A.release()
    cap_B.release()
    out.release()
    csv_file.close()
    cv2.destroyAllWindows()
    
    print("Processing complete. Processed video saved as 'processedVideo.mp4'")
    print("CSV log saved as 'camera.csv'")
    
    # Evaluate Boxing Routine
    print("\nEvaluating boxing routine...")
    arm_csv_path = os.path.join(OUTPUT_DIR, r"arm.csv")
    camera_csv_path = os.path.join(OUTPUT_DIR, r"camera.csv")
    routine_csv_path = os.path.join(OUTPUT_DIR, r"Routine.csv")
    
    evaluation = evaluate_boxing_routine(arm_csv_path, camera_csv_path, routine_csv_path)
    
    routine_number = evaluation.get('routine_number', 'Unknown')
    
    print(f"\n======= BOXING ROUTINE {routine_number} EVALUATION =======")
    print(f"Accuracy Score: {evaluation['accuracy_score']:.2f}% ({evaluation['correct']}/{evaluation['total']} correct punches)")
    
    if evaluation['completion_time'] is not None:
        print(f"Completion Time: {evaluation['completion_time']:.2f} seconds")
        print(f"Expected Time: {evaluation['expected_time']:.2f} seconds")
        print(f"Time Score: {evaluation['time_score']:.2f}%")
    
    print(f"\n🏆 TOTAL SCORE: {evaluation['total_score']:.2f}% 🏆")
    print("=============================================")
    
    
    # Generate a performance grade
    def get_grade(score):
        if score >= 95: return "S (Perfect)"
        if score >= 90: return "A (Excellent)"
        if score >= 80: return "B (Great)"
        if score >= 70: return "C (Good)"
        if score >= 60: return "D (Fair)"
        return "F (Needs Improvement)"
    
    print(f"Performance Grade: {get_grade(evaluation['total_score'])}")
    
    print("\nDetailed Results:")
    for result in evaluation['details']:
        print(f"Event {result['Event']}:")
        print(f"  Expected: {result['Expected']}")
        print(f"  Detected: {result['Detected']}")
        print(f"  Correct: {'✓' if result['Correct'] else '✗'}")
        if result['Time Diff'] is not None:
            print(f"  Time Difference: {result['Time Diff']:.3f}s")
        else:
            print("  Time Difference: N/A")
        print()
    
    # Create and save a comprehensive results summary CSV
    results_df = pd.DataFrame(evaluation['details'])
    
    # Add summary information
    summary_data = {
        'Metric': ['Accuracy Score', 'Time Score', 'Total Score', 'Grade', 'Correct Punches', 'Total Punches', 
                  'Completion Time', 'Expected Time', 'Routine Number'],
        'Value': [f"{evaluation['accuracy_score']:.2f}%", f"{evaluation['time_score']:.2f}%", 
                 f"{evaluation['total_score']:.2f}%", get_grade(evaluation['total_score']),
                 evaluation['correct'], evaluation['total'], 
                 f"{evaluation['completion_time']:.2f}s" if evaluation['completion_time'] is not None else "N/A",
                 f"{evaluation['expected_time']:.2f}s", routine_number]
    }
    summary_df = pd.DataFrame(summary_data)
    
    # Save both results
    details_csv_path = os.path.join(OUTPUT_DIR, "evaluation_details.csv")
    summary_csv_path = os.path.join(OUTPUT_DIR, "evaluation_summary.csv")
    
    results_df.to_csv(details_csv_path, index=False)
    summary_df.to_csv(summary_csv_path, index=False)
    
    print(f"Detailed evaluation results saved to '{details_csv_path}'")
    print(f"Summary results saved to '{summary_csv_path}'")
    return evaluation['total_score']

# Function to be called from other scripts
def process_and_evaluate(bottom_video=None, top_video=None, output_dir=OUTPUT_DIR):
    """
    Process boxing videos and evaluate performance.
    Can be called directly from other scripts like main.py.
    
    Args:
        bottom_video (str): Path to bottom camera video
        top_video (str): Path to top camera video
        output_dir (str): Directory for output files
        
    Returns:
        float: The total score from the evaluation
    """
    global OUTPUT_DIR
    
    # Update output directory if provided
    if output_dir != OUTPUT_DIR:
        OUTPUT_DIR = output_dir
        os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Use default paths if none provided
    if not bottom_video:
        bottom_video = os.path.join(OUTPUT_DIR, "bottom_video.mp4")
    if not top_video:
        top_video = os.path.join(OUTPUT_DIR, "top_video.mp4")
    
    # Run the main processing
    return main(bottom_video, top_video)

if __name__ == "__main__":
    main()
    display_summary()

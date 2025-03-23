import time
import cv2
import mediapipe as mp
import serial
import csv
import datetime
import os
import subprocess
import sys
import importlib.util
from collections import deque
import argparse
import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk

# Configuration constants
SERIAL_PORT = "/dev/tty.usbmodem14401"  #usbmodem146401" #1462301"  # Change as needed
BAUD_RATE = 115200
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
FRAME_RATE = 30
READY_POSITION_Y_THRESHOLD = 0.05

# File paths
OUTPUT_DIR = "output"
ARM_CSV_PATH = os.path.join(OUTPUT_DIR, "arm.csv")
ROUTINE_CSV_PATH = os.path.join(OUTPUT_DIR, "Routine.csv")

# Initialize Mediapipe Pose for ready position detection
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Global variables
top_cam = None
bottom_cam = None
top_writer = None
bottom_writer = None
recording = False
arm_csv_file = None
arm_csv_writer = None

def initialize_serial():
    """Initializes serial communication with the Arduino/Teensy."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("✅ Serial connection established with Teensy.")
        return ser
    except Exception as e:
        print(f"❌ Serial Error: {e}")
        return None

def initialize_cameras():
    """Initializes both top and bottom cameras."""
    global top_cam, bottom_cam
    
    # Initialize top camera (usually camera 1)
    top_cam = cv2.VideoCapture(1)
    if not top_cam.isOpened():
        print("❌ Error: Could not open top camera (ID: 1)")
        print("Trying alternative camera ID...")
        top_cam = cv2.VideoCapture(1)  # Try alternative ID
        
    # Initialize bottom camera (usually camera 0)
    bottom_cam = cv2.VideoCapture(2)
    if not bottom_cam.isOpened():
        print("❌ Error: Could not open bottom camera (ID: 0)")
        return False
        
    # Set camera properties
    for cam in [top_cam, bottom_cam]:
        if cam is not None and cam.isOpened():
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
            cam.set(cv2.CAP_PROP_FPS, FRAME_RATE)
    
    # Check if both cameras are working
    if top_cam is None or bottom_cam is None or not top_cam.isOpened() or not bottom_cam.isOpened():
        print("❌ One or both cameras failed to initialize.")
        return False
        
    print("✅ Both cameras initialized successfully.")
    return True

def in_ready_position(wrist_y, shoulder_y):
    """Checks if the wrist is close enough to the shoulder in the Y-axis."""
    return abs(wrist_y - shoulder_y) < READY_POSITION_Y_THRESHOLD

def detect_ready_position():
    """Uses OpenCV & Mediapipe to check if the fighter is in ready position."""
    global bottom_cam
    
    if bottom_cam is None or not bottom_cam.isOpened():
        print("❌ Error: Camera not available for ready position detection.")
        return False

    print("🔍 Waiting for fighter to get into ready position...")
    
    while True:
        ret, frame = bottom_cam.read()
        if not ret:
            continue

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            wrist_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
            shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y

            if in_ready_position(wrist_y, shoulder_y):
                print("✅ Fighter is in READY POSITION! Starting countdown...")
                return True

        cv2.putText(frame, "Waiting for ready position...", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Ready Position Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    return False

def start_video_recording(fighter_name, routine):
    """Starts video recording using both cameras."""
    global top_cam, bottom_cam, top_writer, bottom_writer, recording
    
    # Create timestamp for file naming
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create output directory if it doesn't exist
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    # Define file paths for videos
    top_video_path = os.path.join(OUTPUT_DIR, f"top_video.mp4")
    bottom_video_path = os.path.join(OUTPUT_DIR, f"bottom_video.mp4")
    
    # Setup video writers
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frame_width = int(bottom_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(bottom_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(bottom_cam.get(cv2.CAP_PROP_FPS)) or FRAME_RATE
    
    top_writer = cv2.VideoWriter(top_video_path, fourcc, fps, (frame_width, frame_height))
    bottom_writer = cv2.VideoWriter(bottom_video_path, fourcc, fps, (frame_width, frame_height))
    
    if top_writer.isOpened() and bottom_writer.isOpened():
        print(f"📹 Recording started:")
        print(f"   - Top video: {top_video_path}")
        print(f"   - Bottom video: {bottom_video_path}")
        recording = True
    else:
        print("❌ Error: Could not start video recording!")

def stop_video_recording():
    """Stops video recording and saves the files."""
    global recording, top_writer, bottom_writer, top_cam, bottom_cam
    
    if recording:
        recording = False
        if top_writer is not None:
            top_writer.release()
        if bottom_writer is not None:
            bottom_writer.release()
        print("📹 Recording stopped and saved.")
    else:
        print("⚠️ No active recording to stop.")

def close_csv():
    """Close the CSV file properly."""
    global arm_csv_file
    if arm_csv_file is not None:
        arm_csv_file.close()
        print(f"✅ CSV file saved and closed.")

# Update the log_teensy_message function to handle the "ROUTINE DONE" message
def log_teensy_message(routine, message, timestamp):
    """Parse Teensy message and log to CSV."""
    global arm_csv_writer
    
    try:
        # Check for routine completion message
        if "ROUTINE DONE" in message:
            print(f"🏁 Routine {routine} complete!")
            return True  # Return True to indicate routine is complete
            
        # Handle movement position and strike messages
        if "Robot position:" in message:
            key = "POSITION"
            value = message.split("Robot position:")[1].strip()
        elif "STRIKE:" in message:
            key = "STRIKE"
            value = message.split("STRIKE:")[1].strip()
        else:
            # Don't print warnings for status messages
            if not message.startswith("STATUS:") and not "Executing movement" in message:
                print(f"ℹ️ Info message: {message}")
            return False
        
        position = 0  # Default position
        strike_value = 0  # Default strike value (0 = no contact)
        
        if key == "POSITION":
            position = int(value)
        elif key == "STRIKE":
            strike_value = int(value)
            # Convert integer to boolean (any non-zero value is considered TRUE)
            bool_value = strike_value > 0
        
        # Write to CSV (timestamp, routine, position, bool)
        # Keep the original CSV format for compatibility with NewCV_Swap.py
        arm_csv_writer.writerow([timestamp, routine, position, bool_value])
        
        if key == "POSITION":
            print(f"📝 Logged: {timestamp:.3f}s | R{routine} | Pos:{position}")
        else:
            print(f"📝 Logged: {timestamp:.3f}s | R{routine} | {'✓ STRIKE!' if bool_value else '✗ No strike'} (value: {strike_value})")
        
        return False  # Routine not complete
        
    except ValueError:
        print(f"⚠️ Error parsing value in message: {message}")
    except Exception as e:
        print(f"❌ Error logging message: {e}")
    
    return False  # Default: routine not complete

# Update the record_video_and_listen function to handle routine completion
def record_video_and_listen(ser, routine):
    """Captures video frames while listening for Teensy messages."""
    global top_cam, bottom_cam, top_writer, bottom_writer, recording
    
    start_time = time.time()  # Record start time for timestamps
    routine_complete = False
    last_message_time = start_time
    timeout_duration = 20  # 20 seconds of no messages causes auto-timeout
    
    print("📹 Recording started. Waiting for Arduino messages...")
    
    while recording and not routine_complete:
        # Capture frames from both cameras
        ret_top, frame_top = top_cam.read()
        ret_bottom, frame_bottom = bottom_cam.read()
        
        if not ret_top and not ret_bottom:
            print("❌ Both cameras failed to capture frames. Exiting...")
            break
        
        timestamp = time.time() - start_time
        
        # Write frames to video files if valid
        if ret_top:
            top_writer.write(frame_top)
            cv2.imshow("Top Camera", frame_top)
        
        if ret_bottom:
            bottom_writer.write(frame_bottom)
            cv2.imshow("Bottom Camera", frame_bottom)
        
        # Listen for Teensy messages
        if ser and ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if message:
                print(f"📩 Received: {message}")
                routine_complete = log_teensy_message(routine, message, timestamp)
                last_message_time = time.time()
        
        # Check for timeout (no messages for a while)
        if time.time() - last_message_time > timeout_duration:
            print(f"⚠️ No messages received for {timeout_duration} seconds. Assuming routine complete.")
            routine_complete = True
        
        # Check for quit command
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("❌ Manual Stop Detected!")
            break
    
    if routine_complete:
        print("✅ Routine completed successfully!")
    else:
        print("⚠️ Recording stopped without routine completion.")

# Update the send_countdown function to better handle the routine completion
def send_countdown(ser, fighter_name, routine):
    """Detects ready position, counts down, then starts recording and listening."""
    if not ser:
        print("❌ No serial connection. Cannot send countdown.")
        return
    
    # Wait for fighter to be in ready position
    if not detect_ready_position():
        print("❌ Ready position not detected. Exiting...")
        return
    
    # Initialize CSV file
    initialize_csv(fighter_name, routine)
    
    # Start recording
    start_video_recording(fighter_name, routine)
    
    # Countdown
    for i in range(3, 0, -1):
        print(f"Countdown: {i}")
        ser.write(f"{i}\n".encode())  # Send countdown numbers to Arduino
        time.sleep(1)
    
    # Send start command with routine to Teensy
    print(f"🚀 START! Routine: {routine}")
    ser.write(f"START {routine}\n".encode())
    
    # Record video and listen for Teensy messages
    record_video_and_listen(ser, routine)
    
    # Clean up
    stop_video_recording()
    close_csv()
    
    # Add the analysis step
    print("\n🔍 Analyzing boxing performance...")
    try:
        # Import the NewCV_Swap module
        spec = importlib.util.spec_from_file_location("NewCV_Swap", "NewCV_Swap.py")
        cv_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cv_module)
        
        # Call the processing function
        top_video_path = os.path.join(OUTPUT_DIR, "top_video.mp4")
        bottom_video_path = os.path.join(OUTPUT_DIR, "bottom_video.mp4")
        score = cv_module.process_and_evaluate(bottom_video=bottom_video_path, top_video=top_video_path)
        print(f"✅ Analysis complete! Total Score: {score:.2f}%")
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        import traceback
        traceback.print_exc()
        
def get_user_input():
    """Prompts user for fighter name and routine choice."""
    fighter_name = input("Enter fighter's name: ").strip().replace(" ", "_")
    
    routine = None
    while not routine or not routine.isdigit() or int(routine) < 1 or int(routine) > 3:
        routine = input("Select routine (1, 2, or 3): ").strip()
        if not routine.isdigit() or int(routine) < 1 or int(routine) > 3:
            print("Invalid routine. Please enter 1, 2, or 3.")
    
    routine = int(routine)
    print(f"✅ Selected: Fighter '{fighter_name}', Routine {routine}")
    return fighter_name, routine

def cleanup():
    """Clean up resources."""
    global top_cam, bottom_cam
    
    stop_video_recording()
    close_csv()
    
    if top_cam is not None:
        top_cam.release()
    if bottom_cam is not None:
        bottom_cam.release()
    
    cv2.destroyAllWindows()
    print("🧹 Resources cleaned up.")

def initialize_routine_csv(routine):
    """Create the routine definition CSV file based on the selected routine.
    
    This matches the routines defined in the Arduino sketch.
    """
    # Create output directory if it doesn't exist
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    # Define punch sequences for each routine (matching Arduino's routines array)
    # Map the Arduino move codes to punch names:
    # 0 = blockFront = "Jab" 
    # 1 = blockDown = "Cross"
    # 2 = blockSide5 = "Lead Hook"
    
    arduino_routines = {
        0: [0, 0, 0, 1, 0, 1, 0 , 1, 2,0,1,2],
        1: [0, 2, 0, 0, 0, 0, 0 , 2, 0,0,2,0],
        2: [0, 0, 0, 0, 2, 2, 2 , 2, 0,0,0,0]
    }
    '''
    arduino_routines = {
        0: [0, 0, 0, 1, 0, 1, 0, 1, 2, 0, 1, 2],
        1: [0, 5, 2, 0, 3, 2, 1, 4, 3, 1, 4, 3],
        2: [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]
    
    }
    '''
    
    # Map Arduino move codes to punch names
    move_to_punch = {
        0: "Jab",
        1: "Cross", 
        2: "Lead Hook",
        3: "Rear Hook",
        4: "Lead Uppercut",
        5: "Rear Uppercut"
    }
    
    # Create the routine definition CSV
    with open(ROUTINE_CSV_PATH, mode='w', newline='') as routine_file:
        routine_writer = csv.writer(routine_file)
        routine_writer.writerow(['Punch'])
        
        # Get the routine array (accounting for 0-based indexing in Python vs 1-based in user input)
        routine_idx = routine - 1
        if routine_idx in arduino_routines:
            for move_code in arduino_routines[routine_idx]:
                punch = move_to_punch.get(move_code, "Unknown")
                routine_writer.writerow([punch])
        else:
            print(f"⚠️ Unknown routine {routine}. Creating an empty routine file.")
    
    print(f"✅ Routine definition created at {ROUTINE_CSV_PATH}")
# Modify the initialize_csv function to call the new function
def initialize_csv(fighter_name, routine):
    """Initialize CSV file for arm data logging."""
    global arm_csv_file, arm_csv_writer
    
    # Create output directory if it doesn't exist
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    # Open the CSV file for writing
    arm_csv_file = open(ARM_CSV_PATH, mode='w', newline='')
    arm_csv_writer = csv.writer(arm_csv_file)
    
    # Write the header row
    # Keep the original header format for compatibility with NewCV_Swap.py
    arm_csv_writer.writerow(['Timestamp', 'Routine', 'Position', 'Bool'])
    
    # Initialize the routine definition file
    initialize_routine_csv(routine)
    
    print(f"✅ CSV files initialized at {OUTPUT_DIR}")

def prompt_restart():
    """Asks the user if they want to restart the program."""
    print("\n")
    print("=" * 50)
    print("Training session completed. Press ENTER to start a new session.")
    print("Or press Ctrl+C to exit the program.")
    print("=" * 50)
    try:
        input("\nPress ENTER to continue...")
        return True
    except KeyboardInterrupt:
        print("\nExiting program...")
        return False
    
# Main Execution
if __name__ == "__main__":
    try:
        # Get user input for fighter name and routine
        fighter_name, routine = get_user_input()
        
        # Initialize serial connection
        ser = initialize_serial()
        
        # Initialize cameras
        if initialize_cameras():
            # Send countdown, start training session, and run evaluation
            send_countdown(ser, fighter_name, routine)
            print("\n✅ Boxing training session complete!")
        else:
            print("❌ Camera initialization failed. Cannot continue.")

        restart = prompt_restart()
            
    except KeyboardInterrupt:
        print("\n⚠️ Program interrupted by user.")
        restart = False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        restart = prompt_restart()
    finally:
        cleanup()
        
 

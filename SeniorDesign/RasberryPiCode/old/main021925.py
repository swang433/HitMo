import time
import cv2
import mediapipe as mp
import serial
import sqlite3
import datetime
import os

DB_NAME = "training_log.db"
SERIAL_PORT = "/dev/tty.usbmodem144860601"  
BAUD_RATE = 115200  
VIDEO_WIDTH = 640  
VIDEO_HEIGHT = 480  
FRAME_RATE = 30  

# Initialize Mediapipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

READY_POSITION_Y_THRESHOLD = 0.05  

# Global variables
video_writer = None
cap = None
recording = False

def initialize_serial():
    """ Initializes serial communication with the Teensy. """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("✅ Serial connection established with Teensy.")
        return ser
    except Exception as e:
        print(f"❌ Serial Error: {e}")
        return None

def in_ready_position(wrist_y, shoulder_y):
    """ Checks if the wrist is close enough to the shoulder in the Y-axis. """
    return abs(wrist_y - shoulder_y) < READY_POSITION_Y_THRESHOLD

def detect_ready_position():
    """ Uses OpenCV & Mediapipe to check if the fighter is in ready position. """
    global cap
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        return False

    print("🔍 Waiting for fighter to get into ready position...")

    while True:
        ret, frame = cap.read()
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
                cap.release()  # Release the camera before recording starts
                return True

        cv2.putText(frame, "Waiting for ready position...", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Ready Position Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  

    cap.release()
    return False

def start_video_recording(fighter_name, routine):
    """ Starts video recording using OpenCV. """
    global cap, video_writer, recording

    cap = cv2.VideoCapture(0)

    # Create video file
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    video_filename = f"videos/{fighter_name}_{routine}_{timestamp}.mp4"

    if not os.path.exists("videos"):
        os.makedirs("videos")

    # Define codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))

    if cap.isOpened() and video_writer.isOpened():
        print(f"📹 Recording started: {video_filename}")
        recording = True
    else:
        print("❌ Error: Could not start video recording!")

def stop_video_recording():
    """ Stops video recording and saves the file. """
    global recording, video_writer, cap

    if recording:
        recording = False
        video_writer.release()
        cap.release()
        cv2.destroyAllWindows()
        print("📹 Recording stopped and saved.")
    else:
        print("⚠️ No active recording to stop.")

def record_video_and_listen(ser, fighter_id):
    """ Captures video frames while listening for Teensy messages. """
    global cap, video_writer, recording

    while recording:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame capture failed!")
            break

        video_writer.write(frame)
        cv2.imshow("Recording", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("❌ Manual Stop Detected!")
            stop_video_recording()
            break

        # Listen for Teensy messages
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if message:
                print(f"📩 Received: {message}")
                handle_teensy_message(fighter_id, message)

def send_countdown(ser, fighter_name, routine):
    """ Detects the ready position, starts countdown, then records while listening for Teensy. """
    if not ser:
        print("❌ No serial connection. Cannot send countdown.")
        return

    # Wait for the fighter to be in ready position
    if not detect_ready_position():
        print("❌ Ready position not detected. Exiting...")
        return

    start_video_recording(fighter_name, routine)  # Start recording after stance is detected

    for i in range(3, 0, -1):
        print(f"Countdown: {i}")
        ser.write(f"{i}\n".encode())
        time.sleep(1)

    print("🚀 START!")
    ser.write("START\n".encode())

    record_video_and_listen(ser, fighter_id)  # Capture frames and listen to Teensy

def handle_teensy_message(fighter_id, message):
    """ Processes incoming Teensy messages and logs them with fighter_id. """
    global recording

    try:
        key, value = message.split(":")
        key = key.strip()
        value = value.strip()

        if key.startswith("BUTTON"):
            log_event(fighter_id, "button_press", value)
        elif key == "DISTANCE":
            log_event(fighter_id, "distance_measured", value)
        elif key == "POSITION":
            log_event(fighter_id, "robot_arm_position", value)
        elif key == "STATUS":
            log_event(fighter_id, "program_status", value)
            if value == "COMPLETE":
                print("🎯 Program Completed!")
                stop_video_recording()
    except ValueError:
        print(f"⚠️ Unrecognized format: {message}")

def log_event(fighter_id, event_type, event_data):
    """ Logs an event in SQLite with the correct fighter_id. """
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO events (fighter_id, event_type, event_data, timestamp) VALUES (?, ?, ?, ?)",
                   (fighter_id, event_type, event_data, timestamp))
    conn.commit()
    conn.close()
    print(f"✅ Logged: Fighter ID {fighter_id} | {event_type} | Data: {event_data} | Time: {timestamp}")

def get_user_input():
    """ Prompts user for fighter name and routine choice, then stores in SQLite. """
    fighter_name = input("Enter fighter’s name: ").strip().replace(" ", "_")
    routine = input("Select routine: ").strip().replace(" ", "_")

    if not fighter_name or not routine:
        print("Error: Both fighter name and routine are required.")
        return get_user_input()

    session_timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO fighters (name, routine, session_timestamp) VALUES (?, ?, ?)",
                   (fighter_name, routine, session_timestamp))
    conn.commit()
    fighter_id = cursor.lastrowid
    conn.close()

    print(f"✅ Session created: {fighter_name} selected {routine}")
    print(f"📌 Database Entry ID: {fighter_id}")
    return fighter_id, fighter_name, routine

# Main Execution
if __name__ == "__main__":
    fighter_id, fighter_name, routine = get_user_input()
    ser = initialize_serial()
    send_countdown(ser, fighter_name, routine)

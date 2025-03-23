import time
import sqlite3
import datetime
import serial
import os
import cv2

DB_NAME = "training_log.db"
SERIAL_PORT = "/dev/tty.usbmodem144860601"  # Change this to match your port
BAUD_RATE = 115200  # Match this with Teensy's serial settings
HEARTBEAT_TIMEOUT = 3 # If no heartbeat is received in 3 seconds, log an error
VIDEO_WIDTH = 640   # Adjust based on camera
VIDEO_HEIGHT = 480  # Adjust based on camera
FRAME_RATE = 30     # Frame rate for video recording

# Initialize global variable for video recording
video_writer = None
cap = None

def initialize_serial():
    """
    Initializes serial communication with the Teensy.
    Returns the serial connection object.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Give Teensy time to establish connection
        print("✅ Serial connection established with Teensy.")
        return ser
    except Exception as e:
        print(f"❌ Error: Could not establish serial connection - {e}")
        return None


def start_video_recording(fighter_name, routine):
    """
    Starts recording video using OpenCV.
    """
    global cap, video_writer

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    video_filename = f"videos/{fighter_name}_{routine}_{timestamp}.avi"

    # Ensure the "videos" directory exists
    if not os.path.exists("videos"):
        os.makedirs("videos")

    cap = cv2.VideoCapture(0)  # Open USB camera (Device 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec format
    video_writer = cv2.VideoWriter(video_filename, fourcc, FRAME_RATE, (VIDEO_WIDTH, VIDEO_HEIGHT))

    if cap.isOpened():
        print(f"📹 Video recording started: {video_filename}")
    else:
        print("❌ Error: Could not start video recording!")

def stop_video_recording():
    """
    Stops video recording and saves the file.
    """
    global cap, video_writer

    if cap and video_writer:
        cap.release()
        video_writer.release()
        print("📹 Video recording stopped and saved.")
    else:
        print("⚠️ No active video recording to stop.")
        
        

def send_countdown(ser):
    """
    Sends a countdown (3, 2, 1, START) to the Teensy.
    Also displays the countdown in the terminal.
    """
    if not ser:
        print("❌ Serial connection not available. Cannot send countdown.")
        return
    
    start_video_recording(fighter_name, routine)  # Start recording before countdown

    for i in range(3, 0, -1):
        print(f"Countdown: {i}")
        ser.write(f"{i}\n".encode())  # Send number as a string
        time.sleep(1)

    print("🚀 START!")
    ser.write("START\n".encode())  # Send "START" to the Teensy


def log_event(fighter_id, event_type, event_data):
    """
    Logs an event to the SQLite database.

    Args:
        fighter_id (int): The fighter's database ID.
        event_type (str): Type of event (e.g., 'strike_detected', 'distance_measured').
        event_data (str): The data associated with the event.
    """
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Insert into database
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO events (fighter_id, event_type, event_data, timestamp) VALUES (?, ?, ?, ?)",
                   (fighter_id, event_type, event_data, timestamp))
    conn.commit()
    conn.close()

    print(f"✅ Logged event: {event_type} | Data: {event_data} | Time: {timestamp}")



def get_user_input():
    """
    Prompts the user for the fighter's name and routine choice, then stores this info in SQLite.
    
    Returns:
        fighter_id (int): The ID of the fighter in the database.
        fighter_name (str): The fighter's name.
        routine (str): The selected routine.
    """
    # Get user input
    fighter_name = input("Enter fighter’s name: ").strip().replace(" ", "_")
    routine = input("Select routine: ").strip().replace(" ", "_")

    # Ensure valid input
    if not fighter_name or not routine:
        print("Error: Both fighter name and routine are required.")
        return get_user_input()  # Recursively ask again

    # Get timestamp
    session_timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Insert into database
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO fighters (name, routine, session_timestamp) VALUES (?, ?, ?)",
                   (fighter_name, routine, session_timestamp))
    conn.commit()

    # Get the fighter's ID
    fighter_id = cursor.lastrowid
    conn.close()

    print(f"✅ Session created: {fighter_name} selected {routine}")
    print(f"📌 Database Entry ID: {fighter_id}")

    return fighter_id, fighter_name, routine


def listen_for_teensy(ser, fighter_id):
    """
    Continuously listens for serial messages from Teensy and processes them.
    """
    last_heartbeat_time = time.time()

    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            if not message:
                continue  # Ignore empty messages

            print(f"📩 Received: {message}")  # Print raw message

            # Detect heartbeat signal
            if message == "HEARTBEAT":
                last_heartbeat_time = time.time()
                print("💓 Teensy Heartbeat Received")
                continue

            # Check for heartbeat timeout
            if time.time() - last_heartbeat_time > HEARTBEAT_TIMEOUT:
                print("❌ WARNING: Teensy heartbeat lost!")

            # Parse incoming messages
            try:
                key, value = message.split(":")
                key = key.strip()
                value = value.strip()

                if key.startswith("BUTTON"):
                    log_event(fighter_id, key, value)  # Example: BUTTON_1:1
                elif key == "DISTANCE":
                    log_event(fighter_id, "distance_measured", value)  # Example: DISTANCE:134
                elif key == "POSITION":
                    log_event(fighter_id, "robot_arm_position", value)  # Example: POSITION:2
                elif key == "STATUS":
                    log_event(fighter_id, "program_status", value)  # Example: STATUS: COMPLETE
                    if value == "COMPLETE":
                        print("🎯 Program Completed!")
                        break  # Exit listening loop if program is complete
            except ValueError:
                print(f"⚠️ Unrecognized format: {message}")
                
        # Capture and write video frames while listening for messages
        if cap and video_writer:
            ret, frame = cap.read()
            if ret:
                video_writer.write(frame)

        time.sleep(0.1)  # Adjust as needed
        
# Example Usage
if __name__ == "__main__":
    fighter_id, fighter_name, routine = get_user_input()
    print(f"Fighter ID: {fighter_id}, Fighter: {fighter_name}, Routine: {routine}")
    ser = initialize_serial()  # Start serial communication
    send_countdown(ser)  # Send countdown to Teensy
    listen_for_teensy(ser, fighter_id)  # Start listening for messages

import cv2
import time

# Max resolutions for each camera (from your test results)
CAM0_WIDTH, CAM0_HEIGHT = 1280, 720  # Bottom Camera
CAM1_WIDTH, CAM1_HEIGHT = 1920, 1080  # Top Camera
FPS = 30  # Default FPS, can be adjusted if needed

# Function to initialize a camera with specific resolution
def init_camera(cam_id, width, height, fps=FPS):
    cam = cv2.VideoCapture(cam_id)
    if not cam.isOpened():
        print(f"Error: Camera {cam_id} failed to open.")
        return None
    
    # Set camera resolution and FPS
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, fps)
    
    # Check if camera is actually capturing frames
    ret, frame = cam.read()
    if not ret:
        print(f"Error: Camera {cam_id} is not capturing frames.")
        cam.release()
        return None

    print(f"Camera {cam_id} initialized: {width}x{height} @ {fps} FPS")
    return cam

# Initialize both cameras with their max resolutions
top_cam = init_camera(1, CAM1_WIDTH, CAM1_HEIGHT)  # Camera 1 (Top)
bottom_cam = init_camera(0, CAM0_WIDTH, CAM0_HEIGHT)  # Camera 0 (Bottom)

if not top_cam or not bottom_cam:
    print("One or both cameras failed to start. Exiting...")
    exit()

# Get actual FPS (some cameras may not support 30 FPS)
fps = int(top_cam.get(cv2.CAP_PROP_FPS)) or FPS

# Define video writers with respective resolutions
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # More compatible than 'mp4v'
outT = cv2.VideoWriter("top_video.mp4", fourcc, fps, (CAM1_WIDTH, CAM1_HEIGHT))
outB = cv2.VideoWriter("bottom_video.mp4", fourcc, fps, (CAM0_WIDTH, CAM0_HEIGHT))

print("Recording... Press 'q' to stop.")
time.sleep(2)  # Delay to stabilize cameras

while True:
    retT, frameT = top_cam.read()
    retB, frameB = bottom_cam.read()

    if not retT:
        print("Warning: Top camera lost frames.")
    if not retB:
        print("Warning: Bottom camera lost frames.")

    if not retT and not retB:
        print("Both cameras stopped capturing. Exiting...")
        break

    if retT:
        outT.write(frameT)
        cv2.imshow("Top Camera", frameT)
    if retB:
        outB.write(frameB)
        cv2.imshow("Bottom Camera", frameB)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Recording stopped.")
        break

# Cleanup
top_cam.release()
bottom_cam.release()
outT.release()
outB.release()
cv2.destroyAllWindows()

print("Videos saved as top_video.mp4 and bottom_video.mp4")

import cv2

# Open both cameras
top_cam = cv2.VideoCapture(2)  # Top Camera
bottom_cam = cv2.VideoCapture(1)  # Bottom Camera

# Set video properties
frame_width = int(top_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(top_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(top_cam.get(cv2.CAP_PROP_FPS)) or 30

# Define MP4 video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
outT = cv2.VideoWriter("top_video.mp4", fourcc, fps, (frame_width, frame_height))
outB = cv2.VideoWriter("bottom_video.mp4", fourcc, fps, (frame_width, frame_height))

print("Recording... Press 'q' to stop.")

while top_cam.isOpened() and bottom_cam.isOpened():
    retT, frameT = top_cam.read()
    retB, frameB = bottom_cam.read()

    if not retT or not retB:
        print("Error: One of the cameras stopped working.")
        break

    # Rotate bottom frame
    #frameB = cv2.rotate(frameB, cv2.ROTATE_180)

    # Write frames
    outT.write(frameT)
    outB.write(frameB)

    # Display live preview
    cv2.imshow("Top Camera", frameT)
    cv2.imshow("Bottom Camera", frameB)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Recording stopped.")
        break

top_cam.release()
bottom_cam.release()
outT.release()
outB.release()
cv2.destroyAllWindows()

print("Videos saved as top_video.mp4 and bottom_video.mp4")

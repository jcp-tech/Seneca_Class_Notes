import cv2 as cv
import datetime
import os

# Start a video capture, using device's camera
cap = cv.VideoCapture(0)

# Check if video file opened successfully
if not cap.isOpened():
    print("Error opening video stream or file")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
print("Frame width:", frame_width)
print("Frame height:", frame_height)

# Snapshot counter
snapshot_count = 1

# Read until video is completed
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    cv.imshow('frame', frame)
    key = cv.waitKey(500)

    if key & 0xFF == ord('q'):
        break

    elif key & 0xFF == ord('x'):
        # Pad the snapshot with 40 pixels using constant red border
        padded_frame = cv.copyMakeBorder(frame, 40, 80, 40, 40, cv.BORDER_CONSTANT, value=[0, 0, 255])

        # Crop 15 pixels from all sides
        h, w = padded_frame.shape[:2]
        cropped_frame = padded_frame[15:h-15, 15:w-15]

        # Get current date and time
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Set position for timestamp (bottom left corner)
        text_position = (50, cropped_frame.shape[0] - 30)

        # Add timestamp to image
        cv.putText(cropped_frame, timestamp, text_position, 
                   fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255), thickness=2)

        # Create auto-incrementing filename
        script_dir = os.path.dirname(os.path.abspath(__file__)) # Saves the script directory
        filename = os.path.join(script_dir, f'image{snapshot_count:02d}.jpg')
        cv.imwrite(filename, cropped_frame)
        print(f"Snapshot taken and saved as {filename}")

        # Show the snapshot in a new window for 1 second
        cv.imshow('snapshot', cropped_frame)
        cv.waitKey(1000)
        cv.destroyWindow('snapshot')

        snapshot_count += 1

# Release the video capture
cap.release()
cv.destroyAllWindows()

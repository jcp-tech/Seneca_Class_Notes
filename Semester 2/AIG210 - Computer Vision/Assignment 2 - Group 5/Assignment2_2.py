# Assignment2_2
# Part II: Drawing Application

import numpy as np  
import cv2 as cv

# Q1 - Drawing Setup (Based on OpenCV: Mouse as Paint-Brush - More Advanced Demo)
drawing = False  # true if mouse is pressed
mode = True  # True = polylines, False = polygon
points = []

# Q3a - Load black image (or custom image if you want to swap later)
img = np.zeros((512, 512, 3), np.uint8)
img_copy = img.copy()  # Backup image

# Colors
polyline_colour = (0, 255, 0)    # Green
polygon_colour = (255, 0, 255)   # Purple
preview_colour = (200, 200, 200) # Light gray for preview

# Q1 - Modify shape: draw rectangles or small circles if desired (currently drawing lines)
# Q2 - Remove (-1), control thickness (already using thickness=2)

# Mouse callback function
def draw_shape(event, x, y, flags, param):
    global drawing, points, mode, img

    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        points.append((x, y))

    elif event == cv.EVENT_RBUTTONDOWN:
        drawing = False
        if len(points) > 1:
            pts = np.array(points, np.int32)
            pts = pts.reshape((-1, 1, 2))
            if mode:
                # Q3a - Drawing polylines
                cv.polylines(img, [pts], isClosed=False, color=polyline_colour, thickness=2)
            else:
                # Q3a - Drawing polygons
                cv.polylines(img, [pts], isClosed=True, color=polygon_colour, thickness=2)
        points = []

# Setup
cv.namedWindow('image')
cv.setMouseCallback('image', draw_shape)

while True:
    temp_img = img.copy()

    # Q2 - Lower refresh rate from ultra-fast to visible speed (simulate by adding delay below)
    # Q2 - Preview current line in light gray
    if drawing and len(points) > 1:
        for i in range(len(points)-1):
            cv.line(temp_img, points[i], points[i+1], preview_colour, thickness=1)

    cv.imshow('image', temp_img)
    k = cv.waitKey(100) & 0xFF  # Q2 - Slower refresh rate (changed from 1ms to 100ms)

    if k == ord('m'):
        mode = not mode  # Toggle between polyline and polygon
    elif k == ord('x'):
        # Q3b - Save image
        cv.imwrite('saved_image.png', img)
        print("Image saved as 'saved_image.png'")
    elif k == 27:  # ESC
        break

cv.destroyAllWindows()
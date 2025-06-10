# Assignment02_2.py

import numpy as np
import cv2 as cv
import os
import tkinter as tk
from tkinter import filedialog, messagebox

# --- Initial Setup ---
root = tk.Tk()
root.withdraw() # Hide the main tkinter window

# Drawing Modes
RECTANGLE_MODE = 0
CIRCLE_MODE = 1
POLYLINE_MODE = 2
POLYGON_MODE = 3
drawing_modes = ['Rectangle', 'Circle', 'Polyline', 'Polygon']
current_drawing_mode = RECTANGLE_MODE

drawing = False # true if mouse is pressed
ix,iy = -1,-1
img = None # Will be initialized based on user choice
active_points = [] # For polylines/polygons

# Define a list of colors (B, G, R format)
colors = [
    (0, 255, 0),  # Green
    (0, 0, 255),  # Red
    (255, 0, 0),  # Blue
    (0, 255, 255), # Yellow
    (255, 0, 255), # Magenta
    (255, 255, 0), # Cyan
    (255, 255, 255),# White
    (0,0,0) # Black - Added for completeness, though background is black
]
current_color_index = 0 # Start with Green
drawing_thickness = 2

# --- Ask user to load image or use blank ---
if messagebox.askyesno("Load Image", "Do you want to select an image to load? (No = blank canvas)"):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = filedialog.askopenfilename(initialdir=script_dir, title="Select an Image",
                                           filetypes=(("Image files", "*.jpg *.jpeg *.png *.bmp *.tiff"),
                                                      ("All files", "*.*")))
    if filepath:
        img = cv.imread(filepath)
        if img is None:
            messagebox.showerror("Error", f"Failed to load image: {filepath}\\nStarting with a blank canvas.")
            img = np.zeros((512,512,3), np.uint8)
    else: # User cancelled dialog
        img = np.zeros((512,512,3), np.uint8)
else:
    img = np.zeros((512,512,3), np.uint8)

if img is None: # Should not happen if logic above is correct, but as a fallback
    img = np.zeros((512,512,3), np.uint8)

# mouse callback function
def drawing_callback(event,x,y,flags,param):
    global ix,iy,drawing,current_drawing_mode, current_color_index, colors, drawing_thickness, img, active_points

    current_color = colors[current_color_index]

    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
        if current_drawing_mode == POLYLINE_MODE or current_drawing_mode == POLYGON_MODE:
            active_points.append((x,y))

    elif event == cv.EVENT_MOUSEMOVE:
        if drawing == True:
            temp_img = img.copy()
            if current_drawing_mode == RECTANGLE_MODE:
                cv.rectangle(temp_img,(ix,iy),(x,y),current_color,drawing_thickness)
            elif current_drawing_mode == CIRCLE_MODE:
                # For circle preview, draw on temp_img, actual draw on LBUTTONUP
                # Calculate radius for preview
                radius = int(np.sqrt((x-ix)**2 + (y-iy)**2))
                cv.circle(temp_img,(ix,iy),radius,current_color,drawing_thickness)
            elif (current_drawing_mode == POLYLINE_MODE or current_drawing_mode == POLYGON_MODE) and active_points:
                pts_preview = np.array(active_points + [(x,y)], np.int32)
                pts_preview = pts_preview.reshape((-1,1,2))
                is_closed_preview = current_drawing_mode == POLYGON_MODE
                cv.polylines(temp_img,[pts_preview],isClosed=is_closed_preview, color=current_color, thickness=drawing_thickness)

            if current_drawing_mode != CIRCLE_MODE: # Circle is finalized on LBUTTONUP differently
                 cv.imshow('image',temp_img)


    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        if current_drawing_mode == RECTANGLE_MODE:
            cv.rectangle(img,(ix,iy),(x,y),current_color,drawing_thickness)
        elif current_drawing_mode == CIRCLE_MODE:
            radius = int(np.sqrt((x-ix)**2 + (y-iy)**2)) # Use distance from start to end as radius
            cv.circle(img,(ix,iy),radius,current_color,drawing_thickness)
        # Polyline/Polygon are finalized with RBUTTONDOWN

    elif event == cv.EVENT_RBUTTONDOWN:
        if (current_drawing_mode == POLYLINE_MODE or current_drawing_mode == POLYGON_MODE) and len(active_points) > 1:
            pts = np.array(active_points, np.int32)
            pts = pts.reshape((-1,1,2))
            is_closed = current_drawing_mode == POLYGON_MODE
            cv.polylines(img,[pts],isClosed=is_closed, color=current_color, thickness=drawing_thickness)
        active_points = [] # Reset points for next shape

cv.namedWindow('image')
cv.setMouseCallback('image',drawing_callback)

while(1):
    # Display the current mode on the image
    display_img = img.copy()
    mode_text = f"Mode: {drawing_modes[current_drawing_mode]}"
    color_text = f"Color: {colors[current_color_index]}"
    cv.putText(display_img, mode_text, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1, cv.LINE_AA)
    cv.putText(display_img, color_text, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1, cv.LINE_AA)
    if (current_drawing_mode == POLYLINE_MODE or current_drawing_mode == POLYGON_MODE) and active_points:
        # Draw current polyline points on the display image before mouse move adds the current segment
        pts_preview_static = np.array(active_points, np.int32).reshape((-1,1,2))
        is_closed_preview_static = current_drawing_mode == POLYGON_MODE
        cv.polylines(display_img,[pts_preview_static],isClosed=False, color=colors[current_color_index], thickness=drawing_thickness)


    cv.imshow('image',display_img)
    k = cv.waitKey(20) & 0xFF

    if k == ord('m'):
        current_drawing_mode = (current_drawing_mode + 1) % len(drawing_modes)
        active_points = [] # Clear points when changing mode
        drawing = False # Stop any active drawing
    elif k == ord('c'):
        current_color_index = (current_color_index + 1) % len(colors)
    elif k == ord('x'):
        save_path = filedialog.asksaveasfilename(defaultextension=".png",
                                                 initialdir=os.path.dirname(os.path.abspath(__file__)),
                                                 filetypes=(("PNG files", "*.png"),
                                                            ("JPEG files", "*.jpg"),
                                                            ("All files", "*.*")))
        if save_path:
            cv.imwrite(save_path, img)
            print(f"Image saved to {save_path}")
    elif k == ord('q') or k == 27: # q or ESC to quit
        break

cv.destroyAllWindows()
root.destroy() # Clean up tkinter root.
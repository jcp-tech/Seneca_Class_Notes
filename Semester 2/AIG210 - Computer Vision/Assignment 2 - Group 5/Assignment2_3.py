# Assignment02_3.py

import cv2, os
import numpy as np

# PART 1: Open and Display a Color Image
script_dir = os.path.dirname(os.path.abspath(__file__)) # Get the directory of the current script
img = cv2.imread(os.path.join(script_dir, "nail_polish.jpg"))  # <-- Change to your image file
if img is None:
    print("Error: Image not found.")
    exit()
cv2.imshow('Original Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

while True:
    print("\n\n\n\nMenu:")
    print("1. Rotate")
    print("2. Resize")
    print("3. Perspective Transform")
    print("4. Exit")
    choice = input("\nChoose an option (1-4): ")

    # PART 3: Rotation
    if choice == '1':
        angle = float(input("Enter angle in degrees (positive=CCW, negative=CW): "))
        (h, w) = img.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv2.warpAffine(img, M, (w, h))
        cv2.imshow(f'Rotated by {angle} degrees', rotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # cv2.imwrite(f'rotated_{int(angle)}deg.jpg', rotated) # Optionally, you can save

    # PART 4: Resizing
    elif choice == '2':
        x = float(input("Enter scale factor for width: "))
        y = float(input("Enter scale factor for height: "))
        resized = cv2.resize(img, None, fx=x, fy=y)
        cv2.imshow(f'Resized (fx={x}, fy={y})', resized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # cv2.imwrite(f'resized_{x}_{y}.jpg', resized) # Optionally save the resized image

    # PART 5: Perspective Transformation
    elif choice == '3':
        H = np.array([
            [0.4, -0.4, 190],
            [0.15, 0.4, 100],
            [0, 0, 1]
        ], dtype=np.float32)
        (h, w) = img.shape[:2]
        perspective = cv2.warpPerspective(img, H, (w, h))
        cv2.imshow('Perspective Transform', perspective)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # cv2.imwrite('perspective_transform.jpg', perspective) # Optionally save the perspective transformed image

    elif choice == '4':
        print("Exiting...")
        break
    else:
        print("Invalid input. Please enter 1-4.")
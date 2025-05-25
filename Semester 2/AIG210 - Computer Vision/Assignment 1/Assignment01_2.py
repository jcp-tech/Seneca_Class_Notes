import cv2 as cv

# Load color image from your folder
image = cv.imread('penguin.jpg')  # 🔁 Replace with your actual image filename

# Check if image loaded successfully
if image is None:
    print("Error: Could not load image. Check the file path.")
    exit()

# Show the original image
cv.imshow("Original Image", image)
cv.waitKey(1000)  # Show for 1 second
cv.imwrite('original_image.jpg', image)  # Save original

# Convert from BGR to HSV color space
hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# Set Hue to 0 (red)
hsv_h0 = hsv_image.copy()
hsv_h0[:, :, 0] = 0
bgr_h0 = cv.cvtColor(hsv_h0, cv.COLOR_HSV2BGR)
cv.imshow("Hue 0", bgr_h0)
cv.waitKey(1000)
cv.imwrite('image_hue0.jpg', bgr_h0)  # Save image

# Set Hue to 90 (green)
hsv_h90 = hsv_image.copy()
hsv_h90[:, :, 0] = 90
bgr_h90 = cv.cvtColor(hsv_h90, cv.COLOR_HSV2BGR)
cv.imshow("Hue 90", bgr_h90)
cv.waitKey(1000)
cv.imwrite('image_hue90.jpg', bgr_h90)  # Save image

# Close all windows
cv.destroyAllWindows()

# import numpy as np
# import cv2

# drawing = False  # (Used for drawing state)
# mode = True  # PART 3a: True = polylines, False = polygon
# points = []

# # PART 3a: Black image used
# img = np.zeros((512, 512, 3), np.uint8)
# img_copy = img.copy()

# polyline_colour = (0, 255, 0)
# polygon_colour = (255, 0, 255)

# def draw_circle(event, x, y, flags, param):
#     global drawing, points, mode, img

#     if event == cv2.EVENT_LBUTTONDOWN:
#         drawing = True
#         points.append((x, y))

#     elif event == cv2.EVENT_RBUTTONDOWN:
#         drawing = False
#         if len(points) > 1:
#             pts = np.array(points, np.int32)
#             pts = pts.reshape((-1, 1, 2))

#             # PART 3a + PART 2: Drawing polyline with changed thickness
#             if mode:
#                 cv2.polylines(img, [pts], isClosed=False, color=polyline_colour, thickness=2)
#             else:
#                 cv2.polylines(img, [pts], isClosed=True, color=polygon_colour, thickness=2)
#         points = []

# cv2.namedWindow('image')
# cv2.setMouseCallback('image', draw_circle)

# while True:
#     temp_img = img.copy()

#     if drawing and len(points) > 1:
#         for i in range(len(points) - 1):
#             cv2.line(temp_img, points[i], points[i + 1], (200, 200, 200), 1)

#     cv2.imshow('image', temp_img)
#     k = cv2.waitKey(100) & 0xFF # PART 2: Lowered refresh rate (was 1)

#     if k == ord('m'):
#         mode = not mode  # PART 3a: Toggle between polyline and polygon
#     elif k == ord('x'):
#         cv2.imwrite('saved_image.png', img)  # PART 3b: Save image
#         print("Image saved as 'saved_image.png'")
#     elif k == 27:  # ESC to exit
#         break

# cv2.destroyAllWindows()
import cv2

# Node Generation
# img = cv2.imread('map6.png')
# for i in range(len(img[0])):
#     for j in range(len(img)):
#         pixel = img[j][i]
#         if pixel[0] == pixel[1] == 0 and pixel[2] == 255:
#             print(f'N {i} {j} NA')
#         if pixel[0] == 255 and pixel[1] == pixel[2] == 0:
#             print(f'n {i} {j}')

# Obstacle Generation
# img = cv2.imread('map6.png')
# bw = cv2.inRange(img, (0, 254, 0), (0, 255, 0))
# contours = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
# for contour in contours:
#     x, y, w, h = cv2.boundingRect(contour)
#     print(f'b {x} {y} {x + w - 1} {y + h - 1}')

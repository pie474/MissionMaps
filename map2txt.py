import cv2

img = cv2.imread('map5.png')

# color_lst = []
# for i in range(len(img)):
#     for j in range(len(img[0])):
#         pixel = img[i][j]
#         color_lst.append((pixel[0], pixel[1], pixel[2]))
#
# color_lst = list(set(color_lst))
# color_lst.remove((0, 0, 0))
# color_lst.remove((255, 255, 255))
# color_lst.remove((0, 0, 255))
# color_lst.remove((255, 0, 0))
# print('imposter colors: ', color_lst)

for i in range(len(img)):
    for j in range(len(img[0])):
        pixel = img[i][j]
        if (pixel[0] == pixel[1] == 0 and pixel[2] == 255) or (pixel[0] == 255 and pixel[1] == pixel[2] == 0):
            print(f'n {i} {j}')

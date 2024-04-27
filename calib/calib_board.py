import cv2
from cv2 import aruco
import numpy as np

# 设置ChArUco Board的参数
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
square_length = 0.04
marker_length = 0.02
markers_x = 5
markers_y = 7
size = (markers_x, markers_y)
board = aruco.CharucoBoard(size, square_length, marker_length, aruco_dict)

# 设置Board的尺寸和边缘长度
margins = 50
image_width = int(markers_x * square_length + 2 * margins)
image_height = int(markers_y * square_length + 2 * margins)
image_size = (800, 600)

board_image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
board.generateImage(image_size, board_image, margins)

# 调整图像大小
resized_image = cv2.resize(board_image, image_size, interpolation=cv2.INTER_LINEAR)

# 显示和保存图像
cv2.imshow("ChArUco Board", resized_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("charuco_board.jpg", resized_image)
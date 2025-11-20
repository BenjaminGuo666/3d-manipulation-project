#!/usr/bin/env python3
import cv2
import numpy as np

# 9x6 内角点 → 10x7 个方格（横x纵），可按需改
squares_x, squares_y = 10, 7
px = 80
img = np.zeros((squares_y*px, squares_x*px), np.uint8)

for y in range(squares_y):
    for x in range(squares_x):
        if (x + y) % 2 == 0:
            img[y*px:(y+1)*px, x*px:(x+1)*px] = 255

cv2.namedWindow("board", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("board", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("board", img)
print("全屏棋盘格已显示：按 ESC 退出")
while True:
    if cv2.waitKey(1) & 0xFF == 27:
        break
cv2.destroyAllWindows()

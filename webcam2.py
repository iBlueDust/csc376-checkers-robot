import cv2
import sys
import matplotlib.pyplot as plt
import numpy as np
 
cv2.namedWindow("preview")
vc = cv2.VideoCapture(4)
 
if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False
 
plt.ion()
plt.show()
 
while rval:
    # frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
    frame_blur = cv2.blur(frame, (5, 5))
    # (B, G, R) = cv2.split(frame_blur)
    # B_cny = cv2.Canny(B, 15, 150)
    # G_cny = cv2.Canny(G, 15, 150)
    # R_cny = cv2.Canny(R, 15, 150)
    # img_cny = cv2.merge([B_cny, G_cny, R_cny]) 
    # img_cny_mono = B_cny + G_cny + R_cny
    # img_cny_mono = cv2.cvtColor(img_cny, cv2.COLOR_BGR2GRAY)
 
    BOARD = (142, 75), (478, 418)
 
    board = frame[BOARD[0][1]:BOARD[1][1], BOARD[0][0]:BOARD[1][0], :]
 
    frame = cv2.rectangle(frame, *BOARD, color=(0, 0, 255))
 
    high_green = (96,255,197)
    low_green = (57,93,68)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, low_green, high_green)
    color_mask = cv2.inRange(frame_hsv, (0,0,0), (180,120,54))
 
    for c in range(8):
        for r in range(8):
            BOARD_SIZE = BOARD[1][0] - BOARD[0][0], BOARD[1][1] - BOARD[0][1]
            CELL_SIZE = tuple(b / 8 for b in BOARD_SIZE)
            top_left = BOARD[0][0] + c * CELL_SIZE[0], BOARD[0][1] + r * CELL_SIZE[1]
            bottom_right = top_left[0] + CELL_SIZE[0], top_left[1] + CELL_SIZE[1]
 
            top_left = top_left[0] + CELL_SIZE[0] / 4, top_left[1] + CELL_SIZE[1] / 4
            bottom_right = bottom_right[0] - CELL_SIZE[0] / 4, bottom_right[1] - CELL_SIZE[1] / 4
 
            top_left = tuple(map(int, top_left))
            bottom_right = tuple(map(int, bottom_right))
 
            detection_zone = frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], :]
 
            high_green = np.array([104, 153, 60])
            low_green = np.array([10, 30, 0])
 
            # detection_zone_hsv = cv2.cvtColor(detection_zone, cv2.COLOR_BGR2HSV)
            # cell_mask = cv2.inRange(detection_zone_hsv, low_green, high_green)
            cell_mask = mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
 
            is_piece_present = np.mean(cv2.mean(cell_mask)) < 20
 
            cell_color_mask = color_mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
            is_piece_white = np.mean(cell_color_mask) > 24
 
            if is_piece_present:
                frame = cv2.rectangle(frame, top_left, bottom_right, color=(0, 0, 255) if is_piece_white else (255, 0, 0))
            # else:
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 0] = cell_color_mask
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 1] = cell_color_mask
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 2] = cell_color_mask
 
 
    # circles = cv2.HoughCircles(img_cny_mono, cv2.HOUGH_GRADIENT, 1, 10, param1=30, param2=50, minRadius=10, maxRadius=50)
 
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0,:]:
    #         cv2.circle(img_cny, (i[0], i[1]), i[2], (0, 255, 0), 2)
 
    plt.subplot(121),plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(color_mask, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
 
    plt.draw()
    plt.pause(0.001)
 
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(50)
    if key == 27: # exit on ESC
        break
 
 
 
cv2.destroyWindow("preview")
vc.release()
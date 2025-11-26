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
    BOARD = np.array([[142, 75], [478, 418]])
    BOARD_SIZE = BOARD[1] - BOARD[0]
    CELL_SIZE = BOARD_SIZE / 8
 
    board = frame[BOARD[0][1]:BOARD[1][1], BOARD[0][0]:BOARD[1][0], :]
 
    frame = cv2.rectangle(frame, BOARD[0], BOARD[1], color=(0, 0, 255))

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(frame_hsv, (57, 93, 68), (96, 255, 197)) # green screen
    # color_mask = cv2.inRange(frame_hsv, (0, 0, 0), (180, 120, 54)) # dark pieces
 
    for c in range(8):
        for r in range(8):
            top_left = BOARD[0] + np.multiply(np.array([c, r]), CELL_SIZE)
            bottom_right = top_left + CELL_SIZE
 
            # Shrink every cell to the center of each cell
            top_left = top_left + CELL_SIZE // 4
            bottom_right = bottom_right - CELL_SIZE // 4
 
            detection_zone = frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], :]
 
            detection_zone_hsv = cv2.cvtColor(detection_zone, cv2.COLOR_BGR2HSV)
            # green screen
            cell_mask = cv2.inRange(detection_zone_hsv, (57, 93, 68), (96, 255, 197))
            # black screen
            cell_color_mask = cv2.inRange(frame_hsv, (0, 0, 0), (180, 120, 54))

            # cell_mask = mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
            # cell_color_mask = color_mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
 
            is_piece_present = np.mean(cv2.mean(cell_mask)) < 20 
            is_piece_white = np.mean(cell_color_mask) > 24
 
            if is_piece_present:
                c = (0, 0, 255) if is_piece_white else (255, 0, 0)
                frame = cv2.rectangle(frame, top_left, bottom_right, color=c)
            # else:
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 0] = cell_color_mask
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 1] = cell_color_mask
                # frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], 2] = cell_color_mask
 
 
    # plt.subplot(121),plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(color_mask, cmap='gray')
    # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
 
    # plt.draw()
    # plt.pause(0.001)
 
    cv2.imshow("preview", frame)
    rval = vc.read(frame)
    key = cv2.waitKey(50)
    if key == 27: # exit on ESC
        break
 
 
 
cv2.destroyWindow("preview")
vc.release()
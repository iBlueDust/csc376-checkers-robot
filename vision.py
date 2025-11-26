import cv2
import matplotlib.pyplot as plt
import numpy as np

from board import Board, BLACK, WHITE, EMPTY
from draughts import Board as DraughtsBoard


class Vision:
    rval: bool
    
    def __init__(self, device_index: int = 0):
        cv2.namedWindow("preview")
        self.vc = cv2.VideoCapture(device_index)
 
        if self.vc.isOpened(): # try to get the first frame
            self.rval, self.frame = self.vc.read()
        else:
            self.rval = False
 
 
    def __enter__(self):
        return self
    
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.vc.release()
        cv2.destroyWindow("preview")
        
 
    def get_frame(self) -> tuple[bool, cv2.Mat]:
        rval, _ = self.vc.read(self.frame)
        return rval, self.frame
    
    
    def get_game_board(self) -> Board:
        BOARD = np.array([[142, 75], [478, 418]])
        BOARD_SIZE = BOARD[1] - BOARD[0]
        CELL_SIZE = BOARD_SIZE / 8
    
        rval, _ = self.vc.read(self.frame)
        frame = self.frame.copy()
    
        frame = cv2.rectangle(frame, BOARD[0], BOARD[1], color=(0, 0, 255))
        # frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(frame_hsv, (57, 93, 68), (96, 255, 197)) # green screen
        # color_mask = cv2.inRange(frame_hsv, (0, 0, 0), (180, 120, 54)) # dark pieces
        
        # Build piece lists for FEN
        white_pieces = []
        black_pieces = []
        
        for c in range(8):
            for r in range(8):
                top_left = BOARD[0] + np.multiply(np.array([c, r]), CELL_SIZE)
                bottom_right = top_left + CELL_SIZE
    
                # Shrink every cell to the center of each cell
                top_left = top_left + CELL_SIZE // 4
                bottom_right = bottom_right - CELL_SIZE // 4
    
                detection_zone = frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], :]
                cv2.resize(
                    detection_zone, 
                    (detection_zone.shape[0] // 2, detection_zone.shape[1] // 2), 
                    dst=detection_zone
                )
    
                detection_zone_hsv = cv2.cvtColor(detection_zone, cv2.COLOR_BGR2HSV)
                # green screen
                cell_mask = cv2.inRange(detection_zone_hsv, (57, 93, 68), (96, 255, 197))
                # black screen
                cell_color_mask = cv2.inRange(detection_zone_hsv, (0, 0, 0), (180, 120, 54))

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
                    # Only dark squares have pieces (row + col is odd)
                    if (r + c) % 2 == 1:
                        # Calculate square number (1-32)
                        sq = (r * 4) + (c // 2) + 1
                        if is_piece_white:
                            white_pieces.append(str(sq))
                        else:
                            black_pieces.append(str(sq))
    
        # plt.subplot(121),plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        # plt.subplot(122),plt.imshow(color_mask, cmap='gray')
        # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    
        # plt.draw()
        # plt.pause(0.001)
    
        cv2.imshow("preview", frame)
        # key = cv2.waitKey(50)
        # if key == 27: # exit on ESC
        #     break
        
        # Build FEN string
        white_str = ",".join(white_pieces) if white_pieces else ""
        black_str = ",".join(black_pieces) if black_pieces else ""
        fen = f"W:W{white_str}:B{black_str}"
        
        # Create and return DraughtsBoard from FEN
        return DraughtsBoard(variant="american", fen=fen)
 
 
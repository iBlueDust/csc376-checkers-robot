from robot import Robot
from vision import Vision
from board import Board, BLACK, WHITE, EMPTY
from draughts import Board as DraughtsBoard
from ai_vs_ai import main as run_ai_vs_ai
import minmax
import numpy as np
import cv2
import time
import re


def extract_black_white(fen):
    try:
        if "W:W" in fen:
            white_section = re.search(r"W:W([^:]+):B", fen).group(1)
            black_section = re.search(r":B(.+)", fen).group(1)
            white_nums = re.findall(r"[A-Za-z]*?(\d+)", white_section)
            black_nums = re.findall(r"[A-Za-z]*?(\d+)", black_section)
            white_str = ",".join(white_nums)
            black_str = ",".join(black_nums)
        elif "B:W" in fen: 
            white_section = re.search(r"B:W([^:]+):B", fen).group(1)
            black_section = re.search(r":B(.+)", fen).group(1)
            white_nums = re.findall(r"[A-Za-z]*?(\d+)", white_section)
            black_nums = re.findall(r"[A-Za-z]*?(\d+)", black_section)
            white_str = ",".join(white_nums)
            black_str = ",".join(black_nums)
        print("WHITE", white_str)
        print("BLACK", black_str)
        return black_str, white_str
    except Exception as e:
        print(f'extract_black_white: {e}')

def boards_differ(board1: Board, board2: Board) -> bool:
    print("board1 fn",board1.fen)
    print("board2.fn", board2.fen)
    black1, white1 = extract_black_white(board1.fen) or (None, None)
    black2, white2 = extract_black_white(board2.fen) or (None, None)
    return black1 != black2 or white1 != white2

def validate_move(board1: Board, board2: Board) -> bool:
    # Convert vision boards to FEN
 
    for move in board1.legal_moves():
        board1.push(move)
 
        if not boards_differ(board1, board2):
            return True
        board1.pop()
 
    return False
 
 
def run_human_vs_ai(robot: Robot, cam: Vision, human_color=WHITE):
    """Human vs AI game loop."""
    print("=== Human vs AI Mode ===")
 
    ai_color = BLACK
 
    print("Detecting initial board state...")
    initial_vision = cam.get_game_board()
    print("Initial board:")
    print(initial_vision)
 
    # Start with standard position or detected position
    board = initial_vision # DraughtsBoard(variant="american", fen="startpos")
 
    prev_vision = board
 
    while not board.is_over():
        if board.turn == human_color:
            # Human's turn - wait for them to make a move
            print("\n--- Your turn! Make a move on the board ---")
            print("Press 'c' to confirm your move, 'q' to quit")
 
            print()
            while True:
                # Get current board state from camera
                robot.move_home()

                t_start = time.time()
                current_vision = cam.get_game_board()
                t_end = time.time()
                t_elapsed = t_end - t_start
                print(f'Vision took {t_elapsed * 1000.0:.02f} ms', end='\r')
 
                # Check for key press
                key = cv2.waitKey(100) & 0xFF
 
                if key == ord('q'):
                    print("Game quit by user.")
                    return
 
                elif key == ord('c'):                    
                    print('Board state:')
                    print(current_vision)
                    # User confirms move
                    if boards_differ(prev_vision, current_vision):
                        print("board is different\n now validate move")
                        if validate_move(prev_vision, current_vision):
                            print("After player move")
                            print(prev_vision)
                            board = prev_vision
                            print("Move detected and confirmed!")
                            print("Player made a move!")
                            break
                        else:
                            print("Invalid move detected. Try again!")
                    else:
                        print("No change detected. Player has not made a move yet!")
 
        else:
            # AI's turn
            print(f"\nAI thinking...")
            best = minmax.find_best_move(board, depth=4, color=ai_color)
 
            if not best:
                print("AI has no legal moves.")
                break
 
            print(f"AI plays: {best.pdn_move}")
            board.push(best)
 
            # Execute move on physical board
            robot.exec_player_move(best, color=ai_color)
 
            # Handle captures
            if best.has_captures:
                print(f"Captured pieces: {best.captures}")
                for cap in best.captures:
                    src = robot._cell_id_to_coord(cap)
                    robot.discard_piece(src)
 
            robot.move_home()
            time.sleep(2)  # Wait for robot to finish
            prev_vision = board
 
    # Game over
    winner = board.winner()
    if winner == 0:
        print("\n=== Game Over: Draw! ===")
    elif winner == human_color:
        print("\n=== Game Over: You win! ===")
    else:
        print("\n=== Game Over: AI wins! ===")
 
 
def main():
    print("╔════════════════════════════════════╗")
    print("║     CSC376 Checkers Robot Game     ║")
    print("╠════════════════════════════════════╣")
    print("║  1. AI vs AI                       ║")
    print("║  2. Human vs AI (Human = WHITE)    ║")
    print("╚════════════════════════════════════╝")
 
    choice = input("Select mode (1/2/3): ").strip()
 
    if choice == "1":
        # AI vs AI has its own Robot/Vision context
        run_ai_vs_ai()
    elif choice == "2":
        print('Running 2. Human vs AI')
        with Robot() as robot, Vision(4) as cam:
            robot.move_home()
            run_human_vs_ai(robot, cam, human_color=WHITE)
            robot.move_home()
    else:
        print("Invalid choice. Defaulting to AI vs AI.")
        run_ai_vs_ai()
 
 
if __name__ == "__main__":
    main()
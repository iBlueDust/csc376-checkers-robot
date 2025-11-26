from robot import Robot
from vision import Vision
from board import Board, BLACK, WHITE, EMPTY
from draughts import Board as DraughtsBoard
from ai_vs_ai import main as run_ai_vs_ai
import minmax
import numpy as np
import cv2
import time


def boards_differ(board1: Board, board2: Board) -> bool:
    return board1.fen != board2.fen

def validate_move(board1: Board, board2: Board) -> bool:
    # Convert vision boards to FEN
    result2_fen = board2.fen
    
    for move in board1.legal_moves():
        board1.push(move)
        
        result1_fen = board1.fen
        if result1_fen == result2_fen:
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
    board = DraughtsBoard(variant="american", fen="startpos")
    
    prev_vision = initial_vision
    
    while not board.is_over():
        if board.turn == human_color:
            # Human's turn - wait for them to make a move
            print("\n--- Your turn! Make a move on the board ---")
            print("Press 'c' to confirm your move, 'q' to quit")
            
            while True:
                # Get current board state from camera
                current_vision = cam.get_game_board()
                
                # Check for key press
                key = cv2.waitKey(100) & 0xFF
                
                if key == ord('q'):
                    print("Game quit by user.")
                    return
                
                elif key == ord('c'):
                    # User confirms move
                    if boards_differ(prev_vision, current_vision):
                        if validate_move(prev_vision, current_vision):
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
            best = minmax.find_best_move(board, depth=3, color=ai_color)
            
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
            prev_vision = cam.get_game_board()
    
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
        with Robot() as robot, Vision(4) as cam:
            robot.move_home()
            run_human_vs_ai(robot, cam, human_color=WHITE)
            robot.move_home()
    else:
        print("Invalid choice. Defaulting to AI vs AI.")
        run_ai_vs_ai()


if __name__ == "__main__":
    main()
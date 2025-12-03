from robot import Robot
from vision import Vision
from draughts import Board, Move, BLACK, WHITE

import minmax

def main():
    with Robot(visualizer=False) as robot, Vision(4) as cam:
        robot.move_home()

        # Play loop
        board = Board(variant="american", fen="startpos")
        ai1_color = WHITE
        ai2_color = BLACK
        
        def exec_ai_move(id: int, color) -> bool:
            print(f"AI{id} thinking...")
            best = minmax.find_best_move(board, depth=5, color=color)
            if not best:
                print("No legal move.")
                return False
            print(f"AI plays: {best.pdn_move}")
            board.push(best)

            robot.exec_player_move(best)

            if best.has_captures:
                print("captured")
                for cap in best.captures:
                    src = robot._cell_id_to_coord(cap)
                    robot.discard_piece(src)
            return True

        while not board.is_over():
            if board.turn == ai1_color:
                if not exec_ai_move(1, color=ai1_color):
                    break
            else:
                if not exec_ai_move(2, color=ai2_color):
                    break

        winner = board.winner()
        if winner == 0:
            print("Draw!")
        elif winner == ai1_color:
            print("AI1 wins!")
        else:
            print("AI2 wins!")


if __name__ == "__main__":
    main()
from draughts import Board, Move, WHITE, BLACK


# Count white/black pieces & kings
def evaluate(board: Board) -> int:
    """
    Positive if WHITE ahead, negative if BLACK ahead.
    """
    fen = board.fen
    white_pieces = fen.count("w")
    black_pieces = fen.count("b")
    white_kings = fen.count("W")
    black_kings = fen.count("B")

    material = (white_pieces - black_pieces) + 3 * (white_kings - black_kings)
    return material


# Minimax with α–β pruning
def minimax(board, depth, alpha, beta, maximizing_color):
    if depth == 0 or board.is_over():
        return evaluate(board)

    moves = board.legal_moves()
    if maximizing_color == WHITE:
        max_eval = -float("inf")
        for move in moves:
            board.push(move)
            eval_val = minimax(board, depth - 1, alpha, beta, BLACK)
            board.pop()
            max_eval = max(max_eval, eval_val)
            alpha = max(alpha, eval_val)
            if beta <= alpha:
                break
        return max_eval
    else:
        min_eval = float("inf")
        for move in moves:
            board.push(move)
            eval_val = minimax(board, depth - 1, alpha, beta, WHITE)
            board.pop()
            min_eval = min(min_eval, eval_val)
            beta = min(beta, eval_val)
            if beta <= alpha:
                break
        return min_eval


def find_best_move(board: Board, depth: int, color: int) -> Move:
    best_move = None
    best_value = -float("inf") if color == WHITE else float("inf")

    for move in board.legal_moves():
        board.push(move)
        value = minimax(board, depth - 1, -float("inf"), float("inf"),
                        WHITE if color == BLACK else BLACK)
        board.pop()

        if color == WHITE and value > best_value:
            best_value = value
            best_move = move
        elif color == BLACK and value < best_value:
            best_value = value
            best_move = move

    return best_move


# Play loop
board = Board(variant="american", fen="startpos")
human_color = WHITE
ai_color = BLACK

while not board.is_over():
    print(board)

    if board.turn == human_color:
        moves = board.legal_moves()
        print("Your legal moves:", [m.pdn_move for m in moves])
        mv_str = input("Enter your move: ").strip()
        try:
            mv = Move(board, pdn_move=mv_str)
            board.push(mv)
        except Exception as e:
            print("Invalid move:", e)
            continue
    else:
        print("AI thinking...")
        best = find_best_move(board, depth=3, color=ai_color)
        if not best:
            print("No legal move.")
            break
        print(f"AI plays: {best.pdn_move}")
        board.push(best)

winner = board.winner()
if winner == 0:
    print("Draw!")
elif winner == human_color:
    print("You win!")
else:
    print("AI wins!")

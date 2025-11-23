from draughts import Board, Move, WHITE, BLACK
# from webcam import move_piece

human_color = WHITE
ai_color = BLACK

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


def num_to_coord(n: int) -> tuple[int, int]:
    """
    Convert a square number (1‑32) into 0‑based coordinates (x, y)
    so that 0,0 corresponds to the top‑left dark square.
    """
    row = (n - 1) // 4
    col_in_row = (n - 1) % 4
    # on even rows dark squares start at column 1, on odd rows at 0
    col = col_in_row * 2 + ((row + 1) % 2)
    return col, row


def see_move_robot():
    """
    Build and return a FEN string representing the current board.
    """
    # Implement actual detection code
    return "W:W19,23,25,K30:B12,15,K22"


def move_robot(move: Move):
    steps = move.steps_move
    if not steps or len(steps) < 2:
        return

    # handle multi‑jump captures
    for i in range(len(steps) - 1):
        src_sq = steps[i]
        dst_sq = steps[i + 1]
        src = num_to_coord(src_sq)
        dst = num_to_coord(dst_sq)
        print(f"Robot moving piece from {src} -> {dst}")
        # move_piece(src, dst)


def remove_piece_robot(move: Move):
    # Move somewhere outside of board
    # move_piece(15, 15)
    pass


def play_move():
    custom_fen = see_move_robot()
    board = Board(variant="american", fen=custom_fen)

    print(board)

    print("AI thinking...")
    best = find_best_move(board, depth=3, color=ai_color)
    if not best:
        print("No legal move.")
        return board.winner()
    print(f"AI plays: {best.pdn_move}")
    board.push(best)

    move_robot(best)

    if best.has_captures:
        print("captured")
        remove_piece_robot(best)

    return board.winner() if board.is_over() else None


if __name__ == "__main__":
    # Play loop
    winner = -1
    while True:
        outcome = play_move()
        if outcome is not None:
            winner = outcome
            break

        # Wait for player move

    if winner == 0:
        print("Draw!")
    elif winner == human_color:
        print("You win!")
    else:
        print("AI wins!")

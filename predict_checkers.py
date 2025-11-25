import cv2
import numpy as np
import tensorflow as tf
import sys
# use model from: https://github.com/dimasikson/chess-board-detector.git
# clone this first
#TODO: crop the image to just the board
#TODO: make a function to compare the previous state/fen and the current state/fen to assign kings


# Add the chess-board-detector path for imports
sys.path.append('chess-board-detector/fen_chess_data')

from utils import split_chessboard  

# Paths
MODEL_PATH = 'chess-board-detector/fen_chess_data/models/model_best.h5'


def load_model():
    """Load the pre-trained chess model."""
    model = tf.keras.models.load_model(MODEL_PATH)
    return model


def chess_to_checkers_class(chess_class):
    """
    Map chess model output (13 classes) to checkers classes (3 classes).
    
    Chess classes: 0=empty, 1-6=white pieces (PRNBQK), 7-12=black pieces (prnbqk)
    Checkers classes: 0=empty, 1=white, 2=black
    """
    if chess_class == 0:
        return 0  # empty
    elif 1 <= chess_class <= 6:
        return 1  # white piece
    else:  # 7-12
        return 2  # black piece


def preds_to_checkers_fen(preds, turn='B'):
    """
    Convert chess model predictions to checkers FEN format.
    
    Remaps 13-class chess output to checkers:
    - 0 (empty) -> empty
    - 1-6 (white chess pieces) -> white checker
    - 7-12 (black chess pieces) -> black checker
    
    Output format: "B:W19,23,25:B12,15,22"
    """
    white_pieces = []
    black_pieces = []
    
    for i, p in enumerate(preds):
        chess_cls = np.argmax(p)
        cls = chess_to_checkers_class(chess_cls)
        
        if cls == 0:  # empty
            continue
        
        # Convert square index (0-63) to checkers position (1-32)
        # Checkers only uses dark squares
        row = i // 8
        col = i % 8
        
        # Check if it's a dark square (valid checkers square)
        if (row + col) % 2 == 0:
            continue  # Light square, skip
        
        # Calculate checkers position number (1-32)
        col_in_row = col // 2
        position = row * 4 + col_in_row + 1
        
        if position < 1 or position > 32:
            continue
        
        if cls == 1:  # white piece
            white_pieces.append(str(position))
        elif cls == 2:  # black piece
            black_pieces.append(str(position))
    
    # Build FEN string
    white_str = ",".join(white_pieces) if white_pieces else ""
    black_str = ",".join(black_pieces) if black_pieces else ""
    
    return f"{turn}:W{white_str}:B{black_str}"


def preprocess_board_image(image, target_size=256):
    """
    Preprocess the board image.
    Assumes image is already cropped to just the board.
    """
    # Resize to expected size
    image = cv2.resize(image, (target_size, target_size))
    return image


def predict_board(model, board_image, square_size=32):
    """
    Predict pieces on the board.
    
    Args:
        model: Loaded keras model
        board_image: Image of the board (should be 256x256 or will be resized)
        square_size: Size to resize each square (model expects 32x32)
    
    Returns:
        Checkers FEN string
    """
    # Preprocess
    board_image = preprocess_board_image(board_image, target_size=256)
    
    # Split into 64 squares
    squares = split_chessboard(board_image)
    
    # Resize each square to model input size and normalize
    processed_squares = []
    for sq in squares:
        sq = cv2.resize(sq, (square_size, square_size))
        sq = np.float32(sq) / 255.0  # Normalize
        processed_squares.append(sq)
    
    processed_squares = np.array(processed_squares)
    
    # Predict
    predictions = model.predict(processed_squares, verbose=0)
    
    # Convert to checkers FEN
    fen = preds_to_checkers_fen(predictions, turn='B')
    
    return fen, predictions



if __name__ == "__main__":
    image_path = "sample.jpg"
    model = load_model()

    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not load image from {image_path}")
        sys.exit(1)
    
    fen, predictions = predict_board(model, image)
    print(f"\nPredicted FEN: {fen}")
    


import numpy as np
from draughts import BLACK, WHITE
from typing import Literal, overload

EMPTY = 0
assert BLACK != EMPTY, "pydraughts BLACK constant conflicts with EMPTY"
assert WHITE != EMPTY, "pydraughts WHITE constant conflicts with EMPTY"
    
type BoardCell = Literal[0, 1, 2]

class Board:
    def __init__(self):
        self._state = np.array([[EMPTY] * 8] * 8) # 8x8 EMPTY board
        
    def print_cell(self, cell: BoardCell) -> str:
        if cell == EMPTY:
            return '.'
        elif cell == BLACK:
            return 'b'
        elif cell == WHITE:
            return 'w'
        else:
            raise ValueError("Invalid cell value")
    
    def __str__(self):
        display = ""
        for row in self._state:
            display += " ".join(map(self.print_cell, row)) + "\n"
        return display
    
    def __getitem__(self, key) -> BoardCell | list[BoardCell]:
        return self._state[key]
        
    def __setitem__(self, key, value) -> None:
        self._state[key] = value
        
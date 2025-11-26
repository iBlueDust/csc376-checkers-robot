from robot import Robot
from vision import Vision

def main():
    with Robot() as robot, Vision(4) as cam:
        robot.move_home()
        # move_to_all_corners()
        for i in range(1, 7, 2):
            robot.move_piece((0, i), (0, -3))
        robot.move_home()
        
        board_state = cam.get_game_board()
    
    
if __name__ == "__main__":
    main()
from robot import Robot

def main():
    with Robot(visualizer=False) as robot:
        robot.move_home()
        for i in range(1, 8, 2):
            robot.discard_piece((0, i))


if __name__ == '__main__':
    main()
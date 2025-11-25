import cv2
import numpy as np
import roboticstoolbox as rtb
from csc376_franky.vizualizer import RtbVisualizer
from csc376_franky.motion_generator import RuckigMotionGenerator
from spatialmath import SE3

import csc376_bind_franky

import minmax
from draughts import Board, Move, BLACK


def main():
    np.set_printoptions(precision=4, suppress=True,)

    # I. Speed factor settings
    relative_vel_factor = 0.02
    relative_acc_factor = 0.02
    relative_jerk_factor = 0.04

    # II. RTB, Ruckig, csc376_franky, and Visualizer setup
    panda_rtb_model = rtb.models.Panda()
    motion_generator = RuckigMotionGenerator()

    csc376_franky_robot = csc376_bind_franky.FrankaJointTrajectoryController("192.168.1.107")
    csc376_franky_robot.setupSignalHandler()
    csc376_gripper = csc376_bind_franky.Gripper("192.168.1.107")

    q_start = csc376_franky_robot.get_current_joint_positions()
    visualizer = RtbVisualizer(panda_rtb_model, q_start)


    def move(target: SE3):
        q_start = csc376_franky_robot.get_current_joint_positions()

        # III. Calculate your goal
        se3_start   = panda_rtb_model.fkine(q_start)
        # se3_target = SE3.Ty(-0.10) * se3_start # Relative to start position, pre-multiply for world frame reference
        # se3_target = panda_rtb_model.fkine(q_end)
        se3_target = target
        print("q_start", q_start)
        print("se3_start", se3_start)
        print("se3_target", se3_target)

        # IV. Visualize the trajectory in simulation
        cartesian_traj, dt = motion_generator.calculate_cartesian_pose_trajectory(se3_start, se3_target,
                                                                                relative_vel_factor, relative_acc_factor, relative_jerk_factor) # Interpolation and trajectory parameterization
        print('dt', dt)

        print('Enter API')
        q_traj = motion_generator.cartesian_pose_to_joint_trajectory(panda_rtb_model, q_start, cartesian_traj)
        print('Exit API')
        # input("Press enter, to run in visualizer\n")
        # visualizer.move_gripper(0.07, 0.1)
        # visualizer.run_joint_trajectory(q_traj, dt)

        # V. Run on real robot
        # yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
        # if yes_or_else != "yes":
        #     print("User did not type [yes], will not run on real robot")
        #     return visualizer

        try:
            csc376_franky_robot.run_joint_trajectory(q_traj, dt)
        except Exception as e:
            print(e)

    def move_q(q_end: list[float]):
        q_start = csc376_franky_robot.get_current_joint_positions()
        q_start = np.array(q_start)
        q_end = np.array(q_end)

        GRANULARITY = 60

        q_traj = [q_start]

        for i in range(GRANULARITY):
            s = i / GRANULARITY
            s = 3 * (s ** 2) - 2 * (s ** 3)
            q_step = q_start + (q_end - q_start) * s
            q_traj.append(q_step)

        q_traj.append(q_end)

        dt = 2 / GRANULARITY
        try:
            csc376_franky_robot.run_joint_trajectory(q_traj, dt)
        except Exception as e:
            print(e)

    def grip(q: float):
        # visualizer.move_gripper(q, 0.1)
        print('Enter gripper')
        csc376_gripper.move(q, 0.1)
        print('Exit gripper')


    CAMERA_ORIGIN = SE3.Trans(0.4025, -0.066, 0)
    BOARD_ORIGIN = SE3.Trans(0.480, 0, 0)
    BOARD_SIZE = 0.32
    LOW_FLOOR = -0.000
    MID_FLOOR = 0.05
    HIGH_FLOOR = 0.4

    def move_piece(src: tuple[int, int], dest: tuple[int, int]):
        BOARD_CORNER = BOARD_ORIGIN * SE3.Trans(-BOARD_SIZE / 2, -BOARD_SIZE / 2, 0)
        src_pos = BOARD_CORNER * SE3(BOARD_SIZE * src[0] / 8.0, BOARD_SIZE * src[1] / 8.0, 0)
        dest_pos = BOARD_CORNER * SE3(BOARD_SIZE * dest[0] / 8.0, BOARD_SIZE * dest[1] / 8.0, 0)

        grip(0.05)
        move(src_pos * SE3.Tz(MID_FLOOR) * SE3.Rx(np.pi))
        move(src_pos * SE3.Tz(LOW_FLOOR) * SE3.Rx(np.pi))
        grip(0.0305)
        move(src_pos * SE3.Tz(MID_FLOOR) * SE3.Rx(np.pi))
        move(dest_pos * SE3.Tz(MID_FLOOR) * SE3.Rx(np.pi))
        move(dest_pos * SE3.Tz(LOW_FLOOR) * SE3.Rx(np.pi))
        grip(0.05)
        move(dest_pos * SE3.Tz(MID_FLOOR) * SE3.Rx(np.pi))

    def move_to_all_corners():
        BOARD_CORNER = BOARD_ORIGIN * SE3.Trans(-BOARD_SIZE / 2, -BOARD_SIZE / 2, 0)
        corners = [
            BOARD_CORNER * SE3(BOARD_SIZE * 0 / 8.0, BOARD_SIZE * 0 / 8.0, 0),
            BOARD_CORNER * SE3(BOARD_SIZE * 7 / 8.0, BOARD_SIZE * 0 / 8.0, 0),
            BOARD_CORNER * SE3(BOARD_SIZE * 7 / 8.0, BOARD_SIZE * 7 / 8.0, 0),
            BOARD_CORNER * SE3(BOARD_SIZE * 0 / 8.0, BOARD_SIZE * 7 / 8.0, 0),
        ]

        grip(0.05)
        for corner in corners:
            move(corner * SE3.Tz(MID_FLOOR) * SE3.Rx(np.pi))

    def move_home():
        grip(0.05)
        move_q([-0.010096422112703594, -0.4704397389824306, -0.1172932928857341, -2.1056523873364483, -0.05548446332414944, 1.6359151515430872, 0.6788236314586359])
        move(CAMERA_ORIGIN * SE3.Tz(HIGH_FLOOR) * SE3.Rx(np.pi))

    def num_to_coord(n: int) -> tuple[int, int]:
        row = (n - 1) // 4
        col_in_row = (n - 1) % 4
        # on even rows dark squares start at column 1, on odd rows at 0
        col = col_in_row * 2 + ((row + 1) % 2)
        return col, row


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
            move_piece(src, dst)

    move_home()
    # move_to_all_corners()
    move_piece((7, 0), (4, 7))
    move_piece((2, 5), (3, 6))
    move_home()

    # Play loop
    while True:
        # custom_fen = see_move_robot()
        custom_fen = "B:W19,23,25,K30:B12,15,K22"
        board = Board(variant="american", fen=custom_fen)

        print(board)

        print("AI thinking...")
        best = minmax.find_best_move(board, depth=3, color=BLACK)
        if not best:
            print("No legal move.")
            return board.winner()
        print(f"AI plays: {best.pdn_move}")
        board.push(best)

        move_robot(best)

        if best.has_captures:
            print("captured")
            move_piece(15, 15)

        if board.is_over():
            winner = board.winner()
            break

    # Take picture
    cv2.namedWindow("preview")
    vc = cv2.VideoCapture(4)

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    while rval:
        frame = cv2.rectangle(frame, (100, 100), (200, 200), color=(0, 0, 255))
        # board =

        cv2.imshow("preview", frame)
        rval, frame = vc.read()
        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break

    cv2.destroyWindow("preview")
    vc.release()

    return visualizer


if __name__ == "__main__":
    visualizer = main()
    visualizer.stop() # Makes sure render thread ends

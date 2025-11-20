import cv2
import numpy as np
import roboticstoolbox as rtb
from csc376_franky.vizualizer import RtbVisualizer
from csc376_franky.motion_generator import RuckigMotionGenerator
from spatialmath import SE3

import csc376_bind_franky


def main():
    np.set_printoptions(precision=4, suppress=True,)    

    # I. Speed factor settings
    relative_vel_factor = 0.03
    relative_acc_factor = 0.03
    relative_jerk_factor = 0.05

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
        q_traj = motion_generator.cartesian_pose_to_joint_trajectory(panda_rtb_model, q_start, cartesian_traj)
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

    def grip(q: float):
        # visualizer.move_gripper(q, 0.1)
        csc376_gripper.move(q, 0.1)
    

    def move_piece(src: tuple[int, int], dest: tuple[int, int]):
        BOARD_ORIGIN = SE3.Trans(0.45, 0, 0)
        BOARD_SIZE = 0.5
        src_pos = BOARD_ORIGIN * SE3(BOARD_SIZE * src[0] / 8.0, BOARD_SIZE * src[1] / 8.0, 0)
        dest_pos = BOARD_ORIGIN * SE3(BOARD_SIZE * dest[0] / 8.0, BOARD_SIZE * dest[1] / 8.0, 0)

        grip(0.1)
        move(src_pos * SE3.Tz(0.25) * SE3.Rx(np.pi))
        move(src_pos * SE3.Tz(0.015) * SE3.Rx(np.pi))
        grip(0.05)
        move(src_pos * SE3.Tz(0.25) * SE3.Rx(np.pi))
        move(dest_pos * SE3.Tz(0.25) * SE3.Rx(np.pi))
        move(dest_pos * SE3.Tz(0.015) * SE3.Rx(np.pi))
        grip(0.1)
        move(dest_pos * SE3.Tz(0.25) * SE3.Rx(np.pi))
        move(BOARD_ORIGIN * SE3.Tz(0.25) * SE3.Rx(np.pi))

    move_piece((0, 0), (1, 1))
    move_piece((1, 1), (2, 0))

    # Take picture
    cv2.namedWindow("preview")
    vc = cv2.VideoCapture(5)

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    while rval:
        frame = cv2.rectangle(frame, (100, 100), (200, 200), color=(0, 0, 255))

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
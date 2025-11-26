import numpy as np
import roboticstoolbox as rtb
from csc376_franky.vizualizer import RtbVisualizer
from csc376_franky.motion_generator import RuckigMotionGenerator
from spatialmath import SE3
import csc376_bind_franky

 
 
class Robot:
    '''
    Robot class to manipulate pieces on game board
    
    Usage:
    with Robot() as robot:
        robot.move_piece((0, 0), (1, 1))
        robot.move_home()
        robot.grip_open()
    '''
    # I. Speed factor settings
    relative_vel_factor = 0.02
    relative_acc_factor = 0.02
    relative_jerk_factor = 0.04
    
    camera_origin = SE3.Trans(0.4275, -0.066, 0) 
    board_origin = SE3.Trans(0.505, 0, 0)
    board_size = 0.32
    low_floor = 0 + 0.022
    mid_floor = 0.05 + 0.022
    high_floor = 0.5 + 0.022
    
    grip_open_q = 0.05
    grip_closed_q = 0.0305

    def __init__(self):   
        # II. RTB, Ruckig, csc376_franky, and Visualizer setup
        self.sim_model = rtb.models.Panda()
        self.motion_generator = RuckigMotionGenerator()
    
        self.real_robot = csc376_bind_franky.FrankaJointTrajectoryController(
            "192.168.1.107")
        self.real_robot.setupSignalHandler()
        self.real_gripper = csc376_bind_franky.Gripper("192.168.1.107")
    
        q_start = self.real_robot.get_current_joint_positions()
        self.visualizer = RtbVisualizer(self.sim_model, q_start)
 
    def move_ik(self, target: SE3):
        '''Move robot to target SE3 pose using inverse kinematics'''
        q_start = self.real_robot.get_current_joint_positions()
 
        # III. Calculate your goal
        se3_start    = self.sim_model.fkine(q_start)
        # se3_target = SE3.Ty(-0.10) * se3_start # Relative to start position, pre-multiply for world frame reference
        # se3_target = self.sim_model.fkine(q_end)
        se3_target = target
        print("q_start", q_start)
        print("se3_start", se3_start)
        print("se3_target", se3_target)
 
        # IV. Visualize the trajectory in simulation
        cartesian_traj, dt = self.motion_generator \
            .calculate_cartesian_pose_trajectory(
                se3_start, 
                se3_target, 
                self.relative_vel_factor, 
                self.relative_acc_factor, 
                self.relative_jerk_factor
            ) # Interpolation and trajectory parameterization
 
        q_traj = self.motion_generator.cartesian_pose_to_joint_trajectory(
            self.sim_model, 
            q_start, 
            cartesian_traj
        )
        # input("Press enter, to run in visualizer\n")
        # visualizer.move_gripper(0.07, 0.1)
        # visualizer.run_joint_trajectory(q_traj, dt)
 
        # V. Run on real robot
        # yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
        # if yes_or_else != "yes":
        #     print("User did not type [yes], will not run on real robot")
        #     return visualizer
 
        try:
            self.real_robot.run_joint_trajectory(q_traj, dt)
        except Exception as e:
            print(e)
 
    def move_q(self, q_end: list[float], duration_sec=2.0, dt_sec=0.03):
        q_start = self.real_robot.get_current_joint_positions()
        q_start = np.array(q_start)
        q_end = np.array(q_end)
 
        n_steps = duration_sec // dt_sec
 
        q_traj = [q_start]
 
        for i in range(n_steps):
            s = i / n_steps
            s = 3 * (s ** 2) - 2 * (s ** 3) # Smoothstep interpolation
            q_step = q_start + (q_end - q_start) * s
            q_traj.append(q_step)
 
        q_traj.append(q_end)
 
        self.real_robot.run_joint_trajectory(q_traj, dt_sec)
        
    def get_pose(self) -> SE3:
        q_current = self.real_robot.get_current_joint_positions()
        return self.sim_model.fkine(q_current)
 
    def grip(self, q: float):
        # visualizer.move_gripper(q, 0.1)
        print('Enter gripper')
        self.real_gripper.move(q, 0.1)
        print('Exit gripper')

    def grip_open(self):
        self.grip(self.grip_open_q)
        
    def grip_close(self):
        self.grip(self.grip_closed_q)
 
    def move_piece(self,
                   src: tuple[int, int], 
                   dest: tuple[int, int], 
                   drop_at_height=False
                   ) -> None:
        board_corner = self.board_origin \
            * SE3.Trans(-self.board_size / 2, -self.board_size / 2, 0)
        src_pos = board_corner \
            * SE3.Tx(self.board_size * src[0] / 8.0) \
            * SE3.Ty(self.board_size * src[1] / 8.0)
        dest_pos = board_corner \
            * SE3.Tx(self.board_size * dest[0] / 8.0) \
            * SE3.Ty(self.board_size * dest[1] / 8.0)
 
        self.grip_open()
        self.move(src_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        self.move(src_pos * SE3.Tz(self.low_floor) * SE3.Rx(np.pi))
        self.grip_close()
        self.move(src_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        self.move(dest_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        if not drop_at_height: 
            self.move(dest_pos * SE3.Tz(self.low_floor) * SE3.Rx(np.pi))
        self.grip_open()
        if not drop_at_height: 
            self.move(dest_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
 
    def discard_piece(self, src: tuple[int, int]) -> None:
        dest = (1, -3)
        self.move_piece(src, dest, drop_at_height=True)
 
    def move_to_all_corners(self):
        board_corner = self.board_origin * SE3.Trans(-self.board_size / 2, -self.board_size / 2, 0)
        corners = [
            board_corner \
                * SE3(self.board_size * x / 8.0, self.board_size * y / 8.0, 0)
            for x, y in [(0,0), (7,0), (7,7), (0,7)]
        ]
 
        self.grip(0.05)
        for corner in corners:
            self.move(corner * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
 
    def move_home(self):
        self.grip_open()
        # This moves to about home
        self.move_q([-0.010096422112703594, -0.4704397389824306, -0.1172932928857341, -2.1056523873364483, -0.05548446332414944, 1.6359151515430872, 0.6788236314586359])
        
        current_pose = self.get_pose()
        if not np.allclose(current_pose.t, self.camera_origin.t, atol=1e-3):
            # Not quite there yet, lets use IK to fix
            self.move_ik(self.camera_origin * SE3.Tz(self.high_floor) * SE3.Rx(np.pi))
 
    def __enter__(self):
        return self
    
    def __exit__(self):
        self.visualizer.stop() # Makes sure render thread ends
 
 
if __name__ == "__main__":
    visualizer = main()
    visualizer.stop() # Makes sure render thread ends
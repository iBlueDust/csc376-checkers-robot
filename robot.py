import numpy as np
import roboticstoolbox as rtb
from csc376_franky.vizualizer import RtbVisualizer
from csc376_franky.motion_generator import RuckigMotionGenerator
from spatialmath import SE3
import csc376_bind_franky
from draughts import Move
from math import ceil

 
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
    relative_vel_factor = 0.035
    relative_acc_factor = 0.035
    relative_jerk_factor = 0.12
    
    camera_origin = SE3.Trans(0.4275, -0.066, 0) # only pertains information of the xy-coordinate of the camera
    board_origin = SE3.Trans(0.505, 0, 0) # 0.505 in the x-direction from the robot base
    board_size = 0.32 # size of the board in the world frame
    low_floor = 0 + 0.022 + 0.060
    mid_floor = 0.03 + 0.022 + 0.060
    high_floor = 0.5 + 0.022 + 0.060
    
    grip_open_q = 0.05
    grip_closed_q = 0.0305

    def __init__(self, visualizer=True, franka_url='192.168.1.107'):   
        if visualizer:
            import pymp

        # II. RTB, Ruckig, csc376_franky, and Visualizer setup
        self.sim_model = rtb.models.Panda()
        self.motion_generator = RuckigMotionGenerator()
    
        self.real_robot = csc376_bind_franky.FrankaJointTrajectoryController(franka_url)
        self.real_robot.setupSignalHandler()
        self.real_gripper = csc376_bind_franky.Gripper(franka_url)
    
        q_start = self.real_robot.get_current_joint_positions()
        self.visualizer = RtbVisualizer(self.sim_model, q_start) if visualizer else None
 
    def move_ik(self, target: SE3):
        '''Move robot to target SE3 pose using inverse kinematics'''
        q_start = self.real_robot.get_current_joint_positions()
 
        # III. Calculate your goal
        se3_start = self.sim_model.fkine(q_start)
        se3_target = target
 
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
 
        self.real_robot.run_joint_trajectory(q_traj, dt)
        # if self.visualizer is None: 
        #     # move robot directly
        #     self.real_robot.run_joint_trajectory(q_traj, dt)
        # else:
        #     # move robot and visualizer simultaneously
        #     with pymp.Parallel(2) as p:
        #         if p.thread_num == 0:
        #             self.real_robot.run_joint_trajectory(q_traj, dt)
        #         elif p.thread_num == 1:
        #             self.visualizer.run_joint_trajectory(q_traj, dt)
 
    def move_q(self, q_end: list[float], duration_sec=2.0, dt_sec=0.03):
        """robot motion with smoothstep time scaling
        """

        q_start = self.real_robot.get_current_joint_positions()
        q_start = np.array(q_start)
        q_end = np.array(q_end)
 
        n_steps = ceil(duration_sec / dt_sec)
 
        q_traj = [q_start]
 
        for i in range(n_steps):
            s = i / n_steps
            s = 3 * (s ** 2) - 2 * (s ** 3) # Smoothstep interpolation
            q_step = q_start + (q_end - q_start) * s
            q_traj.append(q_step)
 
        q_traj.append(q_end)
 
        self.real_robot.run_joint_trajectory(q_traj, dt_sec)
        # if self.visualizer is None: 
        #     # move robot directly
        #     self.real_robot.run_joint_trajectory(q_traj, dt_sec)
        # else:
        #     # move robot and visualizer simultaneously
        #     with pymp.Parallel(2) as p:
        #         if p.thread_num == 0:
        #             self.real_robot.run_joint_trajectory(q_traj, dt_sec)
        #         elif p.thread_num == 1:
        #             self.visualizer.run_joint_trajectory(q_traj, dt_sec)
        
    def get_pose(self) -> SE3:
        """get current robot pose in SE3. this perfroms forward kinematics"""
        q_current = self.real_robot.get_current_joint_positions()
        return self.sim_model.fkine(q_current)
 
    def grip(self, q: float):       
        """move gripper to q position"""
        self.real_gripper.move(q, 0.1)
        # if self.visualizer is None: 
        #     # move robot directly
        #     self.real_gripper.move(q, 0.1)
        # else:
        #     # move robot and visualizer simultaneously
        #     with pymp.Parallel(2) as p:
        #         if p.thread_num == 0:
        #             self.real_gripper.move(q, 0.1)
        #         elif p.thread_num == 1:
        #             self.visualizer.move_gripper(q, 0.1)

    def grip_open(self):
        """open gripper to grip_open_q position"""
        self.grip(self.grip_open_q)
        
    def grip_close(self):
        """close gripper to grip_closed_q position"""
        self.grip(self.grip_closed_q)
 
    def move_piece(self,
                   src: tuple[int, int], 
                   dest: tuple[int, int], 
                   drop_at_height=False
                   ) -> None:
        """move piece from src to dest on the board"""
        board_corner = self.board_origin \
            * SE3.Trans(-self.board_size / 2, -self.board_size / 2, 0) # this is the 0,0 coordinate of the board in the world frame
        src_pos = board_corner \
            * SE3.Tx(self.board_size * src[0] / 8.0) \
            * SE3.Ty(self.board_size * src[1] / 8.0)
        dest_pos = board_corner \
            * SE3.Tx(self.board_size * dest[0] / 8.0) \
            * SE3.Ty(self.board_size * dest[1] / 8.0)
 
        self.grip_open()
        self.move_ik(src_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        self.move_ik(src_pos * SE3.Tz(self.low_floor) * SE3.Rx(np.pi))
        self.grip_close()
        self.move_ik(src_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        self.move_ik(dest_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
        if not drop_at_height: 
            self.move_ik(dest_pos * SE3.Tz(self.low_floor) * SE3.Rx(np.pi))
        self.grip_open()
        if not drop_at_height: 
            self.move_ik(dest_pos * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
 
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
            self.move_ik(corner * SE3.Tz(self.mid_floor) * SE3.Rx(np.pi))
 
    def move_home(self):
        self.grip_open()
        # This moves to about home
        self.move_q([-0.0019072040291727667, -0.3332439469118848, -0.11625275145144795, -1.7683176150558584, -0.04128156322020071, 1.4369431870248583, 0.671426051719321])
        
        current_pose = self.get_pose()
        if not np.allclose(current_pose.t, self.camera_origin.t, atol=1e-3):
            # Not quite there yet, lets use IK to fix
            self.move_ik(self.camera_origin * SE3.Tz(self.high_floor) * SE3.Rx(np.pi))
 
    def _cell_id_to_coord(self, n: int) -> tuple[int, int]:
        row = (n - 1) // 4
        col_in_row = (n - 1) % 4
        # on even rows dark squares start at column 1, on odd rows at 0
        col = col_in_row * 2 + ((row + 1) % 2)
        return row, col
    
    def exec_player_move(self, move: Move, color=None):
        steps = move.steps_move
        if not steps or len(steps) < 2:
            return

        # handle multi‑jump captures
        for i in range(len(steps) - 1):
            src_sq = steps[i]
            dst_sq = steps[i + 1]
            src = self._cell_id_to_coord(src_sq)
            dst = self._cell_id_to_coord(dst_sq)
            print(f"Robot moving piece from {src} -> {dst}")
            self.move_piece(src, dst)
 
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if self.visualizer is not None:
            self.visualizer.stop() # Makes sure render thread ends
 
 
# if __name__ == "__main__":
#     visualizer = main()
#     visualizer.stop() # Makes sure render thread ends
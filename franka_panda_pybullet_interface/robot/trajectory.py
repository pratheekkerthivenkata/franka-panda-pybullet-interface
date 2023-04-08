from sys import path

import numpy as np

build_path = '/home/anuj/npm/ruckig/build'
path.insert(0, str(build_path))
from ruckig import InputParameter, Result, Ruckig, ControlInterface


class Trajectory:
    def __init__(self, moveit):
        self.moveit = moveit

    def get_ruckig_traj(self, start_q, end_q):
        otg = Ruckig(6, 0.001)  # DoFs, control cycle
        inp = InputParameter(6)

        inp.control_interface = ControlInterface.Velocity

        inp.current_position = start_q
        inp.target_position = end_q

        inp.current_velocity = np.zeros(6)
        inp.target_velocity = np.zeros(6)

        inp.current_acceleration = np.zeros(6)
        inp.target_acceleration = np.zeros(6)

        # vel_lims = [3, 3, 3, 2.5, 2.5, 2.5]
        # acc_lims = [9, 9, 9, 17, 17, 17]
        # jerk_lims = [4500, 4500, 4500, 8500, 8500, 8500]

        # vel_lims = [1.7 / 4, 1.7 / 4, 1.7 / 4, 2.1750 / 4, 2.1750 / 4, 2.1750 / 4]
        # acc_lims = [13 / 10, 13 / 10, 13 / 10, 10 / 10, 10 / 10, 10 / 10]
        # jerk_lims = [5500 / 10, 5500 / 10, 5500 / 10, 3000 / 10, 3000 / 10, 3000 / 10]
        vel_lims = [1.7, 1.7, 1.7, 2.5, 2.5, 2.5]
        acc_lims = [13, 13, 13, 25, 25, 25]
        jerk_lims = [6500, 6500, 6500, 12500, 12500, 12500]

        inp.max_velocity = vel_lims
        inp.max_acceleration = acc_lims
        inp.max_jerk = jerk_lims

        traj = Trajectory(6)
        result = otg.calculate(inp, traj)
        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        # import pdb; pdb.set_trace()

        q_traj = []
        dq_traj = []
        ddq_traj = []
        for t in np.arange(0, traj.duration, 0.001):
            q, dq, ddq = traj.at_time(t)
            q_traj.append(q)
            dq_traj.append(dq)
            ddq_traj.append(ddq)

        return q_traj, dq_traj, ddq_traj

    def get_plan(self, start_q, end_q):
        self.__group.set_start_state(self._create_ros_joint_msg(start_q)[1])
        self.__group.set_joint_value_target(self._create_ros_joint_msg(end_q)[0])
        retval = self.__group.plan()
        # return retval[1]
        success, traj, _, _ = retval
        # traj = self.__group.retime_trajectory(self.__group.get_current_state(),
        #                                       traj, velocity_scaling_factor=0.3, acceleration_scaling_factor=0.3, algorithm='time_optimal_trajectory_generation')

        traj = traj.joint_trajectory
        pts = traj.points

        q = []
        dq = []
        ddq = []
        tau = []
        t = []
        for waypt in pts:
            q.append(waypt.positions)
            dq.append(waypt.velocities)
            ddq.append(waypt.accelerations)
            tau.append(waypt.effort)
            t.append(waypt.time_from_start.to_sec())
        return t, tau, q, dq, ddq

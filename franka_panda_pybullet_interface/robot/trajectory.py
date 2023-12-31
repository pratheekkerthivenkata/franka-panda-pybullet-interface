import os
from sys import path

import numpy as np

build_path = os.path.join(os.path.expanduser('~'), 'npm/ruckig/build')
path.insert(0, str(build_path))
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface, Trajectory as RuckigTrajectory
from copy import copy


class Trajectory:
    def __init__(self, moveit, robot_limits):
        self.moveit = moveit
        self.robot_limits = robot_limits

    def get_ruckig_traj(self, start_q=None, start_dq=None, start_ddq=None, end_q=None, end_dq=None, end_ddq=None):
        otg = Ruckig(7, 0.001)  # DoFs, control cycle
        inp = InputParameter(7)
        out = OutputParameter(7)

        inp.control_interface = ControlInterface.Velocity

        if start_q is not None:
            inp.current_position = start_q
        if start_dq is not None:
            inp.current_velocity = start_dq
        if start_ddq is not None:
            inp.current_acceleration = start_ddq

        if end_q is not None:
            inp.target_position = end_q
        if end_dq is not None:
            inp.target_velocity = end_dq
        if end_ddq is not None:
            inp.target_acceleration = end_ddq

        # inp.max_position = self.robot_limits.q
        inp.max_velocity = self.robot_limits.dq[:, 1] / 2
        inp.max_acceleration = self.robot_limits.ddq[:, 1] / 2
        inp.max_jerk = self.robot_limits.dddq[:, 1]

        traj = RuckigTrajectory(7)
        result = otg.calculate(inp, traj)
        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

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
        self.moveit.group.set_start_state(self.moveit._create_ros_joint_msg(start_q)[1])
        self.moveit.group.set_joint_value_target(self.moveit._create_ros_joint_msg(end_q)[0])

        retval = self.moveit.group.plan()
        success, traj, _, _ = retval

        if success:
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
        return None

    # def get_traj(self, start_q, end_q):
    #     retval = self.get_plan(start_q, end_q)
    #     if retval is None:
    #         return None
    #     # t, tau, q, dq, ddq = retval
    #     return retval
    #     # dense_t = []
    #     # dense_q = []
    #     # for i in range(len(t) - 1):
    #     #     ts, qs = self.linear_interpolation_multidimensional([t[i], t[i + 1]], [q[i], q[i + 1]])
    #     #     dense_t.extend(ts)
    #     #     dense_q.extend(qs)
    #     # return dense_t, None, dense_q, None, None
    #
    #     # dense_q = []
    #     # dense_dq = []
    #     # dense_ddq = []
    #     # start_q = q[0]
    #     # start_dq = dq[0]
    #     # start_ddq = ddq[0]
    #     # for i in range(len(t) - 1):
    #     #     q_traj, dq_traj, ddq_traj = self.get_ruckig_traj(start_q=start_q, start_dq=start_dq, start_ddq=start_ddq,
    #     #                                                      end_q=q[i+1], end_dq=dq[i+1], end_ddq=ddq[i+1])
    #     #     dense_q.extend(q_traj)
    #     #     dense_dq.extend(dq_traj)
    #     #     dense_ddq.extend(ddq_traj)
    #     #     start_q = dense_q[-1]
    #     #     start_dq = dense_dq[-1]
    #     #     start_ddq = dense_ddq[-1]
    #     # dense_t = np.arange(0, len(dense_q) * 0.001, 0.001)
    #     # return dense_t, None, dense_q, dense_dq, dense_ddq

    def get_traj(self, start_q, end_q):
        retval = self.get_plan(start_q, end_q)
        if retval is None:
            return None
        t, _, q, dq, ddq = retval
        # t, tau, q, dq, ddq = retval
        # Create instances: the Ruckig OTG as well as input and output parameters
        otg = Ruckig(7, 0.001,
                     len(q) - 2)  # DoFs, control cycle rate, maximum number of intermediate waypoints for memory allocation
        inp = InputParameter(7)  # DoFs
        out = OutputParameter(7, len(q))  # DoFs, maximum number of intermediate waypoints for memory allocation

        inp.current_position = q[0]
        inp.current_velocity = np.zeros(7)
        inp.current_acceleration = np.zeros(7)

        inp.intermediate_positions = q[1:-1]

        inp.target_position = q[-1]
        inp.target_velocity = np.zeros(7)
        inp.target_acceleration = np.zeros(7)

        # inp.max_velocity = self.robot_limits.dq[:, 1]
        # inp.max_acceleration = self.robot_limits.ddq[:, 1]
        # inp.max_jerk = self.robot_limits.dddq[:, 1]
        inp.max_velocity = [2.1750 / 2, 2.1750 / 2, 2.1750 / 2, 2.1750 / 2, 2.6100 / 2, 2.6100 / 2, 2.1600 / 2]
        inp.max_acceleration = [15 / 2, 7.5 / 2, 10 / 2, 12.5 / 2, 15 / 2, 20 / 2, 20 / 2]
        inp.max_jerk = [5500, 2000, 3000, 3250, 4500, 6000, 6000]

        # Generate the trajectory within the control loop
        first_output, out_list = None, []
        res = Result.Working
        qs = [q[0]]
        dqs = [[0, 0, 0, 0, 0, 0, 0]]
        ddqs = [[0, 0, 0, 0, 0, 0, 0]]
        ts = [0.0]
        while res == Result.Working:
            res = otg.update(inp, out)

            out_list.append(copy(out))
            qs.append(copy(out.new_position))
            dqs.append(copy(out.new_velocity))
            ddqs.append(copy(out.new_acceleration))
            ts.append(out.time)

            out.pass_to_input(inp)

            if not first_output:
                first_output = copy(out)
        return ts, None, qs, dqs, ddqs, t, q

        # dense_q = []
        # dense_dq = []
        # dense_ddq = []
        # for i in range(len(t) - 1):
        #     q_traj, dq_traj, ddq_traj = self.get_ruckig_traj(start_q=q[i], start_dq=dq[i], start_ddq=ddq[i],
        #                                                      end_q=q[i+1], end_dq=dq[i+1], end_ddq=ddq[i+1])
        #     dense_q.extend(q_traj)
        #     dense_dq.extend(dq_traj)
        #     dense_ddq.extend(ddq_traj)
        # dense_t = np.arange(0, len(dense_q) * 0.001, 0.001)
        # return dense_t, None, dense_q, dense_dq, dense_ddq

    def linear_interpolation_multidimensional(self, timestamps, joint_angles, delta_t=0.001):
        new_timestamps = np.arange(timestamps[0], timestamps[-1], delta_t)
        new_joint_angles = np.zeros((len(new_timestamps), len(joint_angles[0])))

        for i in range(len(joint_angles[0])):
            new_joint_angles[:, i] = np.interp(new_timestamps, timestamps,
                                               [joint_angle[i] for joint_angle in joint_angles])

        return new_timestamps, new_joint_angles

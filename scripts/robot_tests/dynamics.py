#!/usr/bin/env python

from franka_panda_pybullet_interface.robot.dynamics import Dynamics
from franka_panda_pybullet_interface.robot.limits import Limits
from franka_panda_pybullet_interface.robot.robot import Robot
from franka_panda_pybullet_interface.envs import RobotEnv

if __name__ == '__main__':
    limits = Limits()
    env = RobotEnv(enable_gui=True, enable_realtime=True)
    #Robot(client_id=self.sim_id, enable_realtime=self.enable_realtime, timestep=self.timestep)
    q = limits.sample_q()
    dq = limits.sample_dq()
    tau = limits.sample_tau()

    print('q:\n', q)
    print('\ndq:\n', dq)
    print('\ntau:\n', tau)
    robot=Robot(env.sim_id,env.enable_realtime,env.timestep)

    dynamics = Dynamics(robot)
    ddq = dynamics.get_fd_solution(q, dq, tau)
    print('\nddq = FD(q, dq, tau) =\n', ddq)
    print('\ntau = ID(q, dq, ddq) =\n', dynamics.get_id_solution(q, dq, ddq))

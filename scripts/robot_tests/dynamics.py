#!/usr/bin/env python

from franka_panda_pybullet_interface.robot.dynamics import Dynamics
from franka_panda_pybullet_interface.robot.limits import Limits

if __name__ == '__main__':
    limits = Limits()
    q = limits.sample_q()
    dq = limits.sample_dq()
    tau = limits.sample_tau()

    print('q:\n', q)
    print('\ndq:\n', dq)
    print('\ntau:\n', tau)

    dynamics = Dynamics()
    ddq = dynamics.get_fd_solution(q, dq, tau)
    print('\nddq = FD(q, dq, tau) =\n', ddq)
    print('\ntau = ID(q, dq, ddq) =\n', dynamics.get_id_solution(q, dq, ddq))

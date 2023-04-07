#!/usr/bin/env python

from franka_panda_pybullet_interface.robot.limits import Limits

if __name__ == '__main__':
    limits = Limits()

    print('q limits:\n', limits.q)  # also available: dq, ddq, dddq, tau, ee_pose
    print('\nsampled q:\n', limits.sample_q())  # also available: sample_dq()

    # also available: scale_dq_limits(), scale_ddq_limits(), scale_dddq_limits(), scale_tau_limits()
    limits.scale_q_limits(0.5)
    print('\nscaled q limits:\n', limits.q)

    print('\nis within limits (pass):\n', limits.is_valid(q=limits.sample_q()))
    # if multiple args provided, all must be within limits
    # valid args: q, dq, ddq, dddq, tau
    print('\nis within limits (fail):\n', limits.is_valid(q=limits.sample_q(), dq=50 * limits.sample_dq()))

#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

from franka_panda_pybullet_interface.envs.single_arm_hiro_env import SingleArmHIROEnv

if __name__ == '__main__':
    env = SingleArmHIROEnv(enable_gui=True, enable_realtime=True)

    start_q = env.robot.current_state.real_q
    end_q = env.robot.limits.sample_q()

    retval = env.robot.trajectory.get_traj(start_q=start_q, end_q=end_q)
    if retval is not None:
        dense_t, _, dense_q, dense_dq, dense_ddq, t, q = retval
        print(dense_t[0], dense_t[-1])
        print('---------------------')
        print(dense_q[0], dense_q[-1])
        print('---------------------')
        print(start_q, end_q)
        print('---------------------')
        print(dense_dq[0], dense_dq[-1])
        print('---------------------')
        print(dense_ddq[0], dense_ddq[-1])
        env.robot.execute_q_trajectory_real(dense_dq, dense_t)

        print(env.robot.current_state.real_q)
        print(end_q)

# from franka_panda_pybullet_interface.envs.single_arm_hiro_env import SingleArmHIROEnv
#
# if __name__ == '__main__':
#     env = SingleArmHIROEnv(enable_gui=True, enable_realtime=True)
#     # env.robot.motion.move_to_default_q_real()
#     # print(env.robot.current_state.q)
#     # print(env.robot.current_state.real_q)
#
#     random_q = env.robot.limits.sample_q()
#     retval = env.robot.trajectory.get_traj(start_q=env.robot.current_state.real_q, end_q=random_q)
#     if retval is not None:
#         dense_t, _, dense_q, _, _ = retval
#         env.robot.execute_q_trajectory_real(dense_q, dense_t)



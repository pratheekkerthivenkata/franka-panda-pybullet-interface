#!/usr/bin/env python

from franka_panda_pybullet_interface.envs.single_arm_hiro_env import SingleArmHIROEnv

if __name__ == '__main__':
    env = SingleArmHIROEnv(enable_gui=True, enable_realtime=True)

    random_q = env.robot.limits.sample_q()

    retval = env.robot.trajectory.get_traj(start_q=env.robot.q, end_q=random_q)
    if retval is not None:
        dense_t, _, dense_q, _, _ = retval
        env.robot.execute_q_trajectory(dense_q, dense_t)

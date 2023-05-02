self.motion = Motion()
input('Press Enter to continue...')

# plot, actual vs expected
# print err

# ret traj from fns etc


#!/usr/bin/env python

from franka_panda_pybullet_interface.envs.single_arm_hiro_env import SingleArmHIROEnv


if __name__ == '__main__':
    env = SingleArmHIROEnv(enable_gui=True, enable_realtime=False)

    for _ in range(25):
        q = env.robot.limits.sample_q()
        retval = env.robot.trajectory.get_plan(start_q=env.robot.q, end_q=q)
        if retval is None:
            continue
        t, _, q, _, _ = retval
        env.robot.execute_q_trajectory(q, t)
        print(f'Self-Collision: {env.collision_checker.is_robot_in_self_collision()}, '
              f'Environment Collision: {env.collision_checker.is_robot_in_collision_with_env()}')
        input()

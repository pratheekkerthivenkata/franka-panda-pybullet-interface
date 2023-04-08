#!/usr/bin/env python

from franka_panda_pybullet_interface.envs.single_arm_hiro_env import SingleArmHIROEnv


if __name__ == '__main__':
    env = SingleArmHIROEnv(enable_gui=True, enable_realtime=False)

    for _ in range(10):
        q = env.robot.limits.sample_q()
        env.robot.move_to_q(q, direct=True)
        print(f'Self-Collision: {env.collision_checker.is_robot_in_self_collision()}, '
              f'Environment Collision: {env.collision_checker.is_robot_in_collision_with_env()}')

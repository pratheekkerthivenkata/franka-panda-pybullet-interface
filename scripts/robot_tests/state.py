#!/usr/bin/env python

from franka_panda_pybullet_interface.envs import RobotEnv

if __name__ == '__main__':
    env = RobotEnv(enable_gui=True, enable_realtime=False)
    print('q: ', env.robot.q)
    print('dq: ', env.robot.dq)
    print('ddq: ', env.robot.ddq)
    print('ee_pose: ', env.robot.ee_pose)
    print('ee_velocity: ', env.robot.ee_velocity)

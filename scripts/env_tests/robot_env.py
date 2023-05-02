#!/usr/bin/env python

from franka_panda_pybullet_interface.envs import RobotEnv


if __name__ == '__main__':
    env = RobotEnv(enable_gui=True, enable_realtime=False)
    input('Press Enter to continue...')

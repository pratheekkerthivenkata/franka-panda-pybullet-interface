#!/usr/bin/env python

from franka_panda_pybullet_interface.envs import SimEnv


if __name__ == '__main__':
    env = SimEnv(enable_gui=True, enable_realtime=False)
    input('Press Enter to continue...')

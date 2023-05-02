#!/usr/bin/env python

from franka_panda_pybullet_interface.envs import SingleArmHIROEnv


if __name__ == '__main__':
    env = SingleArmHIROEnv(enable_gui=True, enable_realtime=False)
    input()

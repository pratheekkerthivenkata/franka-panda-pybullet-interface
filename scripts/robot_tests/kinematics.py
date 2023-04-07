#!/usr/bin/env python

import os

from franka_panda_pybullet_interface.robot.kinematics import Kinematics
from franka_panda_pybullet_interface.robot.limits import Limits
from franka_panda_pybullet_interface.definitions import ASSETS_DIR

if __name__ == '__main__':
    limits = Limits()
    q = limits.sample_q()

    urdf_path = os.path.join(ASSETS_DIR, 'robot', 'panda.urdf')

    print('q: ', q)

    kinematics = Kinematics(urdf_path, 'panda_link0', 'panda_grasptarget')
    ee_pose = kinematics.get_fk_solution(q)
    print('ee_pose = FK(q) =\n', ee_pose)
    print('q = IK(ee_pose) =\n', kinematics.get_ik_solution(ee_pose, seed_q=limits.sample_q()))

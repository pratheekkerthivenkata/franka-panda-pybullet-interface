from copy import deepcopy

import numpy as np
import transformations as tf
from trac_ik_python.trac_ik import IK

from ..utils.datatypes import Pose, Point
from ..utils.file_io import load_txt
from ..utils.robot import get_dh_params, get_tf_mat


class Kinematics:
    def __init__(self, urdf_path, base_link, tip_link):
        self.ik_solver = IK(base_link=base_link, tip_link=tip_link,
                            solve_type='Distance', urdf_string=load_txt(urdf_path))

    @staticmethod
    def get_fk_solution(q, get_mat=False, euler=False):
        dh_params = get_dh_params(q)

        T = np.eye(4)
        for i in range(10):
            T = T @ get_tf_mat(i, dh_params)
        euler_angles = tf.euler_from_matrix(T, axes='sxyz')
        translations = tf.translation_from_matrix(T)

        if get_mat:
            return T

        pose = Pose(position=Point(*translations), orientation=Point(*euler_angles))
        pose.convert_orientation(euler=euler)
        return pose

    def get_ik_solution(self, ee_pose, seed_q):
        ee_pose_copy = deepcopy(ee_pose)
        ee_pose_copy.convert_orientation(euler=False)

        target_q = self.ik_solver.get_ik(
            seed_q,
            ee_pose_copy.position.x, ee_pose_copy.position.y, ee_pose_copy.position.z,
            ee_pose_copy.orientation.x, ee_pose_copy.orientation.y, ee_pose_copy.orientation.z, ee_pose_copy.orientation.w)

        if target_q is None:
            return None
        return np.array(target_q)

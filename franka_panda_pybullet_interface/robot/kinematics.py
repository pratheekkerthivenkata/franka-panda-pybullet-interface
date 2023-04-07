import numpy as np
import transformations as tf
from trac_ik_python.trac_ik import IK

from ..utils.datatypes import Pose, Point
from ..utils.file_io import load_txt
from ..utils.misc import convert_orientation
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

        return Pose(position=Point(*translations), orientation=convert_orientation(Point(*euler_angles), euler))

    def get_ik_solution(self, ee_pose, seed_q):
        ee_orientation = convert_orientation(ee_pose.orientation, euler=False)

        target_q = self.ik_solver.get_ik(
            seed_q,
            ee_pose.position.x, ee_pose.position.y, ee_pose.position.z,
            ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w)

        return np.array(target_q)

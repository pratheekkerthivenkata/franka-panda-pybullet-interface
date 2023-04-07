import panda_dynamics_model as pdm
import pybullet as pb

# numba-optimized functions cannot be non-static class members
from ..utils.robot import get_jacobian, get_jacobian_derivative, get_manipulability, get_dynamic_manipulability


class Attributes:
    def __init__(self, robot_id):
        self._joint_ids = []
        self._joint_names = []
        self._finger_joint_ids = []
        for joint_id in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, joint_id)

            joint_name = str(joint_info[1])
            joint_type = joint_info[2]

            if joint_name == 'panda_grasptarget_hand':
                self._ee_link_id = joint_id

            if joint_type != pb.JOINT_FIXED:
                if 'finger' in joint_name:
                    self._finger_joint_ids.append(joint_id)
                else:
                    self._joint_ids.append(joint_id)
                    self._joint_names.append(joint_name)

    @property
    def num_joints(self):
        return len(self._joint_ids)

    @property
    def joint_ids(self):
        return self._joint_ids

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def ee_link_id(self):
        return self._ee_link_id

    @property
    def ee_link_name(self):
        return 'panda_grasptarget'

    @property
    def finger_joint_ids(self):
        return self._finger_joint_ids

    @staticmethod
    def get_jacobian(q):
        return get_jacobian(q)

    @staticmethod
    def get_jacobian_derivative(dq, q=None, J=None):
        assert (q is None) ^ (J is None), 'Either q or J must be provided, but not both'
        if J is None:
            J = get_jacobian(q)
        return get_jacobian_derivative(dq, J)

    @staticmethod
    def get_mass_matrix(q):
        return pdm.get_mass_matrix(q)

    @staticmethod
    def get_coriolis_matrix(q, dq):
        return pdm.get_coriolis_matrix(q, dq)

    @staticmethod
    def get_gravity_vector(q):
        return pdm.get_gravity_vector(q)

    @staticmethod
    def get_friction_torques(dq):
        return pdm.get_friction_torques(dq)

    @staticmethod
    def get_manipulability(q):
        return get_manipulability(q)

    def get_dynamic_manipulability(self, q):
        return get_dynamic_manipulability(q, self.get_mass_matrix(q))

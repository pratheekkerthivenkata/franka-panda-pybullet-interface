import numpy as np
from ..utils.datatypes import Pose, Point


class Limits:
    def __init__(self):
        """
        https://frankaemika.github.io/docs/control_parameters.html#limits-for-panda
        """

        self._q = np.array([[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]]).transpose()

        self._dq = np.array([[-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61],
                             [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]]).transpose()

        self._ddq = np.array([[-15, -7.5, -10, -12.5, -15, -20, -20],
                              [15, 7.5, 10, 12.5, 15, 20, 20]]).transpose()

        # self._dddq = np.array([[-7500, 3750, -5000, -6250, -7500, -10000, -10000],
        #                        [7500, 3750, 5000, 6250, 7500, 10000, 10000]]).transpose()
        self._dddq = np.array([[-5500, -2000, -3000, -3250, -4500, -6000, -6000],
                               [5500, 2000, 3000, 3250, 4500, 6000, 6000]]).transpose()

        self._tau = np.array([[-87, -87, -87, -87, -12, -12, -12],
                              [87, 87, 87, 87, 12, 12, 12]]).transpose()

        """
        https://wiredworkers.io/wp-content/uploads/2019/12/Panda_FrankaEmika_ENG.pdf
        """

        self._ee_pose = np.array([[-0.855, -0.855, -0.36, -np.pi, -np.pi, -np.pi],
                                  [0.855, 0.855, 1.19, np.pi, np.pi, np.pi]]).transpose()

    @property
    def q(self):
        return self._q

    @property
    def dq(self):
        return self._dq

    @property
    def ddq(self):
        return self._ddq

    @property
    def dddq(self):
        return self._dddq

    @property
    def tau(self):
        return self._tau

    @property
    def ee_pose(self):
        return self._ee_pose

    def is_valid(self, **kwargs):
        for key, value in kwargs.items():
            if key == 'q':
                if not np.all(self._q[:, 0] <= value) or not np.all(value <= self._q[:, 1]):
                    return False
            elif key == 'dq':
                if not np.all(self._dq[:, 0] <= value) or not np.all(value <= self._dq[:, 1]):
                    return False
            elif key == 'ddq':
                if not np.all(self._ddq[:, 0] <= value) or not np.all(value <= self._ddq[:, 1]):
                    return False
            elif key == 'dddq':
                if not np.all(self._dddq[:, 0] <= value) or not np.all(value <= self._dddq[:, 1]):
                    return False
            elif key == 'tau':
                if not np.all(self._tau[:, 0] <= value) or not np.all(value <= self._tau[:, 1]):
                    return False
            else:
                raise ValueError(f'Unknown key: {key}')

        return True

    def scale_q_limits(self, scale):
        assert scale > 0
        self._q *= scale

    def scale_dq_limits(self, scale):
        assert scale > 0
        self._dq *= scale

    def scale_ddq_limits(self, scale):
        assert scale > 0
        self._ddq *= scale

    def scale_dddq_limits(self, scale):
        assert scale > 0
        self._dddq *= scale

    def scale_tau_limits(self, scale):
        assert scale > 0
        self._tau *= scale

    def sample_q(self):
        return np.random.uniform(self._q[:, 0], self._q[:, 1])

    def sample_dq(self):
        return np.random.uniform(self._dq[:, 0], self._dq[:, 1])

    def sample_tau(self):
        return np.random.uniform(self._tau[:, 0], self._tau[:, 1])

    def sample_ee_pose(self, euler):
        rand_ee_pose = np.random.uniform(self._ee_pose[:, 0], self._ee_pose[:, 1])
        ee_pose = Pose(position=Point(*rand_ee_pose[:3]), orientation=Point(*rand_ee_pose[3:]))
        ee_pose.convert_orientation(euler=euler)
        return ee_pose

import numpy as np
from ..utils.datatypes import Pose, Point
from ..definitions import PANDA_REAL_TIMESTEP


class Limits:
    def __init__(self):
        # https://frankaemika.github.io/docs/control_parameters.html#limits-for-panda
        self._q = np.array([[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]]).transpose()

        self._dq = np.array([[-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61],
                             [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]]).transpose()

        self._ddq = np.array([[-15, -7.5, -10, -12.5, -15, -20, -20],
                              [15, 7.5, 10, 12.5, 15, 20, 20]]).transpose()

        self._dddq = np.array([[-7500, -3750, -5000, -6250, -7500, -10000, -10000],
                               [7500, 3750, 5000, 6250, 7500, 10000, 10000]]).transpose()

        self._tau = np.array([[-87, -87, -87, -87, -12, -12, -12],
                              [87, 87, 87, 87, 12, 12, 12]]).transpose()

        self._dtau = np.array([[-1000, -1000, -1000, -1000, -1000, -1000, -1000],
                               [1000, 1000, 1000, 1000, 1000, 1000, 1000]]).transpose()

        # https://wiredworkers.io/wp-content/uploads/2019/12/Panda_FrankaEmika_ENG.pdf
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
    def dtau(self):
        return self._dtau

    @property
    def ee_pose(self):
        return self._ee_pose

    def is_valid(self, **kwargs):
        for key, value in kwargs.items():
            if key == 'q':
                if not np.all(value >= self._q[:, 0]) or not np.all(value <= self._q[:, 1]):
                    return False
            elif key == 'dq':
                if not np.all(value >= self._dq[:, 0]) or not np.all(value <= self._dq[:, 1]):
                    return False
            elif key == 'ddq':
                if not np.all(value >= self._ddq[:, 0]) or not np.all(value <= self._ddq[:, 1]):
                    return False
            elif key == 'dddq':
                if not np.all(value >= self._dddq[:, 0]) or not np.all(value <= self._dddq[:, 1]):
                    return False
            elif key == 'tau':
                if not np.all(value >= self._tau[:, 0]) or not np.all(value <= self._tau[:, 1]):
                    return False
            elif key == 'dtau':
                if not np.all(value >= self._dtau[:, 0]) or not np.all(value <= self._dtau[:, 1]):
                    return False
            else:
                raise ValueError(f'Unknown key: {key}')

        return True

    def scale_q_limits(self, scale):
        assert scale >= 0
        self._q *= scale

    def scale_dq_limits(self, scale):
        assert scale >= 0
        self._dq *= scale

    def scale_ddq_limits(self, scale):
        assert scale >= 0
        self._ddq *= scale

    def scale_dddq_limits(self, scale):
        assert scale >= 0
        self._dddq *= scale

    def scale_tau_limits(self, scale):
        assert scale >= 0
        self._tau *= scale

    def scale_dtau_limits(self, scale):
        assert scale >= 0
        self._dtau *= scale

    def sample_q(self, min_q=None, max_q=None):
        if min_q is None:
            min_q = self._q[:, 0]
        if max_q is None:
            max_q = self._q[:, 1]
        return np.random.uniform(min_q, max_q)

    def sample_valid_q(self, min_q=None, max_q=None):
        q = self.sample_q(min_q, max_q)
        # while not self.is_valid(q=q):
        #     q = self.sample_q(min_q, max_q)
        return q

    def sample_dq(self, min_dq=None, max_dq=None):
        if min_dq is None:
            min_dq = self._dq[:, 0]
        if max_dq is None:
            max_dq = self._dq[:, 1]
        return np.random.uniform(min_dq, max_dq)

    def sample_ddq(self, min_ddq=None, max_ddq=None):
        if min_ddq is None:
            min_ddq = self._ddq[:, 0]
        if max_ddq is None:
            max_ddq = self._ddq[:, 1]
        return np.random.uniform(min_ddq, max_ddq)

    def sample_dddq(self, min_dddq=None, max_dddq=None):
        if min_dddq is None:
            min_dddq = self._dddq[:, 0]
        if max_dddq is None:
            max_dddq = self._dddq[:, 1]
        return np.random.uniform(min_dddq, max_dddq)

    def sample_tau(self, min_tau=None, max_tau=None):
        if min_tau is None:
            min_tau = self._tau[:, 0]
        if max_tau is None:
            max_tau = self._tau[:, 1]
        return np.random.uniform(min_tau, max_tau)

    def sample_dtau(self, min_dtau=None, max_dtau=None):
        if min_dtau is None:
            min_dtau = self._dtau[:, 0]
        if max_dtau is None:
            max_dtau = self._dtau[:, 1]
        return np.random.uniform(min_dtau, max_dtau)

    def sample_ee_pose(self, euler):
        rand_ee_pose = np.random.uniform(self._ee_pose[:, 0], self._ee_pose[:, 1])
        ee_pose = Pose(position=Point(*rand_ee_pose[:3]), orientation=Point(*rand_ee_pose[3:]))
        ee_pose.convert_orientation(euler=euler)
        return ee_pose

    def is_q_continuous(self, q_k, q_k1, timestep=PANDA_REAL_TIMESTEP):
        dq = (q_k1 - q_k) / timestep
        return self.is_valid(dq=dq)

    def is_dq_continuous(self, dq_k, dq_k1, timestep=PANDA_REAL_TIMESTEP):
        ddq = (dq_k1 - dq_k) / timestep
        return self.is_valid(ddq=ddq)

    def is_ddq_continuous(self, ddq_k, ddq_k1, timestep=PANDA_REAL_TIMESTEP):
        dddq = (ddq_k1 - ddq_k) / timestep
        return self.is_valid(dddq=dddq)

    def is_tau_continuous(self, tau_k, tau_k1, timestep=PANDA_REAL_TIMESTEP):
        dtau = (tau_k1 - tau_k) / timestep
        return self.is_valid(dtau=dtau)

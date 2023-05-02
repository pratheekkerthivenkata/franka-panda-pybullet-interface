import numpy as np
from numbalsoda import lsoda

from ..utils.robot import deriv, get_dq, get_ddq, get_tau
from ..utils.robot import get_fd_solution, get_id_solution


class Dynamics:
    def __init__(self, robot):
        self.robot = robot

    def get_fd_solution(self, q, dq, tau):
        M = self.robot.attributes.get_mass_matrix(q)
        C = self.robot.attributes.get_coriolis_matrix(q, dq)
        g = self.robot.attributes.get_gravity_vector(q)
        f = self.robot.attributes.get_friction_torques(dq)
        return get_fd_solution(M, C, g, f, dq, tau)

    def get_id_solution(self, q, dq, ddq):
        M = self.robot.attributes.get_mass_matrix(q)
        C = self.robot.attributes.get_coriolis_matrix(q, dq)
        g = self.robot.attributes.get_gravity_vector(q)
        f = self.robot.attributes.get_friction_torques(dq)
        return get_id_solution(M, C, g, f, dq, ddq)

    @staticmethod
    def __simulate(start_state, control_input, duration, deriv_fn):
        rtol = 10e-12
        funcptr = deriv_fn.address

        t = duration
        y0_ = np.array(start_state, dtype=np.float64)
        atol = 10e-12 * np.array([1] * len(start_state))
        data = np.array(control_input, dtype=np.float64)
        usol, success = lsoda(funcptr, y0_, np.array(t), data, rtol=rtol, atol=atol[0])
        return usol

    def simulate(self, start_q, ee_vel, duration):
        q = start_q
        ee_acc_k = np.zeros(6, dtype=np.float64)  # each edge is constant velocity
        states = [start_q]
        timesteps = np.arange(0, duration + 0.001, 0.001)

        for idx, t in enumerate(timesteps[:-1]):
            J = self.robot.attributes.get_jacobian(q)
            dq = get_dq(J, ee_vel)
            if not self.robot.limits.is_valid(dq=dq):
                return False, None

            M = self.robot.attributes.get_mass_matrix(q)
            C = self.robot.attributes.get_coriolis_matrix(q, dq)
            g = self.robot.attributes.get_gravity_vector(q)
            f = self.robot.attributes.get_friction_torques(dq)

            ddq = get_ddq(J, ee_acc_k, dq)
            if not self.robot.limits.is_valid(ddq=ddq):
                return False, None

            tau = get_tau(M, ddq, C, dq, g, f)
            if not self.robot.limits.is_valid(tau=tau):
                return False, None

            data = np.concatenate([tau, M.flatten(), C.flatten(), g, f], dtype=np.float64)
            sol = self.__simulate(start_q, data, [t, timesteps[idx + 1]], deriv)[-1]

            q = sol[:self.robot.num_joints]
            if not self.robot.limits.is_valid(q=q):
                return False, None

            if not self.robot.self_collision_checker(q):
                return False, None

            start_q = q
            states.append(start_q)

        return True, states

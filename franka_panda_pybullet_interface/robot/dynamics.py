from .attributes import Attributes
from ..utils.robot import get_fd_solution, get_id_solution


class Dynamics:
    def __init__(self):
        self.attributes = Attributes()

    def get_fd_solution(self, q, dq, tau):
        M = self.attributes.get_mass_matrix(q)
        C = self.attributes.get_coriolis_matrix(q, dq)
        g = self.attributes.get_gravity_vector(q)
        f = self.attributes.get_friction_torques(dq)
        return get_fd_solution(M, C, g, f, dq, tau)

    def get_id_solution(self, q, dq, ddq):
        M = self.attributes.get_mass_matrix(q)
        C = self.attributes.get_coriolis_matrix(q, dq)
        g = self.attributes.get_gravity_vector(q)
        f = self.attributes.get_friction_torques(dq)
        return get_id_solution(M, C, g, f, dq, ddq)

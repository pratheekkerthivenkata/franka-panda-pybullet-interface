#!/usr/bin/env python

from franka_panda_pybullet_interface.robot.attributes import Attributes
from franka_panda_pybullet_interface.robot.limits import Limits

if __name__ == '__main__':
    limits = Limits()
    q = limits.sample_q()
    dq = limits.sample_dq()

    attributes = Attributes()
    print('Jacobian:\n', attributes.get_jacobian(q))
    print('\nJacobian Derivative (with q):\n', attributes.get_jacobian_derivative(dq, q=q))
    print('\nJacobian Derivative (with J):\n', attributes.get_jacobian_derivative(dq, J=attributes.get_jacobian(q)))
    print('\nMass Matrix:\n', attributes.get_mass_matrix(q))
    print('\nCoriolis Matrix:\n', attributes.get_coriolis_matrix(q, dq))
    print('\nGravity Vector:\n', attributes.get_gravity_vector(q))
    print('\nFriction Torques:\n', attributes.get_friction_torques(dq))
    print('\nManipulability: ', attributes.get_manipulability(q))
    print('\nDynamic Manipulability: ', attributes.get_dynamic_manipulability(q))

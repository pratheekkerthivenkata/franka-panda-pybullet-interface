import warnings

import numba as nb
import numpy as np
from numba import NumbaPerformanceWarning
from numba import njit
from numbalsoda import lsoda_sig

warnings.filterwarnings('ignore', category=NumbaPerformanceWarning)


@nb.cfunc(lsoda_sig)
def deriv(t, y_, dy, p_):
    y = nb.carray(y_, (14,))

    vals = nb.carray(p_, (119,))
    tau = vals[:7]
    M = vals[7:7 + 7 * 7].reshape((7, 7))
    C = vals[7 + 7 * 7:7 + 7 * 7 + 7 * 7].reshape((7, 7))
    g = vals[-14:-7]
    f = vals[-7:]

    dq = y[7:]

    ddq = np.linalg.solve(M, tau - C @ dq - g - f)

    dy[0] = dq[0]
    dy[1] = dq[1]
    dy[2] = dq[2]
    dy[3] = dq[3]
    dy[4] = dq[4]
    dy[5] = dq[5]
    dy[6] = dq[6]
    dy[7] = ddq[0]
    dy[8] = ddq[1]
    dy[9] = ddq[2]
    dy[10] = ddq[3]
    dy[11] = ddq[4]
    dy[12] = ddq[5]
    dy[13] = ddq[6]


@njit
def get_ddq(J, ee_acc_k, dq):
    return np.linalg.pinv(J) @ (ee_acc_k - get_jacobian_derivative(J, dq) @ dq)


@njit
def get_dq(J, ee_vel):
    return np.linalg.pinv(J) @ ee_vel


@njit
def get_tau(M, ddq, C, dq, g, f):
    return M @ ddq + C @ dq + g + f


def get_dh_params(q):
    return np.array([[0, 0.333, 0, q[0]],
                     [0, 0, -np.pi / 2, q[1]],
                     [0, 0.316, np.pi / 2, q[2]],
                     [0.0825, 0, np.pi / 2, q[3]],
                     [-0.0825, 0.384, -np.pi / 2, q[4]],
                     [0, 0, np.pi / 2, q[5]],
                     [0.088, 0, np.pi / 2, q[6]],
                     [0, 0.107, 0, 0],
                     [0, 0, 0, -np.pi / 4],
                     [0, 0.1034, 0, 0]], dtype=np.float64)


@njit
def get_tf_mat(i, dh):
    a = dh[i][0]
    d = dh[i][1]
    alpha = dh[i][2]
    theta = dh[i][3]

    tf_mat = np.zeros((4, 4), dtype=np.float64)

    tf_mat[0][0] = np.cos(theta)
    tf_mat[0][1] = -np.sin(theta)
    tf_mat[0][2] = 0
    tf_mat[0][3] = a
    tf_mat[1][0] = np.sin(theta) * np.cos(alpha)
    tf_mat[1][1] = np.cos(theta) * np.cos(alpha)
    tf_mat[1][2] = -np.sin(alpha)
    tf_mat[1][3] = -np.sin(alpha) * d
    tf_mat[2][0] = np.sin(theta) * np.sin(alpha)
    tf_mat[2][1] = np.cos(theta) * np.sin(alpha)
    tf_mat[2][2] = np.cos(alpha)
    tf_mat[2][3] = np.cos(alpha) * d
    tf_mat[3][0] = 0
    tf_mat[3][1] = 0
    tf_mat[3][2] = 0
    tf_mat[3][3] = 1

    return tf_mat


@njit
def vec_to_SO3(vec):
    """
    https://github.com/NxRLab/ModernRobotics/blob/b294cc9e6e94d7ee6d23a719972b822bebcad247/packages/Python/modern_robotics/core.py#L75
    """

    return np.array([[0, -vec[2], vec[1]],
                     [vec[2], 0, -vec[0]],
                     [-vec[1], vec[0], 0]], dtype=np.float64)


@njit
def get_adjoint(vec):
    """
    https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py#L832
    """

    mat = vec_to_SO3([vec[0], vec[1], vec[2]])
    row1 = np.concatenate((mat, np.zeros((3, 3), dtype=np.float64)), axis=1)
    row2 = np.concatenate((vec_to_SO3([vec[3], vec[4], vec[5]]), mat), axis=1)
    return np.concatenate((row1, row2), axis=0)


@njit
def get_jacobian(q):
    dh_params = np.array([[0, 0.333, 0, q[0]],
                          [0, 0, -np.pi / 2, q[1]],
                          [0, 0.316, np.pi / 2, q[2]],
                          [0.0825, 0, np.pi / 2, q[3]],
                          [-0.0825, 0.384, -np.pi / 2, q[4]],
                          [0, 0, np.pi / 2, q[5]],
                          [0.088, 0, np.pi / 2, q[6]],
                          [0, 0.107, 0, 0],
                          [0, 0, 0, -np.pi / 4],
                          [0.0, 0.1034, 0, 0]], dtype=np.float64)

    tf_mat = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]], dtype=np.float64)  # b/c np.eye and np.identity are not supported by numba
    for i in np.arange(0, 10):
        tf_mat = tf_mat @ get_tf_mat(i, dh_params)

    J = np.zeros((6, 10), dtype=np.float64)
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=np.float64)
    for joint_idx in np.arange(0, 10):
        T = T @ get_tf_mat(joint_idx, dh_params)

        p = tf_mat[:3, 3] - T[:3, 3]
        z = T[:3, 2]
        J[:3, joint_idx] = np.cross(z, p)
        J[3:, joint_idx] = z

    return J[:, :7]

@njit
def get_jacobian_derivative(dq, J):
    """
    Modern Robotics (Lynch, Park) Exercise 8.6
    """

    num_joints = J.shape[1]
    Jdot = np.zeros(J.shape, dtype=np.float64)
    for i in range(num_joints):
        for j in range(num_joints):
            if i > j:
                Ji = J[:, i]
                Jj = J[:, j]
                dJi_dqj = get_adjoint(Ji) @ Jj
                Jdot[:, i] += dJi_dqj * dq[j]
    return Jdot

@njit
def get_manipulability(q):
    J = get_jacobian(q)
    return np.sqrt(np.linalg.det(J @ J.T))


@njit
def get_dynamic_manipulability(q, M):
    # mass matrix is passed in as a parameter b/c numba doesn't support the panda_dynamics_model module
    J = get_jacobian(q)
    return np.sqrt(np.linalg.det(J @ np.linalg.inv(M) @ J.T))


@njit
def get_fd_solution(M, C, g, f, dq, tau):
    return np.linalg.solve(M, tau - C @ dq - g - f)


@njit
def get_id_solution(M, C, g, f, dq, ddq):
    return M @ ddq + C @ dq + g + f

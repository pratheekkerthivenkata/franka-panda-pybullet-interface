import numpy as np
import pybullet as pb

from ..utils.datatypes import Pose, Point, Quaternion, Velocity


class State:
    def __init__(self, robot):
        self.robot = robot

    @property
    def q(self):
        joint_states = pb.getJointStates(self.robot.robot_id, self.robot.joint_ids)
        return np.asarray([state[0] for state in joint_states])

    @property
    def dq(self):
        joint_states = pb.getJointStates(self.robot.robot_id, self.robot.joint_ids)
        return np.asarray([state[1] for state in joint_states])

    @property
    def tau(self):
        joint_states = pb.getJointStates(self.robot.robot_id, self.robot.joint_ids)
        return np.asarray([state[3] for state in joint_states])

    @property
    def ee_pose(self):
        ee_state = list(pb.getLinkState(self.robot.robot_id, self.robot.attributes.ee_link_id, computeLinkVelocity=1))
        return Pose(position=Point(*ee_state[0]), orientation=Quaternion(*ee_state[1]))

    @property
    def ee_velocity(self):
        ee_state = list(pb.getLinkState(self.robot.robot_id, self.robot.attributes.ee_link_id, computeLinkVelocity=1))
        return Velocity(linear=Point(x=ee_state[6][0], y=ee_state[6][1], z=ee_state[6][2]),
                        angular=Point(x=ee_state[7][0], y=ee_state[7][1], z=ee_state[7][2]))

    def is_gripper_open(self):
        finger_states = pb.getJointStates(self.robot.robot_id, self.robot.finger_joint_ids)
        return finger_states[0][0] > 0.3 and finger_states[1][0] > 0.3

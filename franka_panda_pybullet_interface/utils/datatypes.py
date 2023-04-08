from dataclasses import dataclass
from typing import Union

import numpy as np
import pybullet as pb


def convert_orientation(orientation, euler):
    if (type(orientation) == Point and euler) or (type(orientation) == Quaternion and not euler):
        return orientation
    elif type(orientation) == Point and not euler:
        return Quaternion(*pb.getQuaternionFromEuler(orientation.tolist()))
    elif type(orientation) == Quaternion and euler:
        return Point(*pb.getEulerFromQuaternion(orientation.tolist()))


@dataclass
class Point:
    x: float
    y: float
    z: float

    def tolist(self):
        return [self.x, self.y, self.z]

    def totuple(self):
        return self.x, self.y, self.z


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def tolist(self):
        return [self.x, self.y, self.z, self.w]

    def totuple(self):
        return self.x, self.y, self.z, self.w


@dataclass
class Pose:
    position: Point
    orientation: Union[Quaternion, Point]

    def tolist(self, flatten=False):
        ls = [self.position.tolist(), self.orientation.tolist()]
        if flatten:
            return [item for sublist in ls for item in sublist]
        return ls

    def totuple(self, flatten=False):
        ls = (self.position.totuple(), self.orientation.totuple())
        if flatten:
            return tuple([item for sublist in ls for item in sublist])
        return ls

    def tonode(self):
        ori = convert_orientation(self.orientation, euler=True)
        return Node(x=self.position.x, y=self.position.y, theta=ori.z)

    def convert_orientation(self, euler):
        self.orientation = convert_orientation(self.orientation, euler=euler)


@dataclass
class Velocity:
    linear: Point
    angular: Point

    def tolist(self):
        return [self.linear.tolist(), self.angular.tolist()]

    def totuple(self):
        return self.linear.totuple(), self.angular.totuple()


@dataclass
class Effort:
    force: Point
    torque: Point

    def tolist(self):
        return [self.force.tolist(), self.torque.tolist()]

    def totuple(self):
        return self.force.totuple(), self.torque.totuple()


@dataclass
class JointLimits:
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray
    dddq: np.ndarray
    tau: np.ndarray


@dataclass
class Node:
    x: float
    y: float
    theta: float  # radians

    def tolist(self):
        return [self.x, self.y, self.theta]

    def totuple(self):
        return self.x, self.y, self.theta

    def topose(self, z, euler):
        return Pose(position=Point(x=self.x, y=self.y, z=z),
                    orientation=convert_orientation(Point(x=0, y=0, z=self.theta), euler=euler))

    def distance(self, other):
        # only calculates positional distance
        return np.linalg.norm(np.array(self.totuple()[:2]) - np.array(other.totuple()[:2]))

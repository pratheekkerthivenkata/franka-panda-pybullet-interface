import pybullet as pb

from .datatypes import Point, Quaternion


def convert_orientation(orientation, euler):
    if (type(orientation) == Point and euler) or (type(orientation) == Quaternion and not euler):
        return orientation
    elif type(orientation) == Point and not euler:
        return Quaternion(*pb.getQuaternionFromEuler(orientation.tolist()))
    elif type(orientation) == Quaternion and euler:
        return Point(*pb.getEulerFromQuaternion(orientation.tolist()))

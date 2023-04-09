import os
from copy import deepcopy

import pybullet as pb

from .scene_object import SceneObject
from ..definitions import ASSETS_DIR
from ..utils import Print
from ..utils.datatypes import Pose, Point, Quaternion


class CollisionObject:
    def __init__(self, urdf_filename, obj_type, client_id):
        self.client_id = client_id

        self.__print = Print(__class__)
        self._id = None  # object ID in PyBullet
        self._link_name = None
        self._name = None
        self.urdf_filename = os.path.join(ASSETS_DIR, urdf_filename)
        assert os.path.exists(self.urdf_filename), 'File not found: {}'.format(self.urdf_filename)

        self.obj_type = obj_type
        self.pose = None
        self.node = None

    def __eq__(self, other):
        if not isinstance(other, CollisionObject) or not isinstance(self, SceneObject) or other is None:
            return False
        return self.name == other.name

    @property
    def link_name(self):
        return self._link_name

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    def is_fixed(self):
        return self.obj_type == 'fixed'

    def spawn(self, pose):
        assert type(pose) == Pose

        pose_copy = deepcopy(pose)
        pose_copy.convert_orientation(euler=False)
        try:
            self._id = pb.loadURDF(self.urdf_filename,
                                   pose_copy.position.tolist(), pose_copy.orientation.tolist(),
                                   useFixedBase=self.is_fixed(), physicsClientId=self.client_id)
        except Exception as e:
            print('Failed to load URDF: {}'.format(self.urdf_filename))
            raise e

        self.update_pose()
        self._link_name = pb.getJointInfo(self.id, 0, self.client_id)[12].decode('utf-8')
        self._name = self.urdf_filename.split('/')[-1].split('.')[0]

    def remove(self, client_id):
        assert self.id is not None, 'Object not spawned yet.'
        pb.removeBody(self.id, physicsClientId=client_id)
        self._id = None
        self.pose = None
        self.node = None

    def relocate(self, pose):
        assert self.id is not None, 'Object not spawned yet.'
        assert type(pose) == Pose

        pose_copy = deepcopy(pose)
        if 'tunnel' in self.urdf_filename:
            pose_copy.convert_orientation(euler=True)
            pose_copy.orientation.x = 0
            pose_copy.orientation.y = 0
        pose_copy.convert_orientation(euler=False)
        pb.resetBasePositionAndOrientation(self.id, pose.position.tolist(), pose.orientation.tolist(),
                                           physicsClientId=self.client_id)
        self.update_pose()

    def update_pose(self):
        self.pose = self.get_sim_pose(euler=False)
        self.node = self.pose.tonode()

    def get_sim_pose(self, euler):
        assert self.id is not None, 'Object not spawned yet.'

        position, orientation = pb.getBasePositionAndOrientation(self.id)
        pose = Pose(position=Point(*position), orientation=Quaternion(*orientation))
        pose.convert_orientation(euler=euler)
        return pose

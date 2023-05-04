from copy import deepcopy

import pybullet as pb

from ..utils import Print
from ..utils.datatypes import Pose, Point, Quaternion


class CollisionObject:
    def __init__(self, scene_obj, client_id):
        self.__print = Print(__class__)

        self.client_id = client_id
        self._id = None  # object ID in PyBullet
        self.urdf_filename = scene_obj.urdf_filename
        self.obj_type = scene_obj.obj_type
        self._link_names = scene_obj.link_names
        self._name = scene_obj.name

        self.pose = None
        self.node = None

    def __eq__(self, other):
        assert self.id is not None, 'Object not spawned yet.'
        assert other.id is not None, 'Other object not spawned yet.'

        if not isinstance(other, CollisionObject) or other is None:
            return False
        return self.id == other.id

    @property
    def link_names(self):
        return self._link_names

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    def spawn(self, pose):
        assert type(pose) == Pose

        pose_copy = deepcopy(pose)
        pose_copy.convert_orientation(euler=False)

        self._id = pb.loadURDF(self.urdf_filename,
                               pose_copy.position.tolist(), pose_copy.orientation.tolist(),
                               useFixedBase=self.obj_type == 'fixed', physicsClientId=self.client_id)
        self.__update_internal_state()

    def remove(self):
        assert self.id is not None, 'Object not spawned yet.'

        pb.removeBody(self.id, physicsClientId=self.client_id)

        self._id = None
        self.pose = None
        self.node = None

    def relocate(self, pose):
        assert self.id is not None, 'Object not spawned yet.'
        assert type(pose) == Pose

        pose_copy = deepcopy(pose)
        if 'tunnel' in self.urdf_filename or 'box' in self.urdf_filename:
            pose_copy.convert_orientation(euler=True)
            pose_copy.orientation.x = 0
            pose_copy.orientation.y = 0
        pose_copy.convert_orientation(euler=False)
        pb.resetBasePositionAndOrientation(self.id, pose.position.tolist(), pose.orientation.tolist(),
                                           physicsClientId=self.client_id)
        self.__update_internal_state()

    def get_sim_pose(self, euler=False):
        assert self.id is not None, 'Object not spawned yet.'

        position, orientation = pb.getBasePositionAndOrientation(self.id, physicsClientId=self.client_id)
        pose = Pose(position=Point(*position), orientation=Quaternion(*orientation))
        pose.convert_orientation(euler=euler)
        return pose

    def __update_internal_state(self):
        self.pose = self.get_sim_pose()
        self.node = self.pose.tonode()

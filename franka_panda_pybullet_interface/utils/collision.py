from itertools import combinations

import numpy as np
import pybullet as pb
from pyb_utils.collision import NamedCollisionObject, CollisionDetector

from ..envs.collision_object import CollisionObject
from ..envs.scene_object import SceneObject


class Collision:
    def __init__(self, robot, scene_objects):
        self.client_id = pb.connect(pb.DIRECT)
        self.robot = robot

        collision_robot_id = self.robot.load_robot(self.client_id)  # load robot in collision client
        self.collision_bodies = {'robot': collision_robot_id}

        self.mapper_dict = {link_name: NamedCollisionObject('robot', link_name) for link_name in self.robot.link_names}
        self.robot_self_collision_detector = self.__setup_robot_self_collisions()
        self.robot_object_collision_detector = None
        self.object_object_collision_detector = None

        self.collision_objects = []
        self.collision_obj_names = []
        self.scene_objects = []
        self.add_collision_objects(scene_objects)

    def reset(self):
        # remove all objects from collision client
        for obj in self.collision_objects:
            obj.remove(self.client_id)
            obj.collision_engine_obj_id = None
        self.collision_objects = []
        self.collision_obj_names = []
        self.scene_objects = []
        self.collision_bodies = {'robot': self.collision_bodies['robot']}
        self.mapper_dict = {link_name: NamedCollisionObject('robot', link_name) for link_name in self.robot.link_names}
        self.robot_object_collision_detector = None
        self.object_object_collision_detector = None

    def add_collision_objects(self, scene_objects):
        for scene_obj in scene_objects:
            if scene_obj.name in self.collision_obj_names:
                continue

            self.scene_objects.append(scene_obj)

            collision_obj = CollisionObject(scene_obj.urdf_filename, scene_obj.obj_type, self.client_id)
            collision_obj.spawn(scene_obj.pose)

            self.collision_objects.append(collision_obj)
            self.collision_obj_names.append(collision_obj.name)
            self.collision_bodies[collision_obj.name] = collision_obj.id
            self.mapper_dict[collision_obj.link_name] = \
                NamedCollisionObject(collision_obj.name, collision_obj.link_name)

            # TODO: fix for tunnel! multilink and box!

        self.update_collision_detectors()

    def remove_collision_objects(self, scene_objects):
        for scene_obj in scene_objects:
            if scene_obj.name not in self.collision_obj_names:
                continue

            self.scene_objects.remove(scene_obj)

            # find index of scene_obj.name in self.collision_obj_names
            idx = self.collision_obj_names.index(scene_obj.name)
            collision_obj = self.collision_objects[idx]

            self.collision_objects.pop(idx)
            self.collision_obj_names.pop(idx)
            self.collision_bodies.pop(collision_obj.name)
            self.mapper_dict.pop(collision_obj.link_name)

            collision_obj.remove(client_id=self.client_id)

            # TODO: fix for tunnel! multilink and box!

        self.update_collision_detectors()

    def update_collision_detectors(self):
        self.robot_object_collision_detector = self.__setup_robot_obj_collisions()
        self.object_object_collision_detector = self.__setup_obj_obj_collisions()

    def __setup_robot_self_collisions(self):
        collision_enabled_link_pairs = []
        all_combos = list(combinations(self.mapper_dict.keys(), 2))

        for combo in all_combos:
            val1, val2 = combo
            if [val1, val2] not in self.robot.self_collision_disabled_link_pairs and \
               [val2, val1] not in self.robot.self_collision_disabled_link_pairs:
                collision_enabled_link_pairs.append((self.mapper_dict[val1], self.mapper_dict[val2]))

        return CollisionDetector(self.client_id, self.collision_bodies, collision_enabled_link_pairs)

    def __setup_robot_obj_collisions(self):
        collision_enabled_link_pairs = []
        all_combos = list(combinations(self.mapper_dict.keys(), 2))
        # import pdb; pdb.set_trace()

        for combo in all_combos:
            val1, val2 = combo
            # want this function to detect collision between robot links and object links only,
            # not amongst the same object class
            if val1 == 'panda_link0' and val2 in ['table_wooden_link', 'table_ikea_link', 'wall', 'support_beam_link', 'clamps_link', 'planeLink']:
                continue
            if 'panda' in val1 and not 'panda' in val2:
                collision_enabled_link_pairs.append((self.mapper_dict[val1], self.mapper_dict[val2]))
        import pdb; pdb.set_trace()
        return CollisionDetector(self.client_id, self.collision_bodies, collision_enabled_link_pairs)

    def __setup_obj_obj_collisions(self):
        collision_enabled_link_pairs = []
        all_combos = list(combinations(self.mapper_dict.keys(), 2))

        for combo in all_combos:
            val1, val2 = combo
            # object only
            if 'panda' in val1 or 'panda' in val2:
                continue
            collision_enabled_link_pairs.append((self.mapper_dict[val1], self.mapper_dict[val2]))

        return CollisionDetector(self.client_id, self.collision_bodies, collision_enabled_link_pairs)

    def is_robot_in_self_collision(self, q=None):
        if q is None:
            q = self.robot.q

        assert type(q) == np.ndarray
        assert len(q) == 7

        joint_angles = self.__process_q(q)

        return self.robot_self_collision_detector.in_collision(joint_angles)

    def is_robot_in_collision_with_object(self, scene_object, q=None, scene_object_pose=None):
        if q is None:
            q = self.robot.q
        if scene_object_pose is None:
            scene_object_pose = scene_object.pose

        assert type(q) == np.ndarray
        assert len(q) == 7
        assert type(scene_object) == SceneObject

        joint_angles = self.__process_q(q)
        self.__update_pose_in_collision_engine(scene_object.name, scene_object_pose)

        return self.robot_object_collision_detector.in_collision(joint_angles)

    def is_robot_in_collision_with_env(self, q=None, scene_objects=None, scene_object_poses=None):
        if q is None:
            q = self.robot.q
        if scene_objects is None:
            scene_objects = self.scene_objects
        if scene_object_poses is None:
            scene_object_poses = [obj.pose for obj in scene_objects]

        assert type(q) == np.ndarray
        assert len(q) == 7

        for idx, obj in enumerate(self.scene_objects):
            if self.is_robot_in_collision_with_object(obj, q, scene_object_poses[idx]):
                return True

        return False

    def is_robot_collision_free(self, q=None):
        if q is None:
            q = self.robot.q

        assert type(q) == np.ndarray
        assert len(q) == 7

        return not self.is_robot_in_self_collision(q) and not self.is_robot_in_collision_with_env(q)

    def are_objects_in_collision(self, scene_object_1, scene_object_2, scene_object_1_pose=None, scene_object_2_pose=None):
        if scene_object_1_pose is None:
            scene_object_1_pose = scene_object_1.pose
        if scene_object_2_pose is None:
            scene_object_2_pose = scene_object_2.pose

        assert type(scene_object_1) == SceneObject
        assert type(scene_object_2) == SceneObject

        self.__update_pose_in_collision_engine(scene_object_1.name, scene_object_1_pose)
        self.__update_pose_in_collision_engine(scene_object_2.name, scene_object_2_pose)

        # argument is irrelevant for this function
        return self.object_object_collision_detector.in_collision(self.robot.default_joint_angles)

    def __update_pose_in_collision_engine(self, scene_object_name, scene_object_pose):
        idx = self.collision_obj_names.index(scene_object_name)
        self.collision_objects[idx].relocate(scene_object_pose)

    @staticmethod
    def __process_q(q):
        return [0] + q.tolist() + [0, 0, 0, 0, 0]

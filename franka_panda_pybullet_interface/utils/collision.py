from itertools import combinations

import numpy as np
import pybullet as pb
from pyb_utils.collision import NamedCollisionObject, CollisionDetector

from ..envs.scene_object import SceneObject


class Collision:
    def __init__(self, robot, scene_objects):
        self.client_id = pb.connect(pb.DIRECT)
        self.robot = robot
        self.scene_objects = scene_objects
        self.scene_obj_ids = [obj.id for obj in scene_objects]

        collision_robot_id = self.robot.load_robot(self.client_id)  # load robot in collision client
        self.collision_bodies = {'robot': collision_robot_id}

        self.mapper_dict = {link_name: NamedCollisionObject('robot', link_name) for link_name in self.robot.link_names}
        self.robot_self_collision_detector = self.__setup_robot_self_collisions()
        self.robot_object_collision_detector = None
        self.object_object_collision_detector = None

        self.add_scene_objects(self.scene_objects)

    def reset(self):
        # remove all objects from collision client
        self.scene_objects = []
        self.scene_obj_ids = []
        self.collision_bodies = {'robot': self.collision_bodies['robot']}
        self.mapper_dict = {link_name: NamedCollisionObject('robot', link_name) for link_name in self.robot.link_names}
        self.robot_object_collision_detector = None
        self.object_object_collision_detector = None

    def add_scene_objects(self, scene_objects):
        for obj in scene_objects:
            if obj.id in self.scene_obj_ids:
                continue

            obj_id = obj.spawn(obj.pose, self.client_id)

            self.scene_objects.append(obj)
            self.scene_obj_ids.append(obj_id)
            self.collision_bodies[obj.name] = obj_id
            self.mapper_dict[obj.link_name] = NamedCollisionObject(obj.name, obj.link_name)

            # TODO: fix for tunnel! multilink and box!

        self.update_collision_detectors()

    def remove_scene_objects(self, scene_objects):
        for obj in scene_objects:
            if obj.id not in self.scene_obj_ids:
                continue

            self.scene_objects.remove(obj)
            self.scene_obj_ids.remove(obj.id)
            self.collision_bodies.pop(obj.name)
            self.mapper_dict.pop(obj.link_name)

            obj.remove(client_id=self.client_id)

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

        for combo in all_combos:
            val1, val2 = combo
            # want this function to detect collision between robot links and object links only,
            # not amongst the same object class
            if 'panda' in val1 and 'panda' in val2:
                continue
            if 'object' in val1 and 'object' in val2:
                continue
            collision_enabled_link_pairs.append((self.mapper_dict[val1], self.mapper_dict[val2]))

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

    def is_robot_in_self_collision(self, q):
        assert type(q) == np.array
        assert len(q) == 7

        joint_angles = [0] + q.tolist() + [0, 0, 0, 0, 0]
        return self.robot_self_collision_detector.in_collision(joint_angles)

    def is_robot_in_collision_with_object(self, q, scene_object):
        assert type(q) == np.array
        assert len(q) == 7
        assert type(scene_object) == SceneObject

        joint_angles = [0] + q.tolist() + [0, 0, 0, 0, 0]

        scene_object.update_pose_in_collision_engine(client_id=self.client_id)
        return self.robot_object_collision_detector.in_collision(joint_angles)

    def is_robot_in_collision_with_env(self, q):
        assert type(q) == np.array
        assert len(q) == 7

        for obj in self.scene_objects:
            if self.is_robot_in_collision_with_object(q, obj):
                return True

        return False

    def is_robot_collision_free(self, q):
        assert type(q) == np.array
        assert len(q) == 7

        return not self.is_robot_in_self_collision(q) and not self.is_robot_in_collision_with_env(q)

    def are_objects_in_collision(self, scene_object_1, scene_object_2):
        assert type(scene_object_1) == SceneObject
        assert type(scene_object_2) == SceneObject

        scene_object_1.update_pose_in_collision_engine(client_id=self.client_id)
        scene_object_2.update_pose_in_collision_engine(client_id=self.client_id)

        # argument is irrelevant for this function
        return self.object_object_collision_detector.in_collision(self.robot.default_joint_angles)

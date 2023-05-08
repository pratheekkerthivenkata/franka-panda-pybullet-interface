import os
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb

from ..utils import Print
from ..utils.datatypes import Pose, Point, Quaternion, Node


class SceneObject:
    def __init__(self, urdf_filename, obj_type, moveit_interface, client_id, name_suffix='', object_z=None, apriltag_id=None, apriltag_family=None, camera=None, node_lower_bound=None, node_upper_bound=None):
        """
        Parameters
        ----------
        urdf_filename [str]: relative path inside assets/
                             eg: 'realsense_box.urdf', 'ycb/YcbBanana/model.urdf', etc.
        obj_type [str]: 'fixed', 'movable', 'target'
        node_lower_bound [Node]: lower bound for sampling object poses
        node_upper_bound [Node]: upper bound for sampling object poses
        obj_height [float]: copied from the urdf file; TODO: get this from the urdf file automatically
        table_surface_z [float]
        """

        self.moveit = moveit_interface
        self.client_id = client_id
        self.name_suffix = name_suffix
        self._link_names = []
        self._name = None

        self.__print = Print(__class__)

        assert os.path.exists(urdf_filename), 'File not found: {}'.format(urdf_filename)
        assert os.path.isabs(urdf_filename), 'Please provide absolute path to the URDF file.'
        self.urdf_filename = urdf_filename

        self._id = None  # object ID in PyBullet
        self.obj_type = obj_type
        self.node_lower_bound = node_lower_bound
        self.node_upper_bound = node_upper_bound
        self.object_z = object_z

        self.pose = None
        self.node = None

        self.patch = None

        self.apriltag_id = apriltag_id
        self.apriltag_family = apriltag_family
        self.camera = camera

    def __eq__(self, other):
        assert self.id is not None, 'Object not spawned yet.'
        assert other.id is not None, 'Other object not spawned yet.'

        if not isinstance(other, SceneObject) or other is None:
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

    def is_target(self):
        return self.obj_type == 'target'

    def is_fixed(self):
        return self.obj_type == 'fixed'

    def is_base(self):
        for name in ['table', 'wall', 'clamp', 'beam', 'plane']:
            if name in self.urdf_filename:
                return True
        return False

    def get_link_names(self):
        return []

    @property
    def size(self):
        assert self.id is not None, f'Object ({self.name}) not spawned yet.'
        return pb.getVisualShapeData(self.id)[0][3]

    @property
    def friction(self):
        assert self.id is not None, 'Object not spawned yet.'
        return pb.getDynamicsInfo(self.id, -1)[1]

    def get_xy_bounds(self, obj_pose=None, padding=0.0):
        # TODO: not quite right..., need tight bounding box

        assert self.id is not None, 'Object not spawned yet.'
        if obj_pose is None:
            obj_pose = self.pose
        assert type(obj_pose) == Pose

        x_min = obj_pose.position.x - self.size[0] / 2 - padding
        x_max = obj_pose.position.x + self.size[0] / 2 + padding
        y_min = obj_pose.position.y - self.size[1] / 2 - padding
        y_max = obj_pose.position.y + self.size[1] / 2 + padding

        obj_pose.convert_orientation(euler=True)
        # orientation = convert_orientation(obj_pose.orientation, euler=True)
        rotation_matrix = np.array([[np.cos(obj_pose.orientation.z), -np.sin(obj_pose.orientation.z)],
                                    [np.sin(obj_pose.orientation.z), np.cos(obj_pose.orientation.z)]])
        rotated_x_bounds = np.matmul(rotation_matrix, np.array([[x_min], [x_max]]))
        rotated_y_bounds = np.matmul(rotation_matrix, np.array([[y_min], [y_max]]))

        return (rotated_x_bounds[0], rotated_x_bounds[1]), (rotated_y_bounds[0], rotated_y_bounds[1])

    def is_within_bounds(self, bounds, pose=None):
        assert self.id is not None, 'Object not spawned yet.'
        if pose is None:
            # self.update_pose()
            pose = self.pose
        return bounds[0][0] <= pose.position.x <= bounds[0][1] and \
               bounds[1][0] <= pose.position.y <= bounds[1][1]

    def do_objects_overlap(self, other_obj, tolerance=0.01):
        assert self.id is not None, f'Object ({self.name}) not spawned yet.'
        assert other_obj.id is not None, f'Object ({other_obj.name}) not spawned yet.'
        return len(pb.getClosestPoints(self.id, other_obj.id, tolerance)) > 0

    def in_collision(self, objects):
        assert self.id is not None, 'Object not spawned yet.'
        for obj in objects:
            assert type(obj) == SceneObject
            if obj.id == self.id:
                continue
            if self.do_objects_overlap(obj):
                return True
        return False

    def spawn(self, pose=None, other_objects=None):
        node = Node(x=np.Inf, y=np.Inf, theta=np.Inf)
        if pose is None:
            node.sample(self.node_lower_bound, self.node_upper_bound)
            pose = node.topose(z=self.object_z, euler=False)

        assert type(pose) == Pose
        pose_copy = deepcopy(pose)
        pose_copy.convert_orientation(euler=False)
        # try:
            # print(self.urdf_filename, pose_copy)
        print(self.urdf_filename)
        self._id = pb.loadURDF(self.urdf_filename,
                               pose_copy.position.tolist(), pose_copy.orientation.tolist(),
                               useFixedBase=self.is_fixed(), physicsClientId=self.client_id)
        # except Exception as e:
        #     print('Failed to load URDF: {}'.format(self.urdf_filename))
        #     raise e

        # respawn if in collision
        if other_objects is not None:
            while self.in_collision(other_objects):# or \
                    # (goal_node is not None and self.is_fixed() and self.node.distance(goal_node) < goal_radius):
                node.sample(self.node_lower_bound, self.node_upper_bound)
                self.relocate(node.topose(z=self.object_z, euler=True))

        self.__update_internal_state()

        self.moveit.add_object_to_scene(self)
        link_info = pb.getBodyInfo(self._id, physicsClientId=self.client_id)
        self._link_names.append(link_info[0].decode('utf-8'))  # base link
        for joint_idx in range(pb.getNumJoints(self._id, physicsClientId=self.client_id)):
            link_name = pb.getJointInfo(self._id, joint_idx, physicsClientId=self.client_id)[12].decode('utf-8')
            self._link_names.append(link_name)
        self._name = link_info[1].decode('utf-8').split('.urdf')[0] + self.name_suffix

    def remove(self, client_id):
        assert self.id is not None, 'Object not spawned yet.'
        pb.removeBody(self.id, physicsClientId=client_id)
        self._id = None
        self.pose = None
        self.node = None

    def relocate(self, pose, other_objects=None):
        assert self.id is not None, 'Object not spawned yet.'
        assert type(pose) == Pose

        node = Node(x=np.Inf, y=np.Inf, theta=np.Inf)
        pose_copy = deepcopy(pose)
        if 'tunnel' in self.urdf_filename or 'box' in self.urdf_filename:
            pose_copy.convert_orientation(euler=True)
            pose_copy.orientation.x = 0
            pose_copy.orientation.y = 0
        pose_copy.convert_orientation(euler=False)
        pb.resetBasePositionAndOrientation(self.id, pose_copy.position.tolist(), pose_copy.orientation.tolist(),
                                           physicsClientId=self.client_id)
        # respawn if in collision
        if other_objects is not None:
            while self.in_collision(other_objects):
                node.sample(self.node_lower_bound, self.node_upper_bound)
                self.relocate(node.topose(z=self.object_z, euler=True), other_objects=other_objects)
        # self.update_pose()
        self.__update_internal_state()
        self.moveit.relocate_object_in_scene(self)

    def get_sim_pose(self, euler=False):
        assert self.id is not None, 'Object not spawned yet.'

        position, orientation = pb.getBasePositionAndOrientation(self.id)
        pose = Pose(position=Point(*position), orientation=Quaternion(*orientation))
        pose.convert_orientation(euler=euler)
        return pose

    def get_real_pose(self, euler=False):
        assert not self.is_base(), f'Cannot get AprilTag pose for base objects (here, {self._name}).'
        assert self.apriltag_id is not None
        assert self.apriltag_family is not None
        assert self.camera is not None

        while True:
            pose = self.camera.get_apriltag_pose_in_robot_frame(self.apriltag_id, self.apriltag_family, euler=True)
            if pose is None:
                self.__print.print_error(f'Object with AprilTag ID {self.apriltag_id} not detected.\n'
                                         f'Reposition and press Enter to continue...')
                input()
            else:
                break

        if 'tunnel' in self.urdf_filename:
            pose.position.z -= 0.33655/2
        else:
            # pose.position.z -= self.size[2] / 2  # pose corresponds to object's geometric center in PyBullet
            pose.position.z = self.object_z
            pose.orientation.x = 0
            pose.orientation.y = 0
        pose.convert_orientation(euler=euler)
        return pose

    def get_corner_pts(self, obj_pose=None, size=None):
        assert self.id is not None, 'Object not spawned yet.'
        if size is None:
            obj_size = self.size
        else:
            obj_size = size
        if obj_pose is None:
            obj_pose = self.pose
        obj_pose.convert_orientation(euler=True)

        # TODO: generalize; this bit of code currently assumes all objects are rectangular
        corner_pts_at_origin = np.array([(obj_size[0] / 2, obj_size[1] / 2), (obj_size[0] / 2, -obj_size[1] / 2),
                                         (-obj_size[0] / 2, -obj_size[1] / 2), (-obj_size[0] / 2, obj_size[1] / 2)])
        rotation_matrix = np.array([[np.cos(obj_pose.orientation.z), -np.sin(obj_pose.orientation.z)],
                                    [np.sin(obj_pose.orientation.z), np.cos(obj_pose.orientation.z)]])
        rotated_corner_pts_at_origin = np.matmul(rotation_matrix, corner_pts_at_origin.T).T
        corner_pts = rotated_corner_pts_at_origin + np.array([obj_pose.position.x, obj_pose.position.y])
        return corner_pts

    def get_contour_pts(self, obj_pose=None, corner_pts=None):
        assert self.id is not None, 'Object not spawned yet.'

        if corner_pts is None:
            corner_pts = self.get_corner_pts(obj_pose)
        contour_pts = []
        for i in range(len(corner_pts)):
            pt1 = corner_pts[i]
            pt2 = corner_pts[(i + 1) % len(corner_pts)]
            contour_pts.extend(np.linspace(pt1, pt2, num=20))
        return np.array(contour_pts)

    def plot_contour_in_sim(self, obj_pose=None, timeout=0, c=(0, 0, 1), thickness=1):
        assert self.id is not None, 'Object not spawned yet.'

        if obj_pose is None:
            obj_pose = self.pose

        corner_pts = self.get_corner_pts(obj_pose)
        for pt1, pt2 in zip(corner_pts, corner_pts[1:]):
            pb.addUserDebugLine(pt1.tolist(), pt2.tolist(), c, thickness, timeout)
        pb.addUserDebugLine(corner_pts[-1].tolist(), corner_pts[0].tolist(), c, thickness, timeout)

    def plot_contour_in_matplotlib(self, ax, color=None, fill=False, alpha=1, obj_pose=None):
        assert self.id is not None, 'Object not spawned yet.'

        if obj_pose is None:
            obj_pose = self.pose

        ec = color

        if color is None:
            if self.obj_type == 'target':
                fill = False
                color = 'yellow'
                ec = color
                alpha = 1
            elif self.obj_type == 'fixed':
                fill = True
                color = 'red'
                ec = color
                alpha = 1
            elif self.obj_type == 'movable':
                fill = True
                color = 'white'
                ec = 'black'
                alpha = 0.2

        patch = plt.Polygon(np.fliplr(self.get_corner_pts(obj_pose)), closed=True,
                            fill=fill, facecolor=color, edgecolor=ec, linewidth=1, alpha=alpha)
        ax.add_patch(patch)
        plt.show(block=False)

    def update_sim_from_real(self):
        real_pose = self.get_real_pose()
        if real_pose is None:
            self.__print.print_error(f'Object with AprilTag ID {self.apriltag_id} not detected.')
            return False
        self.relocate(real_pose)

    def __update_internal_state(self):
        self.pose = self.get_sim_pose()
        self.node = self.pose.tonode()

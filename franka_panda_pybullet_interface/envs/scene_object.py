import os

import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb

from env_definitions import ASSETS_DIR
from logger import PrintLogger
from npm_base import Point, Quaternion, Pose, convert_orientation
import transformations as tf


class SceneObject:
    def __init__(self, urdf_filename, obj_type, node_lower_bound, node_upper_bound, sim, object_z=None, obj_height=None, table_surface_z=None, apriltag_id=None, apriltag_family=None, camera=None):
        """
        Parameters
        ----------
        urdf_filename [str]: relative path inside robot-sim-envs/robot_sim_envs/assets
                             eg: 'realsense_box.urdf', 'ycb/YcbBanana/model.urdf', etc.
        obj_type [str]: 'fixed', 'movable', 'target'
        node_lower_bound [Node]: lower bound for sampling object poses
        node_upper_bound [Node]: upper bound for sampling object poses
        obj_height [float]: copied from the urdf file; TODO: get this from the urdf file automatically
        table_surface_z [float]
        """

        self.__printer = PrintLogger(__class__)
        # self.__printer.print_warning('Only rectangular objects supported in this version.')

        self.id = None
        self.collision_engine_obj_id = None
        self.link_name = None
        self.urdf_filename = os.path.join(ASSETS_DIR, urdf_filename)
        assert os.path.exists(self.urdf_filename), 'File not found: {}'.format(self.urdf_filename)

        self.obj_type = obj_type
        self.node_lower_bound = node_lower_bound
        self.node_upper_bound = node_upper_bound
        self.object_z = object_z
        if self.object_z is None:
            assert obj_height is not None
            assert table_surface_z is not None
            self.object_z = table_surface_z + obj_height / 2
        self.sim = sim

        self.pose = None
        self.node = None

        self.patch = None

        self.conds = 'table' in self.urdf_filename or \
                'wall' in self.urdf_filename or \
                'clamp' in self.urdf_filename or \
                'beam' in self.urdf_filename or \
                'plane' in self.urdf_filename

        if not self.sim and not self.conds:
            assert apriltag_id is not None
            assert apriltag_family is not None
            assert camera is not None

            self.apriltag_id = apriltag_id
            self.apriltag_family = apriltag_family
            self.camera = camera

    def __eq__(self, other):
        if not isinstance(other, SceneObject):
            return False
        return self.id == other.id

    def get_xy_bounds(self, obj_pose=None, padding=0.0):
        # TODO: not quite right..., need tight bounding box

        assert self.id is not None, 'Object not spawned yet.'
        if obj_pose is None:
            obj_pose = self.pose
        assert type(obj_pose) == Pose

        size = self.get_size()

        x_min = obj_pose.position.x - size[0] / 2 - padding
        x_max = obj_pose.position.x + size[0] / 2 + padding
        y_min = obj_pose.position.y - size[1] / 2 - padding
        y_max = obj_pose.position.y + size[1] / 2 + padding

        orientation = convert_orientation(obj_pose.orientation, euler=True)
        rotation_matrix = np.array([[np.cos(orientation.z), -np.sin(orientation.z)],
                                    [np.sin(orientation.z), np.cos(orientation.z)]])
        rotated_x_bounds = np.matmul(rotation_matrix, np.array([[x_min], [x_max]]))
        rotated_y_bounds = np.matmul(rotation_matrix, np.array([[y_min], [y_max]]))

        return (rotated_x_bounds[0], rotated_x_bounds[1]), (rotated_y_bounds[0], rotated_y_bounds[1])

    def is_within_bounds(self, bounds, pose=None):
        assert self.id is not None, 'Object not spawned yet.'
        if pose is None:
            self.update_pose()
            pose = self.pose
        return bounds[0][0] <= pose.position.x <= bounds[0][1] and \
               bounds[1][0] <= pose.position.y <= bounds[1][1]

    def is_target(self):
        return self.obj_type == 'target'

    def is_fixed(self):
        return self.obj_type == 'fixed'

    def get_link_names(self):
        return []

    def update_pose(self, pose=None):
        if pose is None:
            if self.sim or self.conds:
                self.pose = self.get_sim_pose(euler=False)
            else:
                self.pose = self.get_real_pose(euler=False)
        if self.pose is None:
            self.node = None
            return
        self.node = self.pose.tonode()
        # self.relocate(self.pose)

    def get_size(self):
        assert self.id is not None, 'Object not spawned yet.'
        return pb.getVisualShapeData(self.id)[0][3]

    def do_objects_overlap(self, other_obj, tolerance=0.01):
        assert self.id is not None, 'Object not spawned yet.'
        assert other_obj.id is not None, 'Object not spawned yet.'
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

    def spawn_in_collision_engine(self, client_id):
        assert self.pose is not None
        orientation = convert_orientation(self.pose.orientation, euler=False).tolist()
        self.collision_engine_obj_id = pb.loadURDF(self.urdf_filename, self.pose.position.tolist(), orientation, useFixedBase=self.is_fixed(), physicsClientId=client_id)
        self.link_name = pb.getJointInfo(self.collision_engine_obj_id, 0, client_id)[12].decode('utf-8')
        return self.collision_engine_obj_id

    def update_pose_in_collision_engine(self, client_id):
        assert self.collision_engine_obj_id is not None
        orientation = convert_orientation(self.pose.orientation, euler=False).tolist()
        pb.resetBasePositionAndOrientation(self.collision_engine_obj_id, self.pose.position.tolist(), orientation, physicsClientId=client_id)

    def spawn(self, pose, other_objects=None, goal_node=None, goal_radius=None, moveit_interface=None):
        assert self.id is None, 'Object already spawned.'
        assert type(pose) == Pose
        assert moveit_interface is not None
        # make sure orientation is in quaternion form
        orientation = convert_orientation(pose.orientation, euler=False).tolist()
        try:
            self.id = pb.loadURDF(self.urdf_filename, pose.position.tolist(), orientation, useFixedBase=self.is_fixed())
        except Exception as e:
            print('Failed to load URDF: {}'.format(self.urdf_filename))
            import pdb; pdb.set_trace()
            raise e

        # respawn if in collision
        if other_objects is not None:
            while self.in_collision(other_objects):# or \
                    # (goal_node is not None and self.is_fixed() and self.node.distance(goal_node) < goal_radius):
                self.relocate(self.sample_pose(), moveit_interface=moveit_interface)

        self.update_pose()

        moveit_interface.add_object_to_scene(self)

    def remove(self):
        assert self.id is not None, 'Object not spawned yet.'
        pb.removeBody(self.id)
        self.id = None
        self.pose = None
        self.node = None

    def relocate(self, pose, other_objects=None, moveit_interface=None):
        assert self.id is not None, 'Object not spawned yet.'
        assert type(pose) == Pose
        assert moveit_interface is not None
        if 'tunnel' in self.urdf_filename:
            orientation = convert_orientation(pose.orientation, euler=True)
            orientation.x = 0
            orientation.y = 0
            orientation = convert_orientation(orientation, euler=False).tolist()
        else:
            orientation = convert_orientation(pose.orientation, euler=False).tolist()
        pb.resetBasePositionAndOrientation(self.id, pose.position.tolist(), orientation)
        # respawn if in collision
        if other_objects is not None:
            while self.in_collision(other_objects):  # or \
                # (goal_node is not None and self.is_fixed() and self.node.distance(goal_node) < goal_radius):
                self.relocate(self.sample_pose(), moveit_interface=moveit_interface)
        self.update_pose()
        moveit_interface.relocate_object_in_scene(self)

    def sample_pose(self):
        return Pose(position=Point(x=np.random.uniform(self.node_lower_bound.x, self.node_upper_bound.x),
                                   y=np.random.uniform(self.node_lower_bound.y, self.node_upper_bound.y),
                                   z=self.object_z),
                    orientation=Point(x=0, y=0,
                                      z=np.random.uniform(self.node_lower_bound.theta, self.node_upper_bound.theta)))

    def get_sim_pose(self, euler):
        assert self.id is not None, 'Object not spawned yet.'
        position, orientation = pb.getBasePositionAndOrientation(self.id)
        orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        return Pose(position=Point(x=position[0], y=position[1], z=position[2]),
                    orientation=convert_orientation(orientation, euler))

    def get_real_pose(self, euler):
        # assert self.id is not None, 'Object not spawned yet.'
        assert not self.sim

        while True:
            pose = self.camera.get_apriltag_pose_in_robot_frame(self.apriltag_id, self.apriltag_family, euler=True)
            if pose is None:
                self.__printer.print_error(f'Object w/ AprilTag ID {self.apriltag_id} not detected.')
                input('Reposition object and press Enter to continue...')
            else:
                break

        if 'tunnel' in self.urdf_filename:
            pose.position.z -= 0.33655/2
            orientation = convert_orientation(pose.orientation, euler=euler)
        else:
            # pose.position.z += -0.065  # self.get_size()[2] / 2  # pose corresponds to object's geometric center in pybullet
            # pose.position.x = 0
            # pose.position.y = 0
            pose.position.z += 0.1
            orientation = convert_orientation(pose.orientation, euler=euler)
        return Pose(position=pose.position, orientation=orientation)

    def update_sim_pose_from_real(self):
        # assert self.id is not None, 'Object not spawned yet.'
        # self.relocate(self.get_real_pose())
        return NotImplementedError

    def get_corner_pts(self, obj_pose=None, size=None):
        assert self.id is not None, 'Object not spawned yet.'
        if size is None:
            obj_size = self.get_size()
        else:
            obj_size = size
        if obj_pose is None:
            obj_pose = self.pose
        obj_pose.orientation = convert_orientation(obj_pose.orientation, euler=True)

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

    def get_friction(self):
        assert self.id is not None, 'Object not spawned yet.'

        return pb.getDynamicsInfo(self.id, -1)[1]

    def plot_contour_in_sim(self, obj_pose=None, timeout=0, c=(0, 0, 1), thickness=1):
        assert self.id is not None, 'Object not spawned yet.'

        if obj_pose is None:
            obj_pose = self.pose

        corner_pts = self.get_corner_pts(obj_pose)
        print(corner_pts)

        # for i in range(len(corner_pts)):
        #     print(tuple(corner_pts[i]), corner_pts[(i + 1) % len(corner_pts)])
        #     pb.addUserDebugLine(list(corner_pts[i]), list(corner_pts[(i + 1) % len(corner_pts)]), c, thickness, timeout)
        for pt1, pt2 in zip(corner_pts, corner_pts[1:]):
            print(pt1.tolist(), pt2.tolist())
            pb.addUserDebugLine(pt1.tolist(), pt2.tolist(), c, thickness, timeout)
        pb.addUserDebugLine(corner_pts[-1].tolist(), corner_pts[0].tolist(), c, thickness, timeout)

    def plot_contour_in_matplotlib(self, ax, color=None, fill=False, alpha=1, obj_pose=None, thickness=1):
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

        # ax.scatter(self.node.y, self.node.x, c=color, marker='.', s=100)
        # print(obj_pose, self.get_sim_pose(euler=True))
        patch = plt.Polygon(np.fliplr(self.get_corner_pts(obj_pose)), closed=True,
                            fill=fill, facecolor=color, edgecolor=ec, linewidth=1, alpha=alpha)
        ax.add_patch(patch)
        plt.show(block=False)

    def get_distance(self, other):
        # TODO: return theta distance too; or combined position and orientation distance?
        assert type(other) == SceneObject
        assert self.id is not None, 'Object not spawned yet.'
        assert other.id is not None, 'Other object not spawned yet.'
        return np.linalg.norm(np.array(self.get_sim_pose(euler=True).position.tolist()) -
                              np.array(other.get_sim_pose(euler=True).position.tolist()))

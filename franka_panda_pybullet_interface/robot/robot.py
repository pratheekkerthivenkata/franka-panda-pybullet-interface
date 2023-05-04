import os

import pybullet as pb

from .attributes import Attributes
from .dynamics import Dynamics
from .kinematics import Kinematics
from .limits import Limits
from .motion import Motion
from .moveit import MoveIt
from .state import State
from .trajectory import Trajectory
from ..definitions import CONFIG_DIR, ASSETS_DIR
from ..utils.datatypes import Pose, Point, Quaternion
from ..utils.file_io import load_yaml


class Robot:
    def __init__(self, client_id, enable_realtime, timestep):
        self.enable_realtime = enable_realtime
        self.timestep = timestep
        self.client_id = client_id
        self.limits = Limits()
        self.metadata = load_yaml(os.path.join(CONFIG_DIR, 'robot.yaml'))
        self.urdf_path = os.path.join(ASSETS_DIR, self.metadata['urdf_path'])
        self.kinematics = Kinematics(self.urdf_path, self.metadata['base_link'], self.metadata['tip_link'])
        self.motion = Motion(self)
        self.sim_steps = 0
        self.self_collision_checker = None

        self.base_pose = Pose(position=Point(*self.metadata['base_pose']['position']),
                              orientation=Quaternion(*self.metadata['base_pose']['orientation']))

        self.robot_id = self.load_robot(self.client_id)
        self.attributes = Attributes(self.robot_id, self.metadata)
        self.current_state = State(self)
        self.dynamics = Dynamics(self.attributes)
        self.moveit = MoveIt()
        self.trajectory = Trajectory(self.moveit, self.limits)

        # add constraint between fingers to center the grip
        c = pb.createConstraint(self.robot_id, self.finger_joint_ids[0],
                                self.robot_id, self.finger_joint_ids[1],
                                jointType=pb.JOINT_GEAR, jointAxis=[1, 0, 0],
                                parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
        pb.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=500)

    def __getattr__(self, name):
        # instead of doing robot.current_state.q, we can do robot.q
        # and similarly for members of attributes, kinematics, dynamics, and motion

        if hasattr(self.current_state, name):
            return getattr(self.current_state, name)

        if hasattr(self.attributes, name):
            return getattr(self.attributes, name)

        if hasattr(self.kinematics, name):
            return getattr(self.kinematics, name)

        if hasattr(self.dynamics, name):
            return getattr(self.dynamics, name)

        if hasattr(self.motion, name):
            return getattr(self.motion, name)

        if hasattr(self.trajectory, name):
            return getattr(self.trajectory, name)

        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    def load_robot(self, client_id):
        # disable rendering to speed up URDF loading
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
        robot_id = pb.loadURDF(self.urdf_path,
                               basePosition=self.base_pose.position.tolist(),
                               baseOrientation=self.base_pose.orientation.tolist(), useFixedBase=True,
                               flags=pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                               physicsClientId=client_id)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

        return robot_id

    def reset_to_default_pose(self):
        self.move_to_q(self.default_q, direct=True)
        self.close_gripper()

    def reset(self):
        self.reset_to_default_pose()
        self.sim_steps = 0

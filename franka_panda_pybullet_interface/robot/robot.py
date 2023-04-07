import os

import pybullet as pb

from .state import State
from .attributes import Attributes
from .dynamics import Dynamics
from .kinematics import Kinematics
from .limits import Limits
from .motion import Motion
from ..definitions import CONFIG_DIR, ASSETS_DIR
from ..utils.file_io import load_yaml
from ..utils.datatypes import Pose, Point, Quaternion


class Robot:
    def __init__(self):
        self.current_state = State()
        self.limits = Limits()
        self.attributes = Attributes(self)
        metadata = load_yaml(os.path.join(CONFIG_DIR, 'robot.yaml'))
        self.kinematics = Kinematics(metadata['urdf_path'], metadata['base_link'], metadata['tip_link'])
        self.dynamics = Dynamics()
        self.motion = Motion(self)

        self.base_pose = Pose(position=Point(*metadata['base_pose']['position']),
                              orientation=Quaternion(*metadata['base_pose']['orientation']))
        self.urdf_path = os.path.join(ASSETS_DIR, metadata['urdf_path'])

        self.robot_id = self.load_robot(self.sim_id)

        # add constraint between fingers to center the grip
        c = pb.createConstraint(self.robot_id, self.gripper_joint_ids[0],
                                self.robot_id, self.gripper_joint_ids[1],
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

        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    def update_current_state(self, state):
        self.current_state = state
        self.attributes.update_state(state)

    def load_robot(self, sim_id):
        # disable rendering to speed up URDF loading
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
        robot_id = pb.loadURDF(self.urdf_path,
                               basePosition=self.base_pose.position.tolist(),
                               baseOrientation=self.base_pose.orientation.tolist(), useFixedBase=True,
                               flags=pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                               physicsClientId=sim_id)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

        return robot_id

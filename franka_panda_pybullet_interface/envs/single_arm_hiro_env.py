import os

from .robot_env import RobotEnv
from .scene_object import SceneObject
from ..definitions import CONFIG_DIR, ASSETS_DIR
from ..utils.datatypes import Pose, Point, Node
from ..utils.file_io import load_yaml


class SingleArmHIROEnv(RobotEnv):
    def __init__(self, enable_gui, enable_realtime):
        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        super(SingleArmHIROEnv, self).__init__(enable_gui=self.enable_gui, enable_realtime=self.enable_realtime)

        self.env_name = 'PandaEnv'
        
        env_metadata = load_yaml(os.path.join(CONFIG_DIR, 'single_arm_hiro_env.yaml'))

        self.table_wooden = \
            self.__setup_env(env_metadata['table_wooden']['filename'],
                             env_metadata['table_wooden']['position'], env_metadata['table_wooden']['orientation'])
        self.table_ikea = \
            self.__setup_env(env_metadata['table_ikea']['filename'],
                             env_metadata['table_ikea']['position'], env_metadata['table_ikea']['orientation'])
        self.back_wall = \
            self.__setup_env(env_metadata['back_wall']['filename'],
                             env_metadata['back_wall']['position'], env_metadata['back_wall']['orientation'])
        self.support_beams = \
            self.__setup_env(env_metadata['support_beams']['filename'],
                             env_metadata['support_beams']['position'], env_metadata['support_beams']['orientation'])
        self.clamps = \
            self.__setup_env(env_metadata['clamps']['filename'],
                             env_metadata['clamps']['position'], env_metadata['clamps']['orientation'])
        self.plane = \
            self.__setup_env(env_metadata['plane']['filename'],
                             env_metadata['plane']['position'], env_metadata['plane']['orientation'])

        self.base_objects = [self.table_wooden, self.table_ikea, self.back_wall, self.support_beams, self.clamps, self.plane]
        self.movable_objects = []
        self.fixed_obstacles = []

        self.collision_checker.add_collision_objects(self.base_objects)

    def __setup_env(self, urdf_filename, position, orientation):
        assert len(orientation) == 3  # euler
        obj = SceneObject(urdf_filename=os.path.join(ASSETS_DIR, urdf_filename), obj_type='fixed', sim=True,
                          moveit_interface=self.moveit_interface, collision_checker=self.collision_checker, client_id=self.sim_id)
        obj_pose = Pose(position=Point(*position), orientation=Point(*orientation))
        obj.spawn(pose=obj_pose, client_id=self.sim_id)
        return obj

    def reset_single_arm_hiro_env(self):
        super().reset_robot_env()
        self.collision_checker.add_collision_objects(self.base_objects)

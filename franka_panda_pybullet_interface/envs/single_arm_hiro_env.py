from .robot_env import RobotEnv
from .scene_object import SceneObject
from ..utils.datatypes import Pose, Point, Node
from ..utils.file_io import load_yaml
import os
from ..definitions import CONFIG_DIR


class SingleArmHIROEnv(RobotEnv):
    def __init__(self, enable_gui, enable_realtime):
        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        super(SingleArmHIROEnv, self).__init__(enable_gui=self.enable_gui, enable_realtime=self.enable_realtime)

        self.env_name = 'PandaEnv'
        
        env_metadata = load_yaml(os.path.join(CONFIG_DIR, 'single_arm_hiro_env.yaml'))

        self.table_wooden = self.__setup_env(env_metadata['table_wooden_panda']['filename'],
                                             env_metadata['table_wooden_panda']['position'],
                                             env_metadata['table_wooden_panda']['orientation'])
        self.table_ikea = self.__setup_env(env_metadata['table_ikea_panda']['filename'],
                                           env_metadata['table_ikea_panda']['position'],
                                           env_metadata['table_ikea_panda']['orientation'])
        self.back_wall = self.__setup_env(env_metadata['back_wall_panda']['filename'],
                                          env_metadata['back_wall_panda']['position'],
                                          env_metadata['back_wall_panda']['orientation'])
        self.support_beams = self.__setup_env(env_metadata['support_beams_panda']['filename'],
                                              env_metadata['support_beams_panda']['position'],
                                              env_metadata['support_beams_panda']['orientation'])
        self.clamps = self.__setup_env(env_metadata['clamps_panda']['filename'],
                                       env_metadata['clamps_panda']['position'],
                                       env_metadata['clamps_panda']['orientation'])
        self.plane = self.__setup_env(env_metadata['plane']['filename'],
                                      env_metadata['plane']['position'],
                                      env_metadata['plane']['orientation'])

        self.base_objects = []
        self.movable_objects = []
        self.fixed_obstacles = []

    def __setup_env(self, urdf_filename, position, orientation):
        assert len(orientation) == 3  # euler
        obj = SceneObject(urdf_filename=urdf_filename, obj_type='fixed',
                          node_lower_bound=Node(x=position[0], y=position[1], theta=orientation[2]),
                          node_upper_bound=Node(x=position[0], y=position[1], theta=orientation[2]),
                          object_z=position[2], sim=self.sim)
        obj_pose = Pose(position=Point(*position), orientation=Point(*orientation))
        # obj.spawn(obj.sample_pose())
        obj.spawn(obj_pose, moveit_interface=self.robot.moveit_interface)
        return obj

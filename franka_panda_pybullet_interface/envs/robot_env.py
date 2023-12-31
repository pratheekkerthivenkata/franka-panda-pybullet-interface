from .sim_env import SimEnv
from ..robot.robot import Robot
from ..utils.collision import Collision


class RobotEnv(SimEnv):
    def __init__(self, enable_gui, enable_realtime):
        """
        Adds a robot to a pre-spawned PyBullet instance.
        """

        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        super(RobotEnv, self).__init__(enable_gui=self.enable_gui, enable_realtime=self.enable_realtime)
        self.env_name = 'RobotEnv'

        self.robot = Robot(client_id=self.sim_id, enable_realtime=self.enable_realtime, timestep=self.timestep)
        self.moveit_interface = self.robot.moveit
        self.collision_checker = Collision(self.robot, [])
        self.robot.self_collision_checker = self.collision_checker.is_robot_in_self_collision
        self.reset_robot_env()

    def reset_robot_env(self):
        self.robot.reset()
        self.collision_checker.reset()

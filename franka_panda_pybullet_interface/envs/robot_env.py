__metaclass__ = type

from logger import PrintLogger
from ..robot.robot import Robot
from .sim_env import SimEnv


class RobotEnv(SimEnv):
    def __init__(self, enable_gui, enable_realtime):
        """
        Adds a robot to a pre-spawned PyBullet instance.
        """

        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        super(RobotEnv, self).__init__(enable_gui=self.enable_gui, enable_realtime=self.enable_realtime)
        self.env_name = 'RobotEnv'

        self.robot = None

        self.reset()

    def spawn_robot(self):
        self.robot = Robot(client_id=self.sim_id, enable_realtime=self.enable_realtime)
        if not self.enable_realtime:
            # total number of executed simulation steps
            self.robot.set_sim_steps(0)

    def reset(self):
        super().reset()

        # reset robot joint configuration to home state
        self.robot.move_to_default_pose(using_planner=False)
        self.robot.operate_gripper(open_=False)

        # reset number of simulation steps
        self.robot.set_sim_steps(0)

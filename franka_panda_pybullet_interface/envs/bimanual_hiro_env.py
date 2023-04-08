from .robot_env import RobotEnv


class BimanualHIROEnv(RobotEnv):
    def __init__(self, enable_gui, enable_realtime):
        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        super(BimanualHIROEnv, self).__init__(enable_gui=self.enable_gui, enable_realtime=self.enable_realtime)

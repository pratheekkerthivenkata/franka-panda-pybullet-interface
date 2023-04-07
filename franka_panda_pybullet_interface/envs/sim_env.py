import os

import pybullet as pb

from ..definitions import CONFIG_DIR
from ..utils.file_io import load_yaml


class SimEnv(object):
    def __init__(self, enable_gui, enable_realtime):
        """
        Spawns and sets up a PyBullet instance.

        Parameters
        ----------
        enable_gui : bool
        enable_realtime : bool
        """

        self.enable_gui = enable_gui
        self.enable_realtime = enable_realtime
        self.env_name = 'SimEnv'

        # load simulation parameters
        self.sim_metadata = load_yaml(os.path.join(CONFIG_DIR, 'sim.yaml'))
        self.timestep = self.sim_metadata['timestep']

        self.sim_id = self.setup_sim()
        self.reset()
        
    def setup_sim(self):
        if self.enable_gui:
            sim_options = f"--width={self.sim_metadata['width']} --height={self.sim_metadata['height']} " \
                          f"--background_color_red={self.sim_metadata['color'][0]} " \
                          f"--background_color_green={self.sim_metadata['color'][1]} " \
                          f"--background_color_blue={self.sim_metadata['color'][2]}"
            sim_id = pb.connect(pb.GUI_SERVER, options=sim_options)

            # GUI setup
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
            pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
            pb.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            pb.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            pb.resetDebugVisualizerCamera(
                cameraDistance=self.sim_metadata['view']['distance'],
                cameraYaw=self.sim_metadata['view']['yaw'],
                cameraPitch=self.sim_metadata['view']['pitch'],
                cameraTargetPosition=self.sim_metadata['view']['target_position']
            )
        else:
            sim_id = pb.connect(pb.DIRECT)  # no GUI

        return sim_id

    def reset(self):
        pb.resetSimulation()

        pb.setGravity(self.sim_metadata['gravity'][0], self.sim_metadata['gravity'][1], self.sim_metadata['gravity'][2])
        pb.setRealTimeSimulation(int(self.enable_realtime))
        if not self.enable_realtime:
            pb.setTimeStep(self.timestep)

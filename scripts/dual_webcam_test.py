#!/usr/bin/env python

from franka_panda_pybullet_interface.vision import DualWebcam

if __name__ == '__main__':
    cam = DualWebcam()
    # print(cam.get_apriltag_pose_in_robot_frame(6, 'tag25h9', euler=True))
    cam.visualize_apriltags_by_family('tag25h9')

from franka_panda_pybullet_interface.vision import DualWebcam


if __name__ == '__main__':
    cam = DualWebcam()
    cam.visualize_apriltags_by_family('tag25h9')

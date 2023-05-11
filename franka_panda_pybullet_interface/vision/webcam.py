import os

import cv2
import numpy as np
import pyudev
import transformations as tf

from .apriltag import AprilTag
from ..definitions import CONFIG_DIR
from ..utils.datatypes import Pose, Point, Quaternion
from ..utils.file_io import load_yaml


class Webcam:
    def __init__(self, cam):
        self.cam = cam
        cam_params = load_yaml(os.path.join(CONFIG_DIR, 'webcam.yaml'))

        # open stream
        self.cap = cv2.VideoCapture(self.__get_device_index(cam_params[self.cam]['serial_number']))
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

        # intrinsics
        self.img_dims = (cam_params[self.cam]['image_width'], cam_params[self.cam]['image_height'])
        self.new_camera_matrix = np.array(cam_params[self.cam]['camera_matrix']).reshape((3, 3))
        self.dist_coeffs = np.array(cam_params[self.cam]['distortion_coefficients'])
        self.newcameramatrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.new_camera_matrix, self.dist_coeffs, self.img_dims, 1, self.img_dims
        )
        self.intrinsics = [self.newcameramatrix[0][0], self.newcameramatrix[1][1],
                           self.newcameramatrix[0][2], self.newcameramatrix[1][2]]

        if cam == 'left':
            self.robot_base_to_cam_tf = [[0.999682, 0.002808, 0.025043, 0.442848],
                                         [0.002739, -0.999992, 0.002779, -0.551149],
                                         [0.025050, -0.002709, -0.999683, 1.232196],
                                         [0, 0, 0, 1]]
        # else:
        #     self.robot_base_to_cam_tf = [[0.996656, -0.080057, 0.016368, 0.481453],
        #                                  [-0.080422, -0.996496, 0.022994, 0.121881],
        #                                  [0.014470, -0.024233, -0.999602, 1.158621],
        #                                  [0, 0, 0, 1]]
            self.robot_base_to_cam_tf = np.array(self.robot_base_to_cam_tf)
        else:
            # extrinsics
            cam_pose = Pose(position=Point(*cam_params[self.cam]['pose_in_robot_frame']['position']),
                            orientation=Quaternion(*cam_params[self.cam]['pose_in_robot_frame']['orientation']))
            cam_pose.convert_orientation(euler=True)
            self.robot_base_to_cam_tf = tf.compose_matrix(translate=cam_pose.position.tolist(),
                                                          angles=cam_pose.orientation.tolist())

        self.apriltag = AprilTag()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    @staticmethod
    def __get_device_index(serial_number):
        context = pyudev.Context()
        video_devices = context.list_devices(subsystem='video4linux')

        for device in video_devices:
            try:
                parent = device.find_parent('usb', 'usb_device')
            except pyudev.DeviceNotFound:
                continue

            serial = parent.attributes.asstring('serial')
            if serial == serial_number:
                return int(device.sys_name[5:])

    def get_rgb_img(self):
        while True:
            ret, rgb_img = self.cap.read()
            if ret:
                break

        undistorted_image = cv2.undistort(
            rgb_img, self.new_camera_matrix, self.dist_coeffs, None, self.newcameramatrix
        )

        return undistorted_image

    def visualize_rgb_frame(self):
        while True:
            cv2.imshow(f'rgb_{self.cam}', self.get_rgb_img())
            if cv2.waitKey(1) == ord('q'):
                break

    def visualize_apriltags_by_family(self, family):
        while True:
            img = self.get_rgb_img()
            _, overlay = self.apriltag.get_data(img, family, self.intrinsics,
                                                visualize=True, verbose=True, annotate=True)
            cv2.imshow(f'apriltags_{self.cam}', img)
            cv2.imshow(f'apriltags_{self.cam}', overlay)
            if cv2.waitKey(1) == ord('q'):
                break

    def get_apriltag_pose_in_robot_frame(self, tag_id, family, euler=False):
        pose_mat = self.apriltag.get_pose(self.get_rgb_img(), tag_id, family, self.intrinsics, matrix=True)
        if pose_mat is None:
            return None
        pose_mat = self.robot_base_to_cam_tf @ pose_mat

        pose = Pose(position=Point(*tf.translation_from_matrix(pose_mat)),
                    orientation=Point(*tf.euler_from_matrix(pose_mat)))
        pose.convert_orientation(euler=euler)
        return pose

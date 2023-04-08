import os

import cv2
import numpy as np
import transformations as tf
from pyudev import Context, DeviceNotFound

from .apriltag import AprilTag
from ..definitions import CONFIG_DIR
from ..utils.datatypes import Pose, Point, Quaternion
from ..utils.file_io import load_yaml


class Webcam:
    def __init__(self, cam):
        cam_params = load_yaml(os.path.join(CONFIG_DIR, 'webcam.yaml' % cam))

        # open stream
        self.cap = cv2.VideoCapture(self.__get_device_index(cam_params[cam]['serial_number']))
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

        # intrinsics
        self.img_dims = (cam_params[cam]['image_width'], cam_params[cam]['image_height'])
        self.new_camera_matrix = np.array(cam_params[cam]['camera_matrix']['data']).reshape((3, 3))
        self.dist_coeffs = np.array(cam_params[cam]['distortion_coefficients']['data'])
        self.newcameramatrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.new_camera_matrix, self.dist_coeffs, self.img_dims, 1, self.img_dims
        )
        self.intrinsics = [self.newcameramatrix[0][0], self.newcameramatrix[1][1],
                           self.newcameramatrix[0][2], self.newcameramatrix[1][2]]

        # extrinsics
        cam_pose = Pose(position=Point(*cam_params[cam]['pose_in_robot_frame']['position']),
                        orientation=Quaternion(*cam_params[cam]['pose_in_robot_frame']['orientation']))
        cam_pose.convert_orientation(euler=True)
        self.robot_base_to_cam_tf = tf.compose_matrix(translate=cam_pose.position.tolist(),
                                                      angles=cam_pose.orientation.tolist())

        self.apriltag = AprilTag()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    @staticmethod
    def __get_device_index(serial_number):
        context = Context()
        video_devices = context.list_devices(subsystem='video4linux')

        for device in video_devices:
            try:
                parent = device.find_parent('usb', 'usb_device')
            except DeviceNotFound:
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
            cv2.imshow('rgb', self.get_rgb_img())
            if cv2.waitKey(1) == ord('q'):
                break

    def visualize_apriltags_by_family(self, family):
        while True:
            img = self.get_rgb_img()
            _, overlay = self.apriltag.get_data(img, family, self.intrinsics,
                                                visualize=True, verbose=True, annotate=True)
            cv2.imshow('rgb', img)
            cv2.imshow('rgb', overlay)
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

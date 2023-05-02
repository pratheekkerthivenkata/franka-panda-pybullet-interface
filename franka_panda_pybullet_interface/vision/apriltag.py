import os
import sys

import transformations as tf

sys.path.append(os.path.join(os.path.expanduser('~'), 'AprilTag/scripts'))
import apriltag

from ..utils.datatypes import Pose, Point, Quaternion
import numpy as np
from ..utils.misc import quaternion_avg_markley


class AprilTag:
    def __init__(self):
        options = apriltag.DetectorOptions(border=0)
        options.families = 'tag36h11'
        self.detector_36h11 = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

        options = apriltag.DetectorOptions(border=0)
        options.families = 'tag25h9'
        self.detector_tag25h9 = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

    def __get_detector(self, family):
        if family == 'tag36h11':
            return self.detector_36h11
        elif family == 'tag25h9':
            return self.detector_tag25h9
        else:
            raise ValueError('Invalid family: {}'.format(family))

    @staticmethod
    def __get_tag_size(family):
        return [0.07, 0.0355][family == 'tag36h11']

    def get_data(self, img, family, intrinsics, visualize=False, verbose=False, annotate=False):
        detector = self.__get_detector(family)
        tag_size = self.__get_tag_size(family)

        return apriltag.detect_tags(img, detector, camera_params=intrinsics, tag_size=tag_size,
                                    vizualization=[0, 3][visualize], verbose=[0, 3][verbose], annotation=annotate)

    def get_pose(self, img, tag_id, family, intrinsics, matrix=False, euler=False, num_samples=2):
        positions = []
        orientations = []
        num_valid_detections = 0
        while num_valid_detections < num_samples:
            results, _ = self.get_data(img, family, intrinsics,
                                       visualize=False, verbose=False, annotate=False)
            pose_mat = None
            for idx, r in enumerate(results):
                if type(r) == apriltag.Detection and r.tag_id == tag_id:
                    pose_mat = results[idx + 1]
                    break
            if pose_mat is None:
                return None

            positions.append(tf.translation_from_matrix(pose_mat))
            quat = tf.quaternion_from_matrix(pose_mat)
            orientations.append([quat[1], quat[2], quat[3], quat[0]])

            num_valid_detections += 1

        avg_position = np.mean(positions, axis=0)
        avg_orientation = quaternion_avg_markley(np.array(orientations), [1] * len(orientations))
        pose = Pose(position=Point(*avg_position), orientation=Quaternion(*avg_orientation))

        if matrix:
            pose.convert_orientation(euler=True)
            return tf.compose_matrix(translate=pose.position.tolist(), angles=pose.orientation.tolist())

        pose.convert_orientation(euler=euler)
        return pose

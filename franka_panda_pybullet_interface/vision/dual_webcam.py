import cv2
import numpy as np
import transformations as tf

from .webcam import Webcam


class DualWebcam:
    def __init__(self):
        self.cam_left = Webcam(cam='left')
        self.cam_right = Webcam(cam='right')

        self.cam_left_position = tf.translation_from_matrix(self.cam_left.robot_base_to_cam_tf)
        self.cam_right_position = tf.translation_from_matrix(self.cam_right.robot_base_to_cam_tf)

    def visualize_rgb_frames(self):
        while True:
            cv2.imshow(f'rgb_{self.cam_left.cam}', self.cam_left.get_rgb_img())
            cv2.imshow(f'rgb_{self.cam_right.cam}', self.cam_right.get_rgb_img())
            if cv2.waitKey(1) == ord('q'):
                break

    @staticmethod
    def __visualize_apriltags_by_family(cam, family):
        img = cam.get_rgb_img()
        _, overlay = cam.apriltag.get_data(img, family, cam.intrinsics,
                                           visualize=True, verbose=True, annotate=True)
        cv2.imshow(f'apriltags_{cam.cam}', img)
        cv2.imshow(f'apriltags_{cam.cam}', overlay)

    def visualize_apriltags_by_family(self, family):
        while True:
            self.__visualize_apriltags_by_family(self.cam_left, family)
            self.__visualize_apriltags_by_family(self.cam_right, family)
            if cv2.waitKey(1) == ord('q'):
                break

    def get_apriltag_pose_in_robot_frame(self, tag_id, family, euler=False):
        left_pose = self.cam_left.get_apriltag_pose_in_robot_frame(tag_id, family, euler=euler)
        right_pose = self.cam_right.get_apriltag_pose_in_robot_frame(tag_id, family, euler=euler)

        if left_pose is None and right_pose is None:
            return None
        elif left_pose is None:
            pose = right_pose
        elif right_pose is None:
            pose = left_pose
        else:
            # find which cam the apriltag is closest to and use that cam for detection
            # b/c pose estimation is more accurate closer to camera center, despite distortion correction

            left_dist = np.linalg.norm(np.array(left_pose.position.tolist()) - self.cam_left_position)
            right_dist = np.linalg.norm(np.array(right_pose.position.tolist()) - self.cam_right_position)

            if left_dist <= right_dist:
                pose = left_pose
            else:
                pose = right_pose

        pose.convert_orientation(pose.orientation, euler)
        return pose

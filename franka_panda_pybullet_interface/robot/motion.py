import numpy as np
import time
import pybullet as pb
import rospy
from panda_sim_real_interface.msg import JointDataArray
from panda_sim_real_interface.srv import RobotTrajectory
from std_srvs.srv import Empty


class Motion:
    def __init__(self, robot):
        self.robot = robot

    def __set_joint_angles(self, q):
        pb.setJointMotorControlArray(self.robot.robot_id, self.robot.joint_ids, pb.POSITION_CONTROL,
                                     targetPositions=q, forces=[500] * self.robot.num_joints)
        if not self.robot.enable_realtime:
            for _ in range(int(1 / self.robot.timestep)):
                pb.stepSimulation()

    def __has_reached_q(self, q):
        err = np.linalg.norm(q - self.robot.current_state.q, ord=np.inf)
        return err <= 1e-2

    def move_to_q(self, q, direct=False):
        # if direct:
        #     self.__disable_env_collisions()
        #     self.__set_joint_angles(q)
        #     self.__enable_env_collisions()
        # else:
        pb.setJointMotorControlArray(self.robot.robot_id, self.robot.joint_ids,
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPositions=q)

        while not self.__has_reached_q(q):
            if self.robot.enable_realtime:
                continue
            else:
                pb.stepSimulation()
                self.robot.sim_steps += 1

        return True

    def execute_q_trajectory(self, q_trajectory, timestamps):
        prev_timestamp = None
        for t, q in zip(timestamps, q_trajectory):
            if prev_timestamp is not None:
                assert t - prev_timestamp == self.robot.timestep
                prev_timestamp = t

            success = self.move_to_q(q, direct=False)
            if not success:
                return False

        return True

    def execute_q_trajectory_real(self, q_trajectory, timestamps):
        # wait for ROS service to be ready
        rospy.wait_for_service('execute_trajectory')
        execute_trajectory = rospy.ServiceProxy('execute_trajectory', RobotTrajectory)

        try:
            q_traj = [JointDataArray(list(q)) for q in q_trajectory]
            resp = execute_trajectory(timestamps, q_traj)
            return resp.success
        except rospy.ServiceException as e:
            print(f'Service call failed: {e}')
            return False

    @staticmethod
    def move_to_default_q_real():
        rospy.wait_for_service('robot_phone_home')
        home = rospy.ServiceProxy('robot_phone_home', Empty)
        home()

    def move_to_ee_pose(self, ee_pose, direct=False):
        q = self.robot.kinematics.get_ik_solution(ee_pose, seed_q=self.robot.state.q)
        if q is None:
            return False
        return self.move_to_q(q, direct=direct)

    def execute_ee_pose_trajectory(self, ee_pose_trajectory, timestamps):
        prev_timestamp = None
        for t, ee_pose in zip(timestamps, ee_pose_trajectory):
            if prev_timestamp is not None:
                assert t - prev_timestamp == self.robot.timestep
                prev_timestamp = t

            success = self.move_to_ee_pose(ee_pose, direct=False)
            if not success:
                return False

        return True

    def __execute_dq_command(self, dq):
        pb.setJointMotorControlArray(self.robot.robot_id, self.robot.joint_ids, pb.VELOCITY_CONTROL,
                                     targetVelocities=dq, forces=self.robot.limits.tau[:, 1])
        if not self.robot.enable_realtime:
            pb.stepSimulation()
            self.robot.sim_steps += 1
        else:
            time.sleep(0.001)

    def execute_dq_trajectory(self, dq_trajectory, q_trajectory, timestamps):
        prev_timestamp = None
        for t, dq, q in zip(timestamps, dq_trajectory, q_trajectory):
            if prev_timestamp is not None:
                assert t - prev_timestamp == self.robot.timestep
                prev_timestamp = t

            # while not self.__has_reached_q(q):
            self.__execute_dq_command(dq)
        # make robot stop moving by switching to position control mode
        self.__set_joint_angles(self.robot.q)#q_trajectory[-1])
        return True

    def execute_ee_velocity(self, ee_velocity, duration):
        timestamps = np.arange(0, duration, self.robot.timestep)
        for _ in timestamps:
            dq = self.robot.kinematics.get_jacobian(self.robot.state.q) @ ee_velocity
            self.__execute_dq_command(dq)

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def open_gripper_real(self):
        rospy.wait_for_service('open_gripper')
        open_gripper = rospy.ServiceProxy('open_gripper', Empty)
        open_gripper()

    def close_gripper_real(self):
        rospy.wait_for_service('close_gripper')
        close_gripper = rospy.ServiceProxy('close_gripper', Empty)
        close_gripper()

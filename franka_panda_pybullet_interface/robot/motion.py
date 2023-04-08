import numpy as np
import pybullet as pb


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
        if direct:
            # self.__disable_env_collisions()
            self.__set_joint_angles(q)
            # self.__enable_env_collisions()
        else:
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

            success = self.move_to_q(q, direct=False)
            if not success:
                return False

        return True

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

    def execute_dq_trajectory(self, dq_trajectory, timestamps):
        prev_timestamp = None
        for t, dq in zip(timestamps, dq_trajectory):
            if prev_timestamp is not None:
                assert t - prev_timestamp == self.robot.timestep

            self.__execute_dq_command(dq)

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

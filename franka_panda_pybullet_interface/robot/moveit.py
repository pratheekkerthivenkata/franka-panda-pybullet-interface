import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class MoveIt:
    def __init__(self):
        rospy.init_node('moveit_pybullet', anonymous=True)
        self.__robot = moveit_commander.RobotCommander()
        self.__scene = moveit_commander.PlanningSceneInterface()
        self.__group = moveit_commander.MoveGroupCommander('panda_arm')

    def _create_ros_joint_msg(self, q):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.__robot.get_joint_names(group='panda_arm')[:-1]
        joint_state.position = q.tolist()
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        return joint_state, moveit_robot_state

    @staticmethod
    def __get_object_name(scene_object):
        return 'object_' + str(scene_object.id)

    def attach_object_to_ee(self, scene_object):
        touch_links = self.__robot.get_link_names(group='panda_hand')
        self.__scene.attach_box('panda_hand', self.__get_object_name(scene_object), touch_links=touch_links)

    def detach_object_from_ee(self, scene_object):
        self.__scene.remove_attached_object('panda_hand', name=self.__get_object_name(scene_object))

    def add_object_to_scene(self, scene_object):
        object_pose = PoseStamped()
        object_pose.header.frame_id = 'panda_link0'
        object_pose.pose.position.x = scene_object.pose.position.x
        object_pose.pose.position.y = scene_object.pose.position.y
        object_pose.pose.position.z = scene_object.pose.position.z
        object_pose.pose.orientation.x = scene_object.pose.orientation.x
        object_pose.pose.orientation.y = scene_object.pose.orientation.y
        object_pose.pose.orientation.z = scene_object.pose.orientation.z
        object_pose.pose.orientation.w = scene_object.pose.orientation.w
        object_name = self.__get_object_name(scene_object)
        if 'tunnel' in scene_object.urdf_filename:
            object_pose.pose.position.z += 0.05
            self.__scene.add_box(object_name, object_pose, size=(0.08, 0.5, 0.24))
        elif 'box' in scene_object.urdf_filename:
            self.__scene.add_box(object_name, object_pose, size=(0.24, 0.24, 0.24))
        else:
            self.__scene.add_box(object_name, object_pose, size=scene_object.get_size())
        return object_name

    def remove_object_from_scene(self, scene_object):
        object_name = self.__get_object_name(scene_object)
        self.__scene.remove_world_object(object_name)

    def relocate_object_in_scene(self, scene_object):
        self.remove_object_from_scene(scene_object)
        self.add_object_to_scene(scene_object)

    def execute_plan(self, plan):
        # plan = self.get_plan(q1, q2)
        self.__group.execute(plan, wait=True)

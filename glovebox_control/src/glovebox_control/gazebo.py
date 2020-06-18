import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel
from geometry_msgs.msg import Pose
from math import pi
import time
from threading import Timer


class GazeboControl(object):
    __current_model_name = "cricket_ball"
    __path_to_models = "/root/.gazebo/models/"

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics and services
        and resets the world to start in a good position.
        """
        if not rospy.get_name()==None:
            rospy.init_node("gazebo_control")


        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/pause_physics")
        self.__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self.__set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        rospy.wait_for_service("/gazebo/delete_model")
        self.__delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.__spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)


    def reset_world(self):
        """
        Resets the object poses in the world and the robot joint angles.
        """
        self.__pause_physics.call()

        joint_names = ['joint1', 'joint2', 'joint3',
                       'joint4', 'joint5', 'joint6', 'joint7', 'finger_joint']
        joint_positions = [0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]

        self.__set_model.call(model_name="glovebox_left",
                              urdf_param_name="robot_description",
                              joint_names=joint_names,
                              joint_positions=joint_positions)


        time.sleep(0.1)
        self.__unpause_physics.call()

        self.__reset_world.call()

    def get_object_pose(self):
        """
        Gets the pose of the ball in the world frame.
        @return The pose of the ball.
        """
        return self.__get_pose_srv.call(self.__current_model_name, "world").pose


    def swap_object(self, new_model_name):
        """
        Replaces the current object with a new one.Replaces
        @new_model_name the name of the folder in which the object is (e.g. beer)
        """
        try:
            self.__delete_model(self.__current_model_name)
        except:
            rospy.logwarn("Failed to delete: " + self.__current_model_name)
        try:
            sdf = None
            initial_pose = Pose()
            initial_pose.position.x = 0.15
            initial_pose.position.z = 0.82
            with open(self.__path_to_models + new_model_name + "/model.sdf", "r") as model:
                sdf = model.read()
            res = self.__spawn_model(new_model_name, sdf, "", initial_pose, "world")
            rospy.logerr("RES: " + str(res))
            self.__current_model_name = new_model_name
        except:
            rospy.logwarn("Failed to delete: " + self.__current_model_name)


    def pause(self):
        """
        Pause physics in Gazebo.
        """
        self.__pause_physics.call()

    def unpause(self):
        """
        Unpause physics in Gazebo.
        """
        self.__unpause_physics.call()

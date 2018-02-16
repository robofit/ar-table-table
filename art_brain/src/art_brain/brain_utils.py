import matplotlib.path as mplPath
import numpy as np
import rospy
import actionlib
from art_msgs.msg import PickPlaceAction, ObjInstance, ArmNavigationAction, \
    ArmNavigationGoal, ArmNavigationFeedback, ArmNavigationResult
import copy
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Pose
from art_msgs.msg import InterfaceState, InstancesArray
from geometry_msgs.msg import PoseStamped
import random

from enum import IntEnum  # sudo pip install enum34


class ArtBrainUtils(object):

    @staticmethod
    def get_pick_obj(obj_type, objects):
        rospy.logdebug("Object to pick: " + str(obj_type))
        if obj_type is None:
            return None
        obj_type = obj_type[0]
        for obj in objects.instances:

            if obj.object_id == obj_type:
                obj_ret = copy.deepcopy(obj)
                return obj_ret
        else:
            return None

    @staticmethod
    def get_pick_obj_from_feeder(obj_type):
        assert isinstance(obj_type, str)
        obj = ObjInstance()
        obj.object_id = None
        obj.object_type = obj_type
        obj.pose = Pose()
        return obj

    @staticmethod
    def get_objects_in_polygon(obj_type, polygon, objects):
        assert isinstance(obj_type, str)

        pick_polygon = []
        pol = None
        obj_ret = []
        if obj_type is None or len(obj_type) == 0:
            rospy.logerr("what the fuck?")
            return []
        # obj_type = obj_type[0]
        # TODO check frame_id and transform to table frame?
        for point in polygon.polygon.points:
            pick_polygon.append([point.x, point.y])
        pick_polygon.append([0, 0])

        if len(pick_polygon) > 0:
            pol = mplPath.Path(np.array(pick_polygon), closed=True)

        for obj in objects.instances:
            if obj.object_type != obj_type:
                continue
            # test if some object is in polygon and take the first one
            if pol.contains_point([obj.pose.position.x, obj.pose.position.y]):
                obj_ret.append(copy.deepcopy(obj))

        return obj_ret

    @staticmethod
    def get_pick_obj_from_polygon(obj_type, polygon, objects):
        arr = ArtBrainUtils.get_objects_in_polygon(obj_type, polygon, objects)
        random.shuffle(arr)
        if len(arr) == 0:
            rospy.logerr('No object in the specified polygon')
            return None

        rospy.logdebug('Selected object: ' + arr[0].object_id)
        return arr[0]

    @staticmethod
    def object_exist(obj_id, objects):
        """
        Checks if object id exists

        Args:
            obj_id: id of the object
            objects: array of detected objects

        @type obj_id: str
        @type objects: InstancesArray

        Returns:

        @rtype: bool

        """
        for o in objects.instances:  # type: ObjInstance
            if o.object_id == obj_id:
                return True
        else:
            return False

    @staticmethod
    def get_place_pose(instruction):
        return instruction.pose

    @staticmethod
    def distance_2d(pose1, pose2):
        a = np.array((pose1.position.x, pose1.position.y))
        b = np.array((pose2.position.x, pose2.position.y))
        return np.linalg.norm(a - b)

    @staticmethod
    def create_service_client(service_name, service_type, print_info=True):
        '''
        Waits for service (eventually informs user about it) and return an instance of it
        Args:
            service_name:
            service_type:
            print_info:

        Returns: created service proxy

        '''
        if print_info:
            rospy.loginfo("Waiting for service: " + str(service_name))
        rospy.wait_for_service(service_name)
        if print_info:
            rospy.loginfo("Service " + str(service_name) + " ready")
        return rospy.ServiceProxy(service_name, service_type)

    @staticmethod
    def normalize_frame_id(frame_id):
        if frame_id[0] == '/':
            return frame_id[1:]
        else:
            return frame_id


class ArtBrainErrorSeverities(IntEnum):
    WARNING = InterfaceState.WARNING
    ERROR = InterfaceState.ERROR
    SEVERE = InterfaceState.SEVERE
    INFO = InterfaceState.INFO


class ArtBrainErrors(IntEnum):
    ERROR_UNKNOWN = 0
    ERROR_NOT_IMPLEMENTED = 1
    ERROR_NOT_EXECUTING_PROGRAM = 2
    ERROR_NO_INSTRUCTION = 3
    ERROR_NO_PROGRAM_HELPER = 4
    ERROR_OBJECT_NOT_DEFINED = 5
    ERROR_GRIPPER_PP_CLIENT_MISSING = 6
    ERROR_PICK_POSE_NOT_SELECTED = 8
    ERROR_PLACE_POSE_NOT_DEFINED = 9
    ERROR_NO_PICK_INSTRUCTION_ID_FOR_PLACE = 10
    ERROR_NOT_ENOUGH_PLACE_POSES = 11
    ERROR_GRIPPER_NOT_DEFINED = 12
    # Learning errors
    ERROR_LEARNING_NOT_IMPLEMENTED = 101
    ERROR_LEARNING_GRIPPER_NOT_DEFINED = 102
    ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED = 103

    ERROR_ROBOT_HALTED = InterfaceState.ERROR_ROBOT_HALTED
    ERROR_OBJECT_MISSING = InterfaceState.ERROR_OBJECT_MISSING
    ERROR_OBJECT_MISSING_IN_POLYGON = InterfaceState.ERROR_OBJECT_MISSING_IN_POLYGON
    ERROR_NO_GRIPPER_AVAILABLE = InterfaceState.ERROR_NO_GRIPPER_AVAILABLE
    ERROR_OBJECT_IN_GRIPPER = InterfaceState.ERROR_OBJECT_IN_GRIPPER
    ERROR_NO_OBJECT_IN_GRIPPER = InterfaceState.ERROR_NO_OBJECT_IN_GRIPPER
    ERROR_PICK_FAILED = InterfaceState.ERROR_PICK_FAILED
    ERROR_PICK_PLACE_SERVER_NOT_READY = InterfaceState.ERROR_PICK_PLACE_SERVER_NOT_READY
    ERROR_PLACE_FAILED = InterfaceState.ERROR_PLACE_FAILED
    ERROR_DRILL_FAILED = InterfaceState.ERROR_DRILL_FAILED
    ERROR_GRIPPER_MOVE_FAILED = InterfaceState.ERROR_GRIPPER_MOVE_FAILED
    ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT = InterfaceState.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT

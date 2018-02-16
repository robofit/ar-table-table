from brain_utils import ArtBrainUtils, ArtBrainErrors, ArtBrainErrorSeverities
import actionlib
from art_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, \
    ArmNavigationAction, ArmNavigationGoal, ArmNavigationResult
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
import rospy
from art_msgs.msg import ObjInstance
from std_msgs.msg import Bool


class ArtGripper(object):

    def __init__(self, arm_id, drill_enabled, pp_client=None, manipulation_client=None, interaction_on_client=None,
                 interaction_off_client=None, get_ready_client=None, move_to_user_client=None, gripper_link=None,
                 holding_object_topic=None, interaction_state=None):
        self.arm_id = arm_id
        self.gripper_link = gripper_link
        self.interaction_state = False

        self.pp_client_name = pp_client
        self.manip_client_name = manipulation_client
        self.drill_enabled = drill_enabled
        if pp_client is not None:
            self.pp_client = actionlib.SimpleActionClient(
                self.pp_client_name, PickPlaceAction)
            rospy.loginfo("Waiting for " + str(arm_id) + "'s gripper pick&place action client")
            self.pp_client.wait_for_server()
            rospy.loginfo("Connected to " + str(arm_id) + "'s gripper pick&place action client")
        else:
            self.pp_client = None
        if manipulation_client is not None:
            self.manip_client = actionlib.SimpleActionClient(
                self.manip_client_name, ArmNavigationAction)
            rospy.loginfo("Waiting for " + str(arm_id) + "'s gripper manipulation action client")
            self.manip_client.wait_for_server()
            rospy.loginfo("Connected to " + str(arm_id) + "'s gripper pick&place action client")
        else:
            self.manip_client = None

        if holding_object_topic is not None:
            self.holding_object_sub = rospy.Subscriber(holding_object_topic, ObjInstance, queue_size=1,
                                                       callback=self.holding_object_cb)
        else:
            self.holding_object_sub = None

        self.holding_object = None
        self.last_pick_instruction_id = None
        if interaction_on_client is not None:
            self.interaction_on_client = ArtBrainUtils.create_service_client(
                interaction_on_client, Trigger)
        else:
            self.interaction_on_client = None
        if interaction_off_client is not None:
            self.interaction_off_client = ArtBrainUtils.create_service_client(
                interaction_off_client, Trigger)
        else:
            self.interaction_off_client = None
        if get_ready_client is not None:
            self.get_ready_client = ArtBrainUtils.create_service_client(
                get_ready_client, Trigger)
        else:
            self.get_ready_client = None
        if move_to_user_client is not None:
            self.move_to_user_client = ArtBrainUtils.create_service_client(
                move_to_user_client, Trigger)
        else:
            self.move_to_user_client = None

        if interaction_state is not None:
            rospy.loginfo("Waiting for arm interaction state message")
            self.interaction_state = rospy.wait_for_message(interaction_state, Bool).data
            if self.interaction_state:
                self.interaction_off()
            rospy.loginfo("Got it")
            self.interaction_state_sub = rospy.Subscriber(interaction_state, Bool, self.interaction_state_cb)

        else:
            self.interaction_state_sub = None

        rospy.loginfo("Arm " + str(arm_id) + " ready.")

    def re_init(self, reset_holding_object=True):
        if reset_holding_object:
            if self.pp_client is not None:
                goal = PickPlaceGoal()
                goal.operation = goal.RESET
                self.pp_client.send_goal(goal)
                self.pp_client.wait_for_result()
            self.last_pick_instruction_id = None
            self.holding_object = None
        self.get_ready()

    def get_ready(self):
        if self.get_ready_client is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_NOT_IMPLEMENTED
        resp = self.get_ready_client.call(TriggerRequest())  # type: TriggerResponse
        if resp.success:
            return None, None
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED

    def move_to_user(self):
        if self.move_to_user_client is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_NOT_IMPLEMENTED
        resp = self.move_to_user_client.call(TriggerRequest())  # type: TriggerResponse
        if resp.success:
            return None, None
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED

    def interaction_on(self):
        if self.interaction_on_client is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_NOT_IMPLEMENTED
        resp = self.interaction_on_client.call(TriggerRequest())  # type: TriggerResponse
        if resp.success:
            return None, None
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED

    def interaction_off(self):
        if self.interaction_off_client is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_NOT_IMPLEMENTED
        resp = self.interaction_off_client.call(TriggerRequest())  # type: TriggerResponse
        if resp.success:
            return None, None
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED

    def move_through_poses(self, poses):
        if self.manip_client is None:
            return False
        goal = ArmNavigationGoal()
        goal.operation = goal.MOVE_THROUGH_POSES
        goal.poses = poses
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def touch_poses(self, object_id, poses, drill_duration=0):

        assert isinstance(poses, list)

        if self.manip_client is None:
            return False
        if not self.drill_enabled and drill_duration > 0:
            return False
        goal = ArmNavigationGoal()
        goal.object = object_id
        goal.operation = goal.TOUCH_POSES
        goal.drill_duration = drill_duration
        goal.poses = poses

        for pose in goal.poses:  # type: PoseStamped
            pose.header.frame_id = "object_id_" + object_id
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        print self.manip_client.get_result().result
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def move_to_pose(self, pose):
        if self.manip_client is None:
            return False
        goal = ArmNavigationGoal()
        goal.poses = [pose]
        goal.operation = ArmNavigationGoal.MOVE_THROUGH_POSES
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def prepare_for_interaction(self):
        if self.move_to_user_client is None or self.interaction_on_client is None:
            return False, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED
        # result = self.move_to_user_client.call()

        # if not result.success:
        #    rospy.logwarn("Can't move gripper to the user: " +
        #                  str(result.message))
        #    return False, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED
        result = self.interaction_on_client.call()
        if not result:
            rospy.logwarn(
                "Can't change gripper interaction state: " + str(result.message))
            # TODO: check arm state, inform user
            return False, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED
        return True, None

    def holding_object_cb(self, obj):
        """

        :param obj:
        :type obj: ObjInstance
        :return:
        """
        rospy.logerr("holding obj cb")
        rospy.logerr(obj)
        if obj.object_type is None or obj.object_type == '':
            self.holding_object = None
        else:
            self.holding_object = obj

    def interaction_state_cb(self, data):
        self.interaction_state = data.data

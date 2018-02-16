from art_gripper import ArtGripper
from brain_utils import ArtBrainErrors, ArtBrainErrorSeverities
import abc
import rospy
from art_msgs.msg import InterfaceState, PickPlaceGoal
import math
from std_srvs.srv import TriggerRequest, TriggerResponse, Trigger
from art_msgs.srv import ReinitArms, ReinitArmsResponse, ReinitArmsRequest
from actionlib.simple_action_client import SimpleGoalState


class ArtBrainRobotInterface:

    __metaclass__ = abc.ABCMeta

    def __init__(self, robot_helper):
        self._arms = []
        self.halted = False
        self.rh = robot_helper
        for arm in self.rh.arms:

            self._arms.append(ArtGripper(arm.arm_id,
                                         drill_enabled=arm.drill_enabled(),
                                         pp_client=arm.get_pick_place_action_name(),
                                         holding_object_topic=arm.get_holding_object_topic_name(),
                                         manipulation_client=arm.get_manipulation_action_name(),
                                         move_to_user_client=arm.get_move_to_user_service_name(),
                                         interaction_on_client=arm.get_interaction_on_service_name(),
                                         interaction_off_client=arm.get_interaction_off_service_name(),
                                         get_ready_client=arm.get_get_ready_service_name(),
                                         interaction_state=arm.get_interaction_state_topic_name(),
                                         gripper_link=arm.get_gripper_link()),)

            # arm reinit service
            self.reinit_srv = rospy.Service(arm.get_reinit_service_name(), ReinitArms, self.re_init_arm_cb)

        # emergency stop and restore
        self.emergency_stop_srv = rospy.Service(self.rh.get_emergency_stop_service_name(), Trigger, self.emergency_stop_cb)
        self.restore_srv = rospy.Service(self.rh.get_robot_restore_service_name(), Trigger, self.restore_cb)
        self.prepare_for_calibration_srv = rospy.Service(self.rh.get_prepare_for_calibration_service_name(), Trigger, self.prepare_for_calibration_cb)

    def re_init_arm_cb(self, data):
        """

        Args:
            data:
        @type data: ReinitArmsRequest

        Returns:

        """
        print data
        if len(data.arm_ids) == 0:
            for arm in self._arms:
                arm.re_init()
        else:
            for arm_id in data.arm_ids:
                arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
                arm.re_init()
        return ReinitArmsResponse()

    def emergency_stop_cb(self, _):
        resp = TriggerResponse()
        if self.emergency_stop():
            resp.success = True
        else:
            resp.success = False
            resp.message = "Failed to stop the robot"
        return resp

    def restore_cb(self, _):
        resp = TriggerResponse()
        if self.restore_robot():
            resp.success = True
        else:
            resp.success = False
            resp.message = "Failed to restore the robot"
        return resp

    def prepare_for_calibration_cb(self, _):
        resp = TriggerResponse()
        if self.prepare_for_calibration():
            resp.success = True
        else:
            resp.success = False
            resp.message = "Failed to prepare for interaction"
        return resp

    @abc.abstractmethod
    def prepare_for_calibration(self):
        pass

    @abc.abstractmethod
    def emergency_stop(self):
        pass

    @abc.abstractmethod
    def restore_robot(self):
        pass

    def arms_reinit(self, arm_ids=[]):
        assert isinstance(arm_ids, list)

        if len(arm_ids) > 0:
            for arm_id in arm_ids:
                arm = self.get_arm_by_id(arm_id)
                arm.re_init()
        for arm in self._arms:
            arm.re_init()

    def pick_object(self, obj, pick_instruction_id, arm_id=None, pick_only_y_axis=False, from_feeder=False):
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)
        severity, error = self.check_arm_for_pick(arm)
        if error is not None:
            return severity, error, arm_id
        if obj is None or obj.object_id is None or obj.object_id == "":
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_OBJECT_NOT_DEFINED, arm_id
        goal = PickPlaceGoal()
        goal.object = obj.object_id
        goal.operation = goal.PICK_OBJECT_ID if not from_feeder else goal.PICK_FROM_FEEDER
        goal.keep_orientation = False
        goal.pick_only_y_axis = pick_only_y_axis
        rospy.logdebug("Picking object with ID: " + str(obj.object_id))
        arm.pp_client.send_goal(goal)
        # arm.pp_client.wait_for_result()
        severity, error, _ = self.wait_for_action_result(arm.pp_client)
        if error is not None:
            print "returning 2"
            return severity, error, arm_id
        # TODO: make some error msg etc
        rospy.logdebug('Results from p&p server')
        rospy.logdebug("result: " + str(arm.pp_client.get_result()))
        rospy.logdebug("status: " + arm.pp_client.get_goal_status_text())
        rospy.logdebug("state: " + str(arm.pp_client.get_state()))
        if arm.pp_client.get_result().result == 0:
            arm.last_pick_instruction_id = pick_instruction_id
            return None, None, arm_id
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PICK_FAILED, arm_id

    def place_object_to_pose(self, place_pose, arm_id, objects_frame_id="marker", pick_only_y_axis=False):
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        if place_pose is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_PLACE_POSE_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
        if arm.holding_object is None or arm.holding_object.object_type is None or arm.holding_object.object_type == "":
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_NO_OBJECT_IN_GRIPPER, arm_id
        goal = PickPlaceGoal()
        goal.operation = goal.PLACE_TO_POSE

        goal.object = arm.holding_object.object_id
        # TODO (Kapi) check if it is safe to place the object
        #  if not self.check_place_pose(place_pose, obj):
        #    return False
        # TODO how to decide between 180 and 90 deg?
        # allow object to be rotated by 90 deg around z axis
        # goal.z_axis_angle_increment = 3.14

        goal.pose = place_pose
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.header.frame_id = objects_frame_id
        # no need to alter place pose z - contacts between attached object and support surface (table) are allowed
        # goal.pose.pose.position.z += 0.01

        rospy.logdebug("Placing object with ID: " + str(arm.holding_object.object_id))
        arm.pp_client.send_goal(goal)
        severity, error, _ = self.wait_for_action_result(arm.pp_client)
        if error is not None:
            return severity, error, arm_id

        if arm.pp_client.get_result().result == 0:
            return None, None, arm_id
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PLACE_FAILED, None

    def wait_for_action_result(self, action_client):
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and action_client.simple_state != SimpleGoalState.DONE:
            print self.is_halted()
            if self.is_halted():
                action_client.cancel_goal()
                print "returning"
                return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_ROBOT_HALTED, None
            r.sleep()
        return None, None, None

    def drill_point(self, arm_id, pose, obj, obj_frame_id, drill_duration):

        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)

        if not arm.touch_poses(obj.object_id, pose, drill_duration):
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_DRILL_FAILED, arm_id
        else:
            return None, None, arm_id

    def move_arm_to_pose(self, pose, arm_id=None, picking=False, drilling=False):
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)

        if not arm.move_to_pose(pose):
            if picking:
                return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PICK_FAILED, arm_id
            elif drilling:
                return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_DRILL_FAILED, arm_id
            else:
                return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED, arm_id
        else:
            return None, None, arm_id

    def arms_get_ready(self, arm_ids=[]):
        assert isinstance(arm_ids, list)
        if not arm_ids:
            for arm in self._arms:
                severity, error = arm.get_ready()
                if error is not None:
                    return severity, error, arm.arm_id
        else:
            for arm_id in arm_ids:
                arm = self.get_arm_by_id(arm_id)
                if arm is None:
                    continue
                arm.get_ready()
        return None, None, None

    def arm_prepare_for_interaction(self, arm_id=None):
        if arm_id is None:
            for arm in self._arms:  # type: ArtGripper
                severity, error, arm_id = self.arm_prepare_for_interaction(arm.arm_id)
                if error is not None:
                    return severity, error, arm_id
            else:
                return None, None, None

        else:
            arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
            # severity, error = arm.move_to_user()
            # if error is not None:
            #    return severity, error, arm_id
            severity, error = arm.interaction_on()
            if error is not None:
                return severity, error, arm_id
            return None, None, arm_id

    def arm_get_ready_after_interaction(self, arm_id=None):
        if arm_id is None:
            for arm in self._arms:  # type: ArtGripper
                severity, error, arm_id = self.arm_get_ready_after_interaction(arm.arm_id)
                if error is not None:
                    return severity, error, arm_id
            else:
                return None, None, None

        else:
            arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
            severity, error = arm.interaction_off()
            if error is not None:
                return severity, error, arm_id
            severity, error = arm.get_ready()
            if error is not None:
                return severity, error, arm_id
            return None, None, arm_id

    def check_arm_for_pick(self, arm):
        severity, error = self.check_arm(arm)
        if error is not None:
            return severity, error
        if arm.holding_object is not None:
            rospy.logwarn(
                "Pick: gripper " +
                arm.id +
                " already holding an object (" +
                arm.holding_object.object_id +
                ")")
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_OBJECT_IN_GRIPPER

        return None, None

    def check_arm_for_place(self, arm):
        severity, error = self.check_arm(arm)
        if error is not None:
            return severity, error
        if arm.holding_object is None:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT
        return None, None

    def check_arm(self, arm):
        if arm is None:
            rospy.logerr("No gripper!")
            return ArtBrainErrorSeverities.SEVERE, ArtBrainErrors.ERROR_NO_GRIPPER_AVAILABLE

        if arm.pp_client is None:
            rospy.logerr("No pick place client!")
            return ArtBrainErrorSeverities.SEVERE, ArtBrainErrors.ERROR_GRIPPER_PP_CLIENT_MISSING

        if not arm.pp_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("Pick place server is not running!")
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PICK_PLACE_SERVER_NOT_READY

        return None, None

    @abc.abstractmethod
    def select_arm_for_pick(self, obj_id, objects_frame_id, tf_listener):
        return

    @abc.abstractmethod
    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        return

    @abc.abstractmethod
    def select_arm_for_drill(self, obj_to_drill, tf_listener):
        return

    def select_arm_for_place(self, obj_type, pick_instruction_ids):
        for arm in self._arms:
            if arm.last_pick_instruction_id in pick_instruction_ids:
                return arm.arm_id
        else:
            for arm in self._arms:
                if arm.holding_object is None:
                    continue
                if arm.holding_object.object_type == obj_type:
                    return arm.arm_id
        return None

    def get_arm_by_id(self, arm_id):
        """

        :param arm_id:
        :return:
        :rtype ArtGripper
        """
        for arm in self._arms:
            if arm.arm_id == arm_id:
                return arm
        else:
            return None

    def init_arms(self, arms=None, reset_holding_object=True):
        if arms is None:
            for arm in self._arms:
                arm.re_init(reset_holding_object=reset_holding_object)
        else:
            for arm in arms:
                arm.re_init(reset_holding_object=reset_holding_object)

    def set_halted(self, halted):
        self.halted = halted

    def is_halted(self):
        return self.halted

    def get_arm_holding_object(self, arm_id):
        return self.get_arm_by_id(arm_id).holding_object

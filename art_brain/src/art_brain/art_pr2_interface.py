from art_brain_robot_interface import ArtBrainRobotInterface
from geometry_msgs.msg import PoseStamped
import rospy
from art_gripper import ArtGripper
from brain_utils import ArtBrainUtils
from art_msgs.srv import ReinitArmsRequest, ReinitArmsResponse
from art_msgs.msg import ObjInstance
from std_srvs.srv import TriggerResponse, TriggerRequest, Trigger
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import Pr2GripperCommand


class ArtPr2Interface(ArtBrainRobotInterface):

    BOTH_ARM = "both_arm"
    LEFT_ARM = "left_arm"
    RIGHT_ARM = "right_arm"

    def __init__(self, robot_helper):
        self.gripper_usage = self.BOTH_ARM
        super(ArtPr2Interface, self).__init__(robot_helper)
        self.motors_halted_sub = rospy.Subscriber(
            "/pr2_ethercat/motors_halted", Bool, self.motors_halted_cb)
        self.halt_motors_srv = rospy.ServiceProxy(
            '/pr2_ethercat/halt_motors', Empty)  # TODO wait for service? where?

        self.reset_motors_srv = rospy.ServiceProxy(
            '/pr2_ethercat/reset_motors', Empty)  # TODO wait for service? where?

        self.look_at_pub = rospy.Publisher(robot_helper.robot_ns + "/look_at", PointStamped, queue_size=10)

        self.gripper_pubs = []
        for pref in ('/l_', '/r_'):
            self.gripper_pubs.append(rospy.Publisher(pref + 'gripper_controller/command', Pr2GripperCommand, queue_size=10))

        for arm in self._arms:  # type: ArtGripper
            if arm.arm_id == self.LEFT_ARM:
                arm.arm_up = ArtBrainUtils.create_service_client(robot_helper.robot_ns + "/" + self.LEFT_ARM + "/arm_up", Trigger)
            elif arm.arm_id == self.RIGHT_ARM:
                arm.arm_up = ArtBrainUtils.create_service_client(robot_helper.robot_ns + "/" + self.RIGHT_ARM + "/arm_up", Trigger)

    def select_arm_for_pick(self, obj, objects_frame_id, tf_listener):

        assert isinstance(obj, ObjInstance)

        free_arm = self.select_free_arm()
        if free_arm in [None, self.LEFT_ARM, self.RIGHT_ARM]:
            return free_arm
        objects_frame_id = ArtBrainUtils.normalize_frame_id(objects_frame_id)

        obj_pose = PoseStamped()
        obj_pose.pose = obj.pose
        obj_pose.header.frame_id = objects_frame_id
        # exact time does not matter in this case
        obj_pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(
            'base_link',
            obj_pose.header.frame_id,
            obj_pose.header.stamp,
            rospy.Duration(1))
        obj_pose = tf_listener.transformPose(
            'base_link', obj_pose)
        if obj_pose.pose.position.y < 0:
            return self.RIGHT_ARM
        else:
            return self.LEFT_ARM

    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):

        assert isinstance(pick_pose, PoseStamped)

        pick_pose.header.frame_id = ArtBrainUtils.normalize_frame_id(pick_pose.header.frame_id)
        free_arm = self.select_free_arm()
        if free_arm in [None, self.LEFT_ARM, self.RIGHT_ARM]:
            return free_arm

        pick_pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(
            'base_link',
            pick_pose.header.frame_id,
            pick_pose.header.stamp,
            rospy.Duration(1))
        obj_pose = tf_listener.transformPose(
            'base_link', pick_pose)
        if obj_pose.pose.position.y < 0:
            return self.RIGHT_ARM
        else:
            return self.LEFT_ARM

    def select_arm_for_drill(self, obj_to_drill, objects_frame_id, tf_listener):

        assert isinstance(obj_to_drill, ObjInstance)

        free_arm = self.select_free_arm()
        if free_arm in [None, self.LEFT_ARM, self.RIGHT_ARM]:
            return free_arm
        objects_frame_id = ArtBrainUtils.normalize_frame_id(objects_frame_id)

        # frameExists("marker") always returns False -> but it is probably not necessary to test it - it should exist
        # if tf_listener.frameExists(
        #        "base_link") and tf_listener.frameExists(ArtBrainUtils.normalize_frame_id(objects_frame_id)):

        obj_pose = PoseStamped()
        obj_pose.pose = obj_to_drill.pose
        obj_pose.header.frame_id = objects_frame_id
        # exact time does not matter in this case
        obj_pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(
            'base_link',
            obj_pose.header.frame_id,
            obj_pose.header.stamp,
            rospy.Duration(1))
        obj_pose = tf_listener.transformPose(
            'base_link', obj_pose)
        if obj_pose.pose.position.y < 0:
            return self.RIGHT_ARM
        else:
            return self.LEFT_ARM

    def select_free_arm(self):
        left_arm = self.get_arm_by_id(self.LEFT_ARM) if self.gripper_usage in [self.BOTH_ARM,
                                                                               self.LEFT_ARM] else None  # type: ArtGripper
        right_arm = self.get_arm_by_id(self.RIGHT_ARM) if self.gripper_usage in [self.BOTH_ARM,
                                                                                 self.RIGHT_ARM] else None  # type: ArtGripper
        if left_arm is None and right_arm is None:
            return None
        rospy.logerr(left_arm.holding_object)
        rospy.logerr(right_arm.holding_object)
        if self.gripper_usage == self.LEFT_ARM:
            if left_arm.holding_object:
                return None
            else:
                return left_arm.arm_id
        elif self.gripper_usage == self.RIGHT_ARM:
            if right_arm.holding_object:
                return None
            else:
                return right_arm.arm_id
        elif self.gripper_usage == self.BOTH_ARM:
            if left_arm.holding_object and not right_arm.holding_object:
                return right_arm.arm_id
            elif not left_arm.holding_object and right_arm.holding_object:
                return left_arm.arm_id
            elif left_arm.holding_object and right_arm.holding_object:
                return None
        return self.BOTH_ARM

    def restore_robot(self):
        self.reset_motors_srv.call(EmptyRequest())
        return True  # TODO: how to check if it worked? wait some time and check topic?

    def emergency_stop(self):
        self.halt_motors_srv.call(EmptyRequest())
        return True  # TODO: how to check if it worked? wait some time and check topic?

    def motors_halted_cb(self, req):

        if self.is_halted() and not req.data:
            rospy.loginfo("Getting ready after halt...")
            for arm in self._arms:
                arm.re_init()

        self.set_halted(req.data)

    def close_grippers(self):

        rospy.loginfo("Closing both grippers.")
        msg = Pr2GripperCommand()
        msg.position = 0.0
        msg.max_effort = 10000.0

        for pub in self.gripper_pubs:
            pub.publish(msg)
            pub.publish(msg)
            pub.publish(msg)

    def arms_get_ready(self, arm_ids=[]):

        self.close_grippers()
        return super(ArtPr2Interface, self).arms_get_ready(arm_ids)

    def arms_reinit(self, arm_ids=[]):

        self.close_grippers()
        return super(ArtPr2Interface, self).arms_reinit(arm_ids)

    def look_at(self, x, y, z, frame_id="marker"):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.look_at_pub.publish(point)

    def look_at_point(self, point, frame_id="marker"):
        self.look_at(point.x, point.y, point.z, frame_id)

    def pick_object(self, obj, pick_instruction_id, arm_id=None, pick_only_y_axis=False, from_feeder=False):
        assert isinstance(obj, ObjInstance)
        self.look_at_point(obj.pose.position)
        return super(ArtPr2Interface, self).pick_object(obj, pick_instruction_id, arm_id, pick_only_y_axis, from_feeder)

    def place_object_to_pose(self, place_pose, arm_id, objects_frame_id="marker", pick_only_y_axis=False):

        assert isinstance(place_pose, PoseStamped)
        self.look_at_point(place_pose.pose.position)

        return super(ArtPr2Interface, self).place_object_to_pose(place_pose, arm_id, objects_frame_id, pick_only_y_axis)

    def move_arm_to_pose(self, pose, arm_id=None, picking=False, drilling=False):
        assert isinstance(pose, PoseStamped)
        self.look_at_point(pose.pose.position)
        return super(ArtPr2Interface, self).move_arm_to_pose(pose, arm_id)

    def prepare_for_calibration(self):
        for arm in self._arms:  # type: ArtGripper
            arm.arm_up.call(TriggerRequest())

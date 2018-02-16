import rospy

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from art_msgs.msg import ArmNavigationAction,  \
    ArmNavigationFeedback, ArmNavigationResult, ArmNavigationGoal
import moveit_commander
import copy

import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from tf import TransformListener


class ArtArmNavigationActionServer(object):
    feedback = ArmNavigationFeedback()
    result = ArmNavigationResult()

    def __init__(self, server_prefix, group_name):
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self._as = actionlib.SimpleActionServer(server_prefix + self.group_name + "/manipulation",
                                                ArmNavigationAction, self.action_cb)
        self._as.start()

        char = self.group_name[0]
        self.jta = actionlib.SimpleActionClient('/' + char + '_arm_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Got it')
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.js_cb, queue_size=1)
        self.angles = None
        self.move_to_pose_pub = rospy.Publisher("/pr2_arm_navigation/move_to_pose", PoseStamped, queue_size=10, latch=True)

        self.joint_names = [char + '_shoulder_pan_joint',
                            char + '_shoulder_lift_joint',
                            char + '_upper_arm_roll_joint',
                            char + '_elbow_flex_joint',
                            char + '_forearm_roll_joint',
                            char + '_wrist_flex_joint',
                            char + '_wrist_roll_joint']

        self.tf_listener = TransformListener()

        self.look_at_pub = rospy.Publisher("/art/robot/look_at", PointStamped, queue_size=1)

    def action_cb(self, goal):
        max_attempt = 3

        self.feedback.attempt = 1
        rospy.loginfo("Got goal with " + str(len(goal.poses)) + "poses.")

        if goal.operation == ArmNavigationGoal.MOVE_THROUGH_POSES:
            for idx, p in enumerate(goal.poses):
                attempt = 1
                self._as.publish_feedback(self.feedback)
                while not self.move_to_point(p):
                    attempt += 1
                    if attempt > max_attempt:
                        self.result.result = self.result.FAILURE
                        self._as.set_aborted(self.result)
                        return
            self.result.result = ArmNavigationResult.SUCCESS
        elif goal.operation == ArmNavigationGoal.TOUCH_POSES:
            for idx, p in enumerate(goal.poses):
                attempt = 1
                self._as.publish_feedback(self.feedback)

                # TODO it would be better to only repeat failed step (e.g. go3)
                world_frame = rospy.get_param("/art/conf/world_frame", "marker")
                p.header.stamp = rospy.Time(0)
                p_marker = self.tf_listener.transformPose(world_frame, p)
                """while not self.touch_point(p_marker, goal.drill_duration):

                    rospy.loginfo("Attempt " + str(attempt) + " out of " + str(max_attempt) + ".")
                    attempt += 1
                    if attempt > max_attempt:
                        self.result.result = self.result.FAILURE
                        self._as.set_aborted(self.result)
                        return"""
                if self.touch_point(p_marker, goal.drill_duration):
                    self.result.result = self.result.SUCCESS
                    self._as.set_succeeded(result=self.result)
                else:
                    self.result.result = self.result.FAILURE
                    self.result.message = "Failed to drill"
                    self._as.set_aborted(result=self.result)

        elif goal.operation == ArmNavigationGoal.MOVE_THROUGH_TRAJECTORY:
            self.result.result = ArmNavigationResult.BAD_REQUEST
            self.result.message = "Not implemented!"
            self._as.set_aborted(self.result)
            return
        rospy.loginfo("return")
        self.result.result = self.result.SUCCESS
        self._as.set_succeeded(self.result)

    def move_to_point(self, pose):

        pt = PointStamped()
        pt.header = pose.header
        pt.point = pose.pose.position
        self.look_at_pub.publish(pt)

        self.group.set_pose_target(pose)
        self.move_to_pose_pub.publish(pose)
        if not self.group.go(wait=True):
            return False
        return True

    def touch_point(self, pose, drill_duration, max_attempts=3):
        rospy.loginfo("Touch point in")
        pre_touch_pose = copy.deepcopy(pose)

        # TODO this works only for world frame_id (marker) -> in general, this pre_touch translation should be done with respect to x-axis of the gripper!
        pre_touch_pose.pose.position.z += 0.05  # 5cm above desired pose
        self.group.set_pose_target(pre_touch_pose)
        rospy.logdebug("Pre touch pose")
        self.move_to_pose_pub.publish(pre_touch_pose)
        attempt = 0
        while not self.group.go(wait=True):
            if attempt > max_attempts:
                rospy.logerr("Failed to reach pre touch pose, giving up")
                return False
            else:
                attempt += 1
            rospy.logerr("Pre touch pose failed, try again")
        self.group.set_pose_target(pose)
        rospy.logdebug("Touching point")
        self.move_to_pose_pub.publish(pose)
        attempt = 0
        while not self.group.go(wait=True):
            if attempt > max_attempts:
                rospy.logerr("Failed to reach touch pose, giving up")
                return False
            else:
                attempt += 1
            rospy.logerr("Touch pose failed, try again")
        self.rotate_gripper(drill_duration)
        self.group.set_pose_target(pre_touch_pose)
        rospy.loginfo("Post touch pose")
        self.move_to_pose_pub.publish(pre_touch_pose)
        attempt = 0
        while not self.group.go(wait=True):
            if attempt > max_attempts:
                rospy.logerr("Failed to reach post touch pose, giving up")
                return False
            else:
                attempt += 1
            rospy.logerr("Post touch pose failed, try again")
        rospy.logdebug("Touch done")
        return True

    def rotate_gripper(self, duration):
        rospy.loginfo("rotate in, duration = " + str(duration))
        if duration == 0:
            return
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        angles = self.angles
        rate = 4
        for i in xrange(duration * rate):
            angles[6] += i * 0.1
            rospy.loginfo(angles[6])
            point.positions = angles
            point.time_from_start = rospy.Duration(1 / rate * i)
            goal.trajectory.points.append(copy.deepcopy(point))
        rospy.loginfo("rotate send goal")
        self.jta.send_goal(goal)
        rospy.sleep(duration + 1)
        rospy.loginfo("rotate out")

    def js_cb(self, data):
        self.angles = [0] * 7
        positions = zip(data.name, data.position)
        for name, position in positions:
            if name in self.joint_names:
                self.angles[self.joint_names.index(name)] = position

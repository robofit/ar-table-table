#!/usr/bin/env python

import unittest
import rostest
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from art_msgs.msg import InstancesArray


class TestSimpleTrackerRos(unittest.TestCase):

    def setUp(self):

        self.tfl = tf.TransformListener()
        rospy.sleep(2.0)
        self.world_frame = "marker"
        self.object_id = "21"

    def test_object_transform(self):

        ps = PoseStamped()
        ps.header.frame_id = "object_id_" + self.object_id
        ps.header.stamp = rospy.Time(0)

        self.tfl.waitForTransform(
            self.world_frame, ps.header.frame_id, ps.header.stamp, rospy.Duration(1.0))

        ps = self.tfl.transformPose(self.world_frame, ps)

        self.assertEquals(ps.header.frame_id, self.world_frame, "test_object_transform")

        self.assertAlmostEqual(ps.pose.position.x, 0, 1, "test_object_position")
        self.assertAlmostEqual(ps.pose.position.y, 0, 1, "test_object_position")
        self.assertAlmostEqual(ps.pose.position.z, 1, 1, "test_object_position")

    def test_topic(self):

        msg = rospy.wait_for_message("/art/object_detector/object_filtered", InstancesArray, 1.0)

        self.assertEquals(msg.header.frame_id, self.world_frame, "test_topic_frame_id")
        self.assertEquals(len(msg.instances), 1, "test_topic_inst_len")
        self.assertEquals(msg.instances[0].object_id, self.object_id, "test_topic_inst_object_id")


if __name__ == '__main__':

    rospy.init_node('art_simple_tracker_test_node')

    rostest.run('art_simple_tracker', 'test_simple_tracker_ros',
                TestSimpleTrackerRos, sys.argv)

#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from . import ArtCellCalibration
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform, PointStamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers


class ArtRobotCalibration(ArtCellCalibration):

    def __init__(self, robot_id, markers_topic, world_frame, robot_frame, main_cell_frame, pc_topic, tfl, look_at_topic='/art/robot/look_at'):

        # cell_id, markers_topic, world_frame, cell_frame, main_cell_frame, pc_topic, tf_listener
        super(ArtRobotCalibration, self).__init__(robot_id, markers_topic, world_frame, robot_frame, main_cell_frame, pc_topic, tfl)

        self.head_look_at_pub = rospy.Publisher(look_at_topic, PointStamped, queue_size=1)
        self.positions = [np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8')]
        self.robot_state = 0
        self.robot_looking_for_id = 10
        self.count = 0

    def reset_markers_searching(self):
        self.positions = [np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8')]
        self.robot_looking_for_id = 10
        self.robot_state = 0
        self.count = 0
        self.start_marker_detection()

    def markers_cb(self, markers):
        if self.calibrated:
            return
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.point.x = 0.3
        point.point.z = 1
        if self.robot_state == 0:
            point.point.y = -0.5
            self.head_look_at_pub.publish(point)
            rospy.sleep(5)
            self.robot_state = 1
        elif self.robot_state == 1 and self.robot_looking_for_id > 12:
            point.point.y = 0.4
            self.head_look_at_pub.publish(point)
            rospy.sleep(5)
            self.robot_state = 2
        elif self.robot_state == 2 and self.robot_looking_for_id > 20:
            self.stop_marker_detection()
            self.calibrate()

        if self.robot_looking_for_id == 12:
            self.robot_looking_for_id += 1
            return
        if self.robot_looking_for_id < 20:
            p = ArtCalibrationHelper.get_marker_position_by_id(markers, self.robot_looking_for_id)
            if p is not None:
                self.positions[self.robot_looking_for_id - 10] += p

                self.count += 1
                if self.count >= self.avg:
                    self.positions[self.robot_looking_for_id - 10] /= self.count
                    self.count = 0
                    self.robot_looking_for_id += 1
                    if self.robot_looking_for_id > 13:
                        self.robot_looking_for_id += 100

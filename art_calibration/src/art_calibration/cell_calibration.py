#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool
import ast
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
#import pcl
import numpy as np
from sensor_msgs.msg import PointField
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
import tf
from std_msgs.msg import Header
import struct
from copy import deepcopy


class ArtCellCalibration(object):

    last_pc = None
    last_pc_transformed = None

    def __init__(self, cell_id, markers_topic, world_frame, cell_frame, main_cell_frame, pc_topic, tf_listener):
        self.cell_id = cell_id
        self.markers_topic = markers_topic
        self.calibrated = False
        self.positions = [np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'),
                          np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8')]
        self.avg = 100
        self.cnt = [0, 0, 0, 0]

        self.cell_frame = cell_frame
        self.world_frame = world_frame
        self.listener = tf_listener  # type: tf.TransformListener()
        self.main_cell_frame = main_cell_frame
        self.transformation = Transform()

        m = rospy.get_param("~" + self.cell_id + "/calibration_matrix", None)

        self.markers_sub = rospy.Subscriber(self.markers_topic, AlvarMarkers, self.markers_cb, queue_size=1)
        self.marker_detection_enable_publisher = rospy.Publisher("/art/" +
                                                                 self.cell_id +
                                                                 "/ar_track_alvar_bundle_calibration/enable_detection",
                                                                 Bool,
                                                                 queue_size=1,
                                                                 latch=True)
        #self.pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self.pc_cb, queue_size=1)
        #self.pc_pub = rospy.Publisher("/test_" + str(cell_id), PointCloud2, queue_size=1)

        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.x_rotate_offset = 0
        self.y_rotate_offset = 0
        self.z_rotate_offset = 0

        if m is not None:

            m = np.matrix(ast.literal_eval(m))

            self.transformation.rotation = ArtCalibrationHelper.normalize_vector(transformations.quaternion_from_matrix(m))
            self.transformation.translation = transformations.translation_from_matrix(m)
            self.calibrated = True
            rospy.loginfo("Cell: " + str(self.cell_id) + " calibration loaded from param")
            self.stop_marker_detection()

        else:

            self.start_marker_detection()
            rospy.loginfo("Cell: " + str(self.cell_id) + " ready")

    def stop_marker_detection(self):
        detect = Bool()
        detect.data = False
        self.marker_detection_enable_publisher.publish(detect)

    def start_marker_detection(self):
        detect = Bool()
        detect.data = True
        self.marker_detection_enable_publisher.publish(detect)

    def calibrate(self):
        rospy.loginfo("Cell: " + str(self.cell_id) + " trying to calibrate")
        '''for p in self.positions:
            if p is None:
                return False'''
        point, m = ArtCalibrationHelper.transform_from_markers_positions(self.positions[0],
                                                                         self.positions[1],
                                                                         self.positions[2],
                                                                         self.positions[3], cell_id=self.cell_id)
        if point is None or m is None:
            if point is None:
                rospy.logerr("Origin was not computed!")
            elif m is None:
                rospy.logerr("Transformation matrix was not computed!")
            self.reset_markers_searching()  # let's try to get new positions of markers
            return

        rospy.set_param("~" + self.cell_id + "/calibration_matrix", str(m.tolist()))

        self.transformation.rotation = ArtCalibrationHelper.normalize_vector(transformations.quaternion_from_matrix(m))
        self.transformation.translation = transformations.translation_from_matrix(m)
        self.calibrated = True
        rospy.loginfo("Cell: " + str(self.cell_id) + " calibration done")

    def get_transform(self, cell_id="all"):
        if not self.calibrated:

            return None

        tr = deepcopy(self.transformation)
        tr.translation[0] += self.x_offset
        if cell_id == "n1":
            tr.translation[0] -= 1.135
        tr.translation[1] += self.y_offset
        tr.translation[2] += self.z_offset
        (x, y, z) = transformations.euler_from_quaternion(tr.rotation)
        tr.rotation = transformations.quaternion_from_euler(x + self.x_rotate_offset,
                                                            y + self.y_rotate_offset,
                                                            z + self.z_rotate_offset)
        return tr

    def reset_markers_searching(self):
        self.start_marker_detection()
        self.positions = [np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8'),
                          np.array([0, 0, 0], dtype='f8'), np.array([0, 0, 0], dtype='f8')]
        self.cnt = [0, 0, 0, 0]
        self.calibrated = False

    def markers_cb(self, markers):
        if self.calibrated:
            return
        all_markers = True
        for i in xrange(4):

            # if i == 2:
            #    continue

            if self.cnt[i] >= self.avg:
                continue

            all_markers = False

            p = ArtCalibrationHelper.get_marker_position_by_id(markers, i + 10)
            if p is not None:
                self.positions[i] += p
                self.cnt[i] += 1
                rospy.loginfo("Cell: " + str(self.cell_id) + " gets marker id " + str(i + 10) + ", cnt: " + str(self.cnt[i]))

        if all_markers:

            for i in xrange(4):
                # if i == 2:
                #    continue
                self.positions[i] /= self.avg

            self.stop_marker_detection()
            self.calibrate()

    '''def pc_cb(self, pc):
        
        if not self.calibrated:
            rospy.logwarn_throttle(1.0, "Not calibrated cell: " + self.cell_id)
            return

        # if self.cell_frame == self.main_cell_frame:
            # if False:
        #    transformed_cloud = pc
        # else:
        p = pcl.PointCloud(np.array(list(pc2.read_points(pc, field_names=['x', 'y', 'z'],
                                                         skip_nans=True)), dtype=np.float32))
        pcloud = pc2.create_cloud_xyz32(pc.header, p.to_list())
        self.last_pc = pcloud
        try:
            print self.cell_frame
            print self.main_cell_frame
            # translation, rotation = self.listener.lookupTransform(pc.header.frame_id, self.main_cell_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        transform = TransformStamped()
        tr = self.get_transform()
        transformed_cloud = self.transform_pcloud(pcloud, tr.translation, tr.rotation,
                                                  self.cell_frame, self.world_frame)

        # transformed_cloud = self.listener.transformPointCloud(self.world_frame, pc)

        p = pcl.PointCloud(np.array(list(pc2.read_points(transformed_cloud, field_names=['x', 'y', 'z'],
                                                         skip_nans=True)), dtype=np.float32))
        filter = p.make_voxel_grid_filter()  # type: pcl.VoxelGridFilter
        filter.set_leaf_size(0.01, 0.01, 0.01)
        p = filter.filter()

        seg = p.make_segmenter()  # type: pcl.Segmentation
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.02)
        indices, model = seg.segment()

        p = p.extract(indices)  # type: pcl.PointCloud

        h = Header()
        h.stamp = pc.header.stamp
        h.seq = pc.header.seq
        h.frame_id = self.world_frame

        pcloud = pc2.create_cloud_xyz32(h, p.to_list())
        pcloud.header.frame_id = self.world_frame

        self.last_pc_transformed = p'''

    '''@staticmethod
    def transform_pcloud(cloud_in, translation, rotation, source_frame_id, target_frame_id):
        transform = TransformStamped()

        transform.transform.translation = Vector3(translation[0], translation[1], translation[2])
        transform.transform.rotation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
        transform.header.frame_id = source_frame_id
        transform.child_frame_id = target_frame_id
        return do_transform_cloud(cloud_in, transform)'''

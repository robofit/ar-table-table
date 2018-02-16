#!/usr/bin/env python

from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from art_calibration import ArtRobotCalibration, ArtCellCalibration
from std_msgs.msg import Bool
from art_msgs.srv import RecalibrateCell, RecalibrateCellRequest, RecalibrateCellResponse
#from pcl.registration import icp, icp_nl, gicp
import tf
from std_msgs.msg import Header


from dynamic_reconfigure.server import Server
from art_calibration.cfg import CalibrationConfig

#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs.point_cloud2 as pc2


class ArtCalibration(object):

    def __init__(self):
        self.listener = tf.TransformListener()

        self.cells = []
        cell_names = rospy.get_param("~cells").split(",")

        for cell in cell_names:
            cell = cell.strip()

        for cell in cell_names:

            if cell == 'pr2':
                self.cells.append(ArtRobotCalibration('pr2', '/pr2/ar_pose_marker',
                                                      '/marker_detected', '/odom_combined',
                                                      '/' + cell_names[0] + '_kinect2_link',
                                                      '/pr2/pointsHACK', self.listener))  # TODO: kapi hack
            else:

                self.cells.append(ArtCellCalibration(cell, '/art/' + cell + '/ar_pose_marker',
                                                     '/marker_detected', '/' + cell + '_kinect2_link',
                                                     '/' + cell_names[0] + '_kinect2_link',
                                                     '/art/' + cell + '/kinect2/qhd/pointsHACK', self.listener))  # TODO: kapi hack

        self.calibrated_pub = rospy.Publisher('/art/system/calibrated', Bool,
                                              queue_size=10, latch=True)
        self.calibrated = Bool()
        self.calibrated.data = False
        self.calibrated_sended = False
        self.calibrated_pub.publish(self.calibrated)
        self.recalibrate_cell_service = rospy.Service("/art/system/calibrate_cell", RecalibrateCell,
                                                      self.recalibrate_cell_cb)

        self.broadcaster = TransformBroadcaster()

        self.dynamic_reconfigure_srv = Server(CalibrationConfig, self.dynamic_reconfigure_cb)

    def recalibrate_cell_cb(self, req):
        resp = RecalibrateCellResponse()
        resp.success = False
        cell_name = req.cell_name
        for cell in self.cells:
            if cell.cell_id == cell_name:
                rospy.loginfo('Recalibrating cell: ' + req.cell_name)
                cell.reset_markers_searching()
                resp.success = True
                break
        else:
            resp.error = "Unknown cell"
        return resp

    def publish_calibration(self):
        calibrated = True

        time = rospy.Time.now() + rospy.Duration(0, int(1000 / 30))

        for cell in self.cells:
            if cell.calibrated:
                tr = cell.get_transform()
                self.broadcaster.sendTransform(tr.translation, tr.rotation,
                                               time, cell.cell_frame,
                                               cell.world_frame)

            else:
                calibrated = False

        if calibrated and not self.calibrated_sended:
            self.calibrated_sended = True
            self.calibrated.data = True
            self.calibrated_pub.publish(self.calibrated)

    def dynamic_reconfigure_cb(self, config, level):

        for cell in self.cells:  # type: ArtCellCalibration
            cell.x_offset = config.get(cell.cell_id + "_x_offset", 0)
            cell.y_offset = config.get(cell.cell_id + "_y_offset", 0)
            cell.z_offset = config.get(cell.cell_id + "_z_offset", 0)
            cell.x_rotate_offset = config.get(cell.cell_id + "_x_rotate_offset", 0)
            cell.y_rotate_offset = config.get(cell.cell_id + "_y_rotate_offset", 0)
            cell.z_rotate_offset = config.get(cell.cell_id + "_z_rotate_offset", 0)
        return config

    '''def calculate_icp(self):
        print "calculate"
        main_cell = self.cells[0]  # type: ArtRobotCalibration
        for c in self.cells:  # type: ArtRobotCalibration

            if c.last_pc_transformed is None or not c.calibrated:
                return
            if c is main_cell:
                h = Header()
                h.stamp = rospy.Time.now()
                h.frame_id = c.world_frame

                pcloud = pc2.create_cloud_xyz32(h, c.last_pc_transformed.to_list())
                c.pc_pub.publish(pcloud)
                continue
            converged, transf, estimate, fitness = icp(main_cell.last_pc_transformed, c.last_pc_transformed, max_iter=25)

            print converged
            print fitness
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = c.world_frame
            print transf
            tt = transformations.inverse_matrix(transf)
            ttt = transformations.concatenate_matrices(transformations.translation_matrix(c.get_transform().translation),
                                                       transformations.quaternion_matrix(c.get_transform().rotation))
            translation2 = transformations.translation_from_matrix(

                transformations.translation_matrix(transformations.translation_from_matrix(tt)) *
                transformations.translation_matrix(c.get_transform().translation))
            rotation2 = transformations.quaternion_from_matrix(

                transformations.quaternion_matrix(transformations.quaternion_from_matrix(tt)) *
                transformations.quaternion_matrix(c.get_transform().rotation))

            # translation = transformations.translation_from_matrix(tttt)
            # rotation = transformations.quaternion_from_matrix(tttt)
            pcloud = pc2.create_cloud_xyz32(h, c.last_pc_transformed.to_list())
            transformed_cloud = c.transform_pcloud(c.last_pc, translation2, rotation2,
                                                   c.world_frame, c.world_frame)
            c.pc_pub.publish(transformed_cloud)'''

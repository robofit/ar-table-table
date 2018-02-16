#!/usr/bin/env python

import numpy as np
from tf import transformations


class ArtCalibrationHelper(object):

    def __init__(self):
        pass

    @staticmethod
    def transform_from_markers_positions(p0, p1, p2, p3, cell_id):
        """

        Args:
            p0:
            p1:
            p2:
            p3:

        Returns:

        """

        #      Markers on the table
        #
        #
        #             Robot
        #    11 ------------------12
        #    |                     |
        #    |                     |
        #    |                     |
        #    |                     |
        #    10-------------------13
        #
        if cell_id != "n1":
            pp01 = p1 - p0
            pp03 = p3 - p0
            pp01 = ArtCalibrationHelper.normalize_vector(pp01)
            pp03 = ArtCalibrationHelper.normalize_vector(pp03)
            if pp01 is None or pp03 is None:
                return None, None
            n = np.cross(pp03, pp01)
            m = np.matrix([[pp03[0], pp01[0], n[0], 0],
                           [pp03[1], pp01[1], n[1], 0],
                           [pp03[2], pp01[2], n[2], 0],
                           [0, 0, 0, 1]])

            translation_matrix = transformations.translation_matrix(p0)
            matrix = translation_matrix * m

            inverted_matrix = transformations.inverse_matrix(matrix)
        else:
            pp32 = p2 - p3
            pp03 = p3 - p0
            pp32 = ArtCalibrationHelper.normalize_vector(pp32)
            pp03 = ArtCalibrationHelper.normalize_vector(pp03)
            n = np.cross(pp03, pp32)
            m = np.matrix([[pp03[0], pp32[0], n[0], 0],
                           [pp03[1], pp32[1], n[1], 0],
                           [pp03[2], pp32[2], n[2], 0],
                           [0, 0, 0, 1]])
            translation_matrix = transformations.translation_matrix(p0)
            matrix = translation_matrix * m

            inverted_matrix = transformations.inverse_matrix(matrix)

        return p3, inverted_matrix

    @staticmethod
    def get_marker_position_by_id(markers, marker_id):
        for marker in markers.markers:
            if marker.id == marker_id:
                pos = np.array([0, 0, 0], dtype='f')
                pos[0] = marker.pose.pose.position.x
                pos[1] = marker.pose.pose.position.y
                pos[2] = marker.pose.pose.position.z

                return pos
        return None

    @staticmethod
    def normalize_vector(v):
        norm = np.linalg.norm(v)
        if norm <= 0.0000001:
            return None
        return v / norm

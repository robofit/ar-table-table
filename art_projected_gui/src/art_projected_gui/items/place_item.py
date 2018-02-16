#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from object_item import ObjectItem
from posestamped_cursor_item import PoseStampedCursorItem
from touch_table_item import TouchTableItem
from point_item import PointItem
from dialog_item import DialogItem
import tf
from art_projected_gui.helpers import conversions
import math
import rospy

translate = QtCore.QCoreApplication.translate

# TODO podle orientace nastavit Z


class PlaceItem(ObjectItem):

    """The class to visualize place pose for a given object / object type.

    """

    def __init__(self, scene, caption, x, y, z, quaternion, object_type, object_id=None, place_pose_changed=None, selected=False, fixed=False, txt=True, rot=True, rot_point=None, rotation_changed=None, parent=None, dashed=False):

        self.in_collision = False
        self.caption = caption
        self.dialog = None
        self.last_angle = None
        self.txt = txt
        self.rot = rot
        self.rot_point = rot_point
        self.other_items = []
        self.point = None

        # TODO temp "fix"
        if quaternion == (0, 0, 0, 1):
            quaternion = (0.707, 0, 0, 0.707)

        super(PlaceItem, self).__init__(scene, object_id, object_type, x, y, z, quaternion, parent=parent, dashed=dashed)

        self.update_text()
        self.fixed = fixed

        if not self.fixed:
            self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
            self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.place_pose_changed = place_pose_changed
        self.rotation_changed = rotation_changed

        if not self.fixed:
            self.set_color(QtCore.Qt.white)
            if self.rot:
                if rot_point is None:
                    self.point = PointItem(scene, 0, 0, self, self.point_changed)  # TODO option to pass pixels?
                    self.point.setPos(self.boundingRect().topLeft())

                    '''
                    self.dialog = DialogItem(self.scene(),
                                             self.pix2m(self.scene().width() / 2),
                                             0.1,
                                             translate(
                        "Place item",
                        "Object place pose options"),
                        [
                        translate(
                            "Place item", "Rotate |"),
                        translate(
                            "Place item", "Rotate --")
                    ],
                        self.dialog_cb)
                    '''

                else:
                    self.point = PointItem(scene, self.rot_point[0], self.rot_point[1], self, self.point_changed)

        if z == 0:

            self.position[2] = self.object_type.bbox.dimensions[self.get_yaw_axis()] / 2
            rospy.logdebug("PlaceItem - applying z value: " + str(self.position[2]))

    def dialog_cb(self, idx):

        # TODO animace: zesednout, srovnat, pustit timer a pak obratit
        # self.set_enabled(False)

        ax = self.get_yaw_axis()

        angle = 0

        # clear yaw & rotate
        if ax == PlaceItem.Z:

            sres = conversions.qv_mult(self.quaternion, (1, 0, 0))
            angle = math.atan2(sres[1], sres[0])

            self.position[2] = self.object_type.bbox.dimensions[0] / 2  # TODO fix this

        elif ax == PlaceItem.X or ax == PlaceItem.Y:

            sres = conversions.qv_mult(self.quaternion, (0, 0, 1))
            angle = math.atan2(sres[1], sres[0])

            self.position[2] = self.object_type.bbox.dimensions[2] / 2  # TODO fix this

        else:

            print "error"
            return

        angle = round(angle / (math.pi / 2)) * math.pi / 2

        axis = [0, 0, 0]
        axis[ax] = 1.0

        q = tf.transformations.quaternion_about_axis(angle, axis)

        self.set_orientation(q)

        # final rotation
        rot = (0, 0, 0, 1)

        if idx == 0:  # "Rotate |"

            rot = (0.707, 0, 0, 0.707)

        elif idx == 1:  # "Rotate --"

            rot = (0, 0.707, 0, 0.707)

        fin_q = tf.transformations.quaternion_multiply(self.quaternion, rot)

        self.set_orientation(fin_q)

        if self.point:
            self.point.setPos(self.boundingRect().topLeft())
        self._update_desc_pos()

    def itemChange(self, change, value):

        if change == QtGui.QGraphicsItem.ItemSceneChange:
            if self.dialog:  # and not value:

                self.scene().removeItem(self.dialog)
                self.dialog = None

        return QtGui.QGraphicsItem.itemChange(self, change, value)

    '''
        Method which updates the position of attribute "rot_point" (instance of PointItem class).
    '''

    def update_point(self):

        if self.rot_point is None:
            return

        self.point.set_pos(self.rot_point[0], self.rot_point[1])

    def update_text(self):

        if not self.txt:
            return

        if self.desc is None:
            return

        desc = self.caption

        if self.hover:

            # desc += "\n" + self.get_pos_str()
            if self.object_id is not None:
                desc += "\n" + translate("ObjectItem", "ID: ") + str(self.object_id)
            else:
                desc += "\n" + self.object_type.name

        self.desc.set_content(desc)

    def mouseReleaseEvent(self, event):

        self.cursor_release()
        super(PlaceItem, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):

        pos = self.get_pos()

        self.position[0] = pos[0]
        self.position[1] = pos[1]
        self.item_moved()
        super(PlaceItem, self).mouseMoveEvent(event)

    def cursor_release(self):

        # TODO call base class method

        if self.place_pose_changed is not None:
            self.place_pose_changed(self)

    def cursor_press(self):

        pass

    def point_changed(self, pt, finished=False):

        # follow angle between "free" point and object center, after release put object back on topLeft corner
        angle = math.atan2(self.point.scenePos().y() - self.scenePos().y(), self.point.scenePos().x() - self.scenePos().x()) + 2.355

        if self.last_angle is None:

            self.last_angle = angle
            return

        ax = self.get_yaw_axis()

        axis = [0, 0, 0]
        axis[ax] = 1.0

        # print ("pc-quaternion-before", self.quaternion)

        q = tf.transformations.quaternion_about_axis(-(angle - self.last_angle), axis)

        self.set_orientation(tf.transformations.quaternion_multiply(self.quaternion, q))

        # print ("pc-quaternion-after", self.quaternion)

        if self.other_items:    # changing rotation of every object in grid
            for it in self.other_items:
                it.setRotation(self.rotation())

        self._update_desc_pos()

        if finished:

            self.last_angle = None

            self.item_moved()
            for it in self.other_items:
                it.item_moved()

            if self.rot_point is None:
                self.point.setPos(self.boundingRect().topLeft())
            else:
                self.point.set_pos(self.rot_point[0], self.rot_point[1])

            if self.rotation_changed is not None:
                in_collision = False
                for it in ([self] + self.other_items):
                    if it.in_collision:
                        in_collision = True
                        break
                if in_collision:
                    self.rotation_changed([])  # in case of collision, new rotations of objects are NOT saved into the ProgramItem message
                else:
                    self.rotation_changed([self] + self.other_items)    # saving new rotations of objects into the ProgramItem message

            if self.place_pose_changed is not None:
                self.place_pose_changed(self)

        self.last_angle = angle

    def item_moved(self):

        for it in self.collidingItems():

            if isinstance(it, PoseStampedCursorItem) or isinstance(it, TouchTableItem):
                continue

            if isinstance(it, PlaceItem):
                self.in_collision = True
                self.set_color(QtCore.Qt.red)
                break
        else:
            self.in_collision = False
            self.set_color(QtCore.Qt.white)

    '''
        Method which sets attribute "other_items".
    '''

    def set_other_items(self, items):
        self.other_items = items

#!/usr/bin/env python

"""
Visualization of detected object(s).

"""

from PyQt4 import QtGui, QtCore
from item import Item
from desc_item import DescItem
import math
import numpy as np
import tf
from art_projected_gui.helpers import conversions

translate = QtCore.QCoreApplication.translate


class ObjectItem(Item):

    """The class to visualize (detected) object.

    It currently supports only rotation around z-axis.

    """

    def __init__(self, scene, object_id, object_type, x,
                 y, z, quaternion=(0, 0, 0, 1), sel_cb=None, selected=False, parent=None, dashed=False):

        self.object_id = object_id
        self.selected = selected
        self.sel_cb = sel_cb
        # TODO check bbox type and use rectangle (used now) / ellipse, consider
        # other angles
        self.object_type = object_type
        self.inflate = 0.01
        self.def_color = QtCore.Qt.gray
        self.lx = 0
        self.ly = 0
        self.dashed = dashed

        self.desc = None
        self.quaternion = (0, 0, 0, 1)
        self.on_table = False

        super(ObjectItem, self).__init__(scene, x, y, z, parent=parent)

        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.desc = DescItem(scene, 0, 0, parent=self)
        self.desc.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)

        self.update_text()

        self.set_orientation(quaternion)

        if selected:
            self.set_selected()

        self._update_desc_pos()
        self.setZValue(50)

    def set_color(self, color=QtCore.Qt.gray):

        self.def_color = color
        self.update()

    def _update_desc_pos(self):

        if self.desc is not None:

            # make upper left corner of description aligned with left extent of
            # the (possibly rotated) object bounding box (highlight area)
            self.desc.setPos(
                self.mapFromScene(
                    self.x() -
                    self.sceneBoundingRect().width() /
                    2,
                    self.y() +
                    self.sceneBoundingRect().height() /
                    2 +
                    self.m2pix(0.01)))

    def set_pos(self, x, y, z=None, parent_coords=False):

        super(ObjectItem, self).set_pos(x, y, z, parent_coords)
        self._update_desc_pos()
        self.update_text()

    def get_yaw_axis(self):

        ax = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

        c_idx = None
        c_dist = None

        for idx in range(len(ax)):

            res = conversions.qv_mult(self.quaternion, ax[idx])

            dist = math.sqrt(res[0]**2 + res[1]**2)

            if c_dist is None or c_dist > dist:

                c_dist = dist
                c_idx = idx

        # TODO disable object (return -1) if dist is too high?

        return c_idx

    def set_orientation(self, q):

        self.quaternion = q

        ax = self.get_yaw_axis()

        if ax == ObjectItem.Z:

            self.lx = self.m2pix(self.inflate + self.object_type.bbox.dimensions[0])
            self.ly = self.m2pix(self.inflate + self.object_type.bbox.dimensions[1])

            sres = conversions.qv_mult(self.quaternion, (1, 0, 0))
            angle = math.atan2(sres[1], sres[0])

            self.on_table = self.position[2] < self.object_type.bbox.dimensions[2] + 0.05

        elif ax in [ObjectItem.X, ObjectItem.Y]:

            res = conversions.qv_mult(self.quaternion, (0, 0, 1))

            self.lx = self.m2pix(self.inflate + self.object_type.bbox.dimensions[2])

            # TODO use correct dimension (x/y) - now let's assume that x and y dimensions are same
            self.ly = self.m2pix(self.inflate + self.object_type.bbox.dimensions[1])

            angle = math.atan2(res[1], res[0])

            self.on_table = self.position[2] < self.object_type.bbox.dimensions[0] + 0.05

        else:

            self.set_enabled(False, True)
            return

        self.setRotation(-angle / (math.pi * 2) * 360)

        # TODO if not on table - display somewhere list of detected objects or what?
        self.set_enabled(self.on_table, True)

        self.update()

    def update_text(self):

        if self.desc is None:
            return

        desc = ""
        desc += translate("ObjectItem", "ID: ") + self.object_id

        if self.hover:

            desc += "\n" + translate("ObjectItem",
                                     "TYPE: ") + self.object_type.name
            # desc += "\n" + self.get_pos_str()

        self.desc.set_content(desc)

    def hover_changed(self):

        self.update_text()
        self.update()

    def boundingRect(self):

        if not self.scene():

            return QtCore.QRectF()

        p = 10.0
        return QtCore.QRectF(-self.lx / 2 - p, -self.ly / 2 - p, self.lx + 2 * p, self.ly + 2 * p)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        if not self.on_table:
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        rr = 10

        painter.setBrush(QtCore.Qt.NoBrush)
        line = QtCore.Qt.SolidLine
        if self.dashed:
            line = QtCore.Qt.DashLine
        pen = QtGui.QPen(self.def_color, 5, line, QtCore.Qt.RoundCap)

        if self.selected:

            pen.setColor(QtCore.Qt.green)
            pen.setWidth(10)

        elif self.hover:

            pen.setWidth(10)

        painter.setPen(pen)

        painter.drawRoundedRect(-self.lx / 2, -self.ly / 2, self.lx,
                                self.ly, rr, rr, QtCore.Qt.RelativeSize)

    def cursor_click(self):

        # TODO call base class method

        if self.sel_cb is not None:
            # callback should handle object selection
            self.sel_cb(self.object_id, self.selected)

        else:
            # no callback - object will handle its selection
            if not self.selected:
                self.set_selected()
            else:
                self.set_selected(False)

    def set_selected(self, selected=True):

        if selected:

            self.selected = True
            # rospy.logdebug('Object ID ' + self.object_id + ' selected')

        else:

            self.selected = False
            # rospy.logdebug('Object ID ' + self.object_id + ' unselected')

        self.update()

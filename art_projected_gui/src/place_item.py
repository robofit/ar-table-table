#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from item import Item

class PlaceItem(Item):

    def __init__(self,  scene,  rpm, caption,  x,  y,  place_pose_changed=None,  outline_diameter=0.1,  selected = False,  fixed=False):

        self.fixed = fixed
        self.outline_diameter = outline_diameter
        self.caption = caption
        self.in_collision = False
        self.place_pose_changed = place_pose_changed

        super(PlaceItem,  self).__init__(scene,  rpm,  x,  y)

        if not self.fixed:
            self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
            self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

    def mouseReleaseEvent(self,  evt):

        if self.place_pose_changed is not None: self.place_pose_changed(self.get_pos())
        super(Item, self).mouseReleaseEvent(evt)

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter)
        return QtCore.QRectF(-es/2, -es/2, es, es)

    def mouseMoveEvent(self, event):

        # TODO testovat kolize jen s PlaceItem?
        if len(self.collidingItems()) > 0: self.in_collision = True
        else: self.in_collision = False

        super(Item, self).mouseMoveEvent(event)

    def paint(self, painter, option, widget):

        es = self.m2pix(self.outline_diameter)

        if self.fixed:
            painter.setBrush(QtCore.Qt.gray)
        elif not self.in_collision:
            painter.setBrush(QtCore.Qt.cyan)
        else:
            painter.setBrush(QtCore.Qt.red)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)

        painter.setFont(QtGui.QFont('Arial', 12));

        painter.setPen(QtCore.Qt.gray)

        if self.hover:

            painter.setPen(QtCore.Qt.white)
            painter.drawText(-es/2,  es/2+40, self.get_pos_str())

        painter.drawText(-es/2,  es/2+20, self.caption);

#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item


class PointItem(Item):

    def __init__(self, scene, x, y, parent, changed_cb=None, fixed=False):

        self.outline_diameter = 0.025
        self.changed_cb = changed_cb
        super(PointItem, self).__init__(scene, x, y, parent=parent)
        self.fixed = fixed

        if not self.fixed:
            self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
            self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

    def boundingRect(self):

        if not self.scene():
            return QtCore.QRectF()

        es = self.m2pix(self.outline_diameter * 1.8)
        return QtCore.QRectF(-es / 2, -es / 2, es, es)

    def shape(self):

        path = QtGui.QPainterPath()

        if not self.scene():
            return path

        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
        return path

    def mouseMoveEvent(self, event):

        self.changed_cb(self, False)
        super(Item, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):

        self.changed_cb(self, True)
        super(Item, self).mouseReleaseEvent(event)

    def item_moved(self):

        if self.changed_cb is not None:
            self.changed_cb(self, False)

    def cursor_release(self):

        self.changed_cb(self, True)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        if self.fixed:
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover:
            # TODO coordinates
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0, 0), es / 2 * 1.8, es / 2 * 1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)

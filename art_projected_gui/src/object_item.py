#!/usr/bin/env python

"""
Visualization of detected object(s)

TODO:
 - preselect (highlight)
 - display additional information on highlight
 - diameter based on boundingbox size?
"""

from PyQt4 import QtGui, QtCore
from item import Item

translate = QtCore.QCoreApplication.translate

class ObjectItem(Item):

    def __init__(self,  scene,  rpm,  object_id,  object_type, x,  y,  sel_cb = None,  outline_diameter=0.1,  selected = False):

        self.object_id = object_id
        self.object_type = object_type
        self.outline_diameter = outline_diameter
        self.selected = selected
        self.sel_cb = sel_cb

        super(ObjectItem,  self).__init__(scene,  rpm,  x,  y)

        if selected: self.set_selected()

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter)

        return QtCore.QRectF(-es/2, -es/2, es, es)

    def paint(self, painter, option, widget):

        eso = self.m2pix(self.outline_diameter*1.3)
        es = self.m2pix(self.outline_diameter)

        if self.selected:

            painter.setBrush(QtCore.Qt.green)
            painter.setPen(QtCore.Qt.green)

            painter.drawEllipse(QtCore.QPoint(0,  0),  eso/2,  eso/2)

        elif self.hover:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)

            painter.drawEllipse(QtCore.QPoint(0,  0),  eso/2,  eso/2)

            # TODO disp add info

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        painter.drawEllipse(QtCore.QPoint(0,  0),  es/2,  es/2)

        painter.setPen(QtCore.Qt.gray)

        # TODO font size
        painter.setFont(QtGui.QFont('Arial', 12));

        if self.hover:

            painter.setPen(QtCore.Qt.white)
            painter.drawText(-eso/2,  eso/2+20+20, translate("ObjectItem", "TYPE: ") + self.object_type);
            painter.drawText(-eso/2,  eso/2+20+40, self.get_pos_str())

        painter.drawText(-eso/2,  eso/2+20, self.object_id);

    def mouseDoubleClickEvent(self,  evt):

        if self.sel_cb is not None:
            # callback should handle object selection
            self.sel_cb(self.object_id,  self.selected)

        else:
            # no callback - object will handle its selection
            if not self.selected: self.set_selected()
            else: self.set_selected(False)


    def set_selected(self,  selected = True):

        if selected:

            self.selected = True
            #rospy.logdebug('Object ID ' + self.object_id + ' selected')

        else:

            self.selected = False
            #rospy.logdebug('Object ID ' + self.object_id + ' unselected')

        self.update()

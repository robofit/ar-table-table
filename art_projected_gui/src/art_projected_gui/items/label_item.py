#!/usr/bin/env python

from PyQt4 import QtGui, QtCore, QtSvg
from item import Item
import rospy
from art_msgs.srv import NotifyUserRequest
import rospkg


class LabelItem(Item):

    def __init__(self, scene, x, y, w, h):

        self.w = w
        self.h = h

        super(LabelItem, self).__init__(scene, x, y)

        self.message = None
        self.temp_msgs = []

        rospack = rospkg.RosPack()
        self.icons_path = rospack.get_path('art_projected_gui') + '/icons/'

        self.icons = {}
        self.icons[NotifyUserRequest.INFO] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-information.svg', self)
        self.icons[NotifyUserRequest.WARN] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-warning.svg', self)
        self.icons[NotifyUserRequest.ERROR] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_emblem-important.svg', self)
        self.icons[NotifyUserRequest.YES_NO_QUESTION] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-question.svg', self)

        s = self.m2pix(self.h)
        for k, v in self.icons.iteritems():

            v.setScale(s / v.boundingRect().height())
            v.setPos(0, 0)
            v.setVisible(False)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(200)

    def add_msg(self, msg, message_type, min_duration=rospy.Duration(3), temp=False):

        md = {"msg": msg, "min_duration": min_duration, "shown_at": None, "type": message_type}

        if temp:
            self.temp_msgs.append(md)
        else:
            self.message = md

        self.update()

    def boundingRect(self):

        w = self.m2pix(self.w)
        h = self.m2pix(self.h)

        return QtCore.QRectF(0, 0, w, h)

    def prune_old_msgs(self):

        msgs_to_delete = []

        for msg in self.temp_msgs:

            if msg["shown_at"] is not None and rospy.Time.now() - msg["shown_at"] > msg["min_duration"]:

                msgs_to_delete.append(msg)

        for msg in msgs_to_delete:

            self.temp_msgs.remove(msg)

        return len(msgs_to_delete) > 0

    def timer_cb(self, evt):

        if self.prune_old_msgs():
            self.update()

    def cursor_click(self):

        try:
            del self.temp_msgs[0]
            rospy.logdebug("Temp notification removed.")
            self.update()
        except IndexError:
            rospy.logdebug("No temp notification to be removed.")

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        try:
            msg = self.temp_msgs[0]
        except IndexError:
            msg = self.message

        if msg is None:
            return

        if msg["shown_at"] is None:

            msg["shown_at"] = rospy.Time.now()

        for k, v in self.icons.iteritems():

            if k == msg["type"]:
                v.setVisible(True)
            else:
                v.setVisible(False)

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        # TODO fix for multiline messages
        font = QtGui.QFont('Arial', self.get_font_size(1.2))
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        h = self.m2pix(self.h)
        left_padding = h + self.m2pix(0.02)

        painter.drawText(left_padding, (h - metrics.height()) / 2, self.m2pix(self.w) - left_padding, h, QtCore.Qt.AlignLeft, msg["msg"])

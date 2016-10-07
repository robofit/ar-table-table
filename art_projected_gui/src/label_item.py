#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from collections import deque

class LabelItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  w,  h):

        self.w = w
        self.h = h

        super(LabelItem,  self).__init__(scene,  rpm,  x,  y)

        # TODO list of messages - display them intelligently, optional duration of message etc
        self.msgs = deque()
        self.current_msg = None

        self.still_msgs = []
        self.temp_msgs = []

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def add_msg(self,  msg,  min_duration=rospy.Duration(3),  temp=False):

        self.msgs.append({"msg": msg,  "min_duration": min_duration,  "shown_at": None,  "temp": temp})
        self.check_for_msgs()

    def boundingRect(self):

        w = self.m2pix(self.w)
        h = self.m2pix(self.h)

        return QtCore.QRectF(0,  0, w, h)

    def prune_old_msgs(self,  arr):

        msgs_to_delete = []

        for msg in arr:

            if msg["temp"] is False: # still messages

                if len(arr) <= 1: # nothing to do if there is only one message

                    break

                if len(msgs_to_delete)==0: msgs_to_delete.append(msg)
                else:

                    if msg["shown_at"] is not None and rospy.Time.now() - msg["shown_at"] > msg["min_duration"] and msg["shown_at"] < msgs_to_delete[0]["shown_at"]:

                        msgs_to_delete[0] = msg

            else:

                if msg["shown_at"] is not None and rospy.Time.now() - msg["shown_at"] > msg["min_duration"]:

                    msgs_to_delete.append(msg)

        for msg in msgs_to_delete:

            arr.remove(msg)

    def check_for_msgs(self):

        if len(self.msgs) > 0:

            tmp = self.msgs.popleft()

            if tmp["temp"] is False:

                self.still_msgs.append(tmp)

            else:

                self.temp_msgs.append(tmp)

        if len(self.still_msgs) > 0 or len(self.temp_msgs) > 0: self.update()

    def timer_cb(self,  evt):

        self.prune_old_msgs(self.still_msgs)
        self.prune_old_msgs(self.temp_msgs)

        self.update()

    def paint(self, painter, option, widget):

        msg = None

        if len(self.temp_msgs) > 0:msg = self.temp_msgs[0]
        elif len(self.still_msgs) > 0: msg = self.still_msgs[0]

        if msg is None: return

        if msg["shown_at"] is None: msg["shown_at"] = rospy.Time.now()

        # TODO animate text as it is shown / disappears

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        # TODO font size
        painter.setFont(QtGui.QFont('Arial', 18))

        painter.drawText(0,  0, msg["msg"])

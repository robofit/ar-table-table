#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem
from desc_item import DescItem

translate = QtCore.QCoreApplication.translate


class DialogItem(Item):

    def __init__(self, scene, x, y, caption, answers, done_cb, parent=None):

        self.w = 0
        self.h = 0

        super(DialogItem, self).__init__(scene, x, y, parent=parent)

        self.sp = self.m2pix(0.01)
        self.done_cb = done_cb
        self.fixed = False

        self.desc = DescItem(scene, 0, 0, self)
        self.desc.set_content(caption)

        self.items = []

        # TODO create button, measure them, arrange them
        self.w = (len(answers) + 1) * self.sp

        for answer in answers:

            btn = ButtonItem(self.scene(), 0, 0, answer, self, self.answer_cb)
            self.items.append(btn)
            self.w += btn._width()

        y = self.sp
        self._place_childs_horizontally(y, self.sp, [self.desc])
        y += self.desc._height() + self.sp
        self._place_childs_horizontally(y, self.sp, self.items)
        y += self.items[0]._height()
        y += self.sp

        self.h = y

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.update()
        self.setZValue(300)

    def answer_cb(self, btn):

        self.done_cb(self.items.index(btn))

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def set_caption(self, caption):
        self.desc.set_content(caption)

#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

# TODO icons

class ButtonItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  caption,  clicked):

        self.w = 180 # TODO spocitat podle rpm
        self.h = 20
        self.clicked = clicked
        self.caption = caption

        super(ButtonItem,  self).__init__(scene,  rpm,  x,  y)

    def boundingRect(self):

        return QtCore.QRectF(0,  0, self.w, self.h)

    def mouseDoubleClickEvent(self,  evt):

        if self.clicked is not None: self.clicked()

    def paint(self, painter, option, widget):

        rect = QtCore.QRectF(5, 0.0, self.w, 40.0)

        font = QtGui.QFont('Arial', 12)
        painter.setFont(font);
        painter.setPen(QtCore.Qt.white)
        painter.setBrush(QtCore.Qt.gray)
        metrics = QtGui.QFontMetrics(font)
        txt = self.caption
        rect.setWidth(metrics.width(txt)+20)

        painter.drawRoundedRect(rect, 10.0, 10.0);
        painter.drawText(rect,  QtCore.Qt.AlignCenter,  txt)


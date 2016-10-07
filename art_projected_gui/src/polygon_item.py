#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

class PolygonPointItem(Item):

    def __init__(self,  scene,  rpm, x,  y,  parent):

        self.outline_diameter = 0.025

        super(PolygonPointItem,  self).__init__(scene,  rpm,  x,  y,  parent)

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        # TODO pri pohybu bodu upravit boundingRect rodice...

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter)

        return QtCore.QRectF(-es/2, -es/2, es, es)

    def mouseMoveEvent(self, event):

        self.parentItem().point_changed()

        super(Item, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self,  evt):

        self.parentItem().point_changed(True)
        super(Item, self).mouseReleaseEvent(evt)

    def paint(self, painter, option, widget):

        es = self.m2pix(self.outline_diameter)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)

class PolygonItem(Item):

    def __init__(self,  scene,  rpm, caption, obj_coords = [],  poly_points=[],  polygon_changed=None):

        self.caption = caption
        self.polygon_changed = polygon_changed

        if len(obj_coords) > 0:

            # podle objects_coords vygenerovat body - nejak urcit x, y

            self.min = [obj_coords[0][0], obj_coords[0][1]]
            self.max = [obj_coords[0][0], obj_coords[0][1]]

            for pt in obj_coords:

                if pt[0] < self.min[0]: self.min[0] = pt[0]
                if pt[1] < self.min[1]: self.min[1] = pt[1]
                if pt[0] > self.max[0]: self.max[0] = pt[0]
                if pt[1] > self.max[1]: self.max[1] = pt[1]

            pad = 0.1

            self.min[0] -= pad
            self.min[1] -= pad
            self.max[0] += pad
            self.max[1] += pad

            super(PolygonItem,  self).__init__(scene,  rpm,  self.min[0],  self.min[1])

            self.pts = []

            self.pts.append(PolygonPointItem(scene,  rpm,  self.min[0],  self.min[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.max[0], self.min[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.max[0],  self.max[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.min[0], self.max[1],  self))

            if self.polygon_changed is not None:
                self.polygon_changed(self.get_poly_points())

        elif len(poly_points) > 0:

            self.min = [poly_points[0][0], poly_points[0][1]]
            self.max = [poly_points[0][0], poly_points[0][1]]

            for pt in poly_points:

                if pt[0] < self.min[0]: self.min[0] = pt[0]
                if pt[0] > self.max[0]: self.max[0] = pt[0]

            super(PolygonItem,  self).__init__(scene,  rpm,  self.min[0],  self.min[1])

            self.pts = []

            for pt in poly_points:

                self.pts.append(PolygonPointItem(scene,  rpm,  pt[0],  pt[1],  self))

        else:

            pass # TODO chyba

        self.update()

    def point_changed(self,  finished = False):

        # update of bounding rect
        self.min[0] = self.pts[0].get_pos()[0]
        self.min[1] = self.pts[0].get_pos()[1]
        self.max[0] = self.pts[0].get_pos()[0]
        self.max[1] = self.pts[0].get_pos()[1]

        for pt in self.pts:

            p = pt.get_pos()

            if p[0] < self.min[0]: self.min[0] = p[0]
            if p[1] < self.min[1]: self.min[1] = p[1]

            if p[0] > self.max[0]: self.max[0] = p[0]
            if p[1] > self.max[1]: self.max[1] = p[1]

        self.update()

        if finished and self.polygon_changed is not None:
                self.polygon_changed(self.get_poly_points())

    def get_poly_points(self):

        pts = []

        for pt in self.pts:

            pts.append(pt.get_pos())

        return pts

    def boundingRect(self):

        return QtCore.QRectF(self.min[0],  self.min[1], self.max[0], self.max[1])

    def paint(self, painter, option, widget):

        # TODO detekovat ze je polygon "divny" (prekrouceny) a zcervenat
        # TODO vypsat kolik obsahuje objektu

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        pen.setWidth(5)
        pen.setBrush(QtCore.Qt.white)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)

        painter.setPen(pen)

        poly = QtGui.QPolygon()

        for i in range(0,  len(self.pts)):

            poly.append(self.pts[i].pos().toPoint())

        painter.drawPolygon(poly)

        # TODO nazev polygonu -> opravit souradnice
        #painter.setFont(QtGui.QFont('Arial', 12));
        #painter.setPen(QtCore.Qt.white)
        #painter.drawText(QtCore.QPoint(self.m2pix(self.min[0]+self.get_pos(pixels=True)[0]), self.m2pix(self.max[1]) + self.get_pos(pixels=True)[1]), self.caption)

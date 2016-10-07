#!/usr/bin/env python

"""
Module contains the main class of 2D projected UI (UICore). It produces 2D image of given size. This image is send to Project class(es) to be displayed.

"""

from PyQt4 import QtGui, QtCore
from object_item import ObjectItem
from place_item import PlaceItem
from label_item import LabelItem
from program_item import ProgramItem
from polygon_item import PolygonItem
import rospy

# z tohodle vyleze 2D obraz
# scene_res -> rozliseni v jakem se ma scena udrzovat
# world_coords -> souradnice rohu obrazu
class UICore(QtCore.QObject):

    def __init__(self,  x,  y,  width,  height):

        super(UICore, self).__init__()

        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.rpm = 1280.0 # resolution per meter ;)

        # TODO make 'DPI' configurable
        w = self.width*self.rpm
        h = self.height/self.width*w

        self.scene = QtGui.QGraphicsScene(0, 0,  int(w),  int(h))
        self.scene.setBackgroundBrush(QtCore.Qt.black)

        self.scene.changed.connect(self.scene_changed)

        self.bottom_label = LabelItem(self.scene,  self.rpm,  0.1,  self.height - 0.05,  self.width-0.2,  0.1)
        self.program_vis = ProgramItem(self.scene,  self.rpm,  0.1,  0.1)

        self.projectors = []

        self.scene_items = []

    def notif(self,  msg,  min_duration=rospy.Duration(3),  temp = False):

        self.bottom_label.add_msg(msg,  min_duration,  temp)

    def add_projector(self,  proj):

        self.projectors.append(proj)

    def debug_view(self):

        self.view = QtGui.QGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet( "QGraphicsView { border-style: none; }" )

        self.view.show()

    def scene_changed(self,  rects):

        pix = QtGui.QPixmap(self.scene.width(), self.scene.height())
        painter = QtGui.QPainter(pix)
        self.scene.render(painter)
        painter.end()

        for proj in self.projectors:

            proj.set_img(pix)

    def get_scene_items_by_type(self,  type):

       for el in self.scene_items:
           if isinstance(el,  type): yield el

    def remove_scene_items_by_type(self,  type):

        its = []

        for it in self.scene_items:

            if not isinstance(it, type): continue
            its.append(it)

        for it in its:

            self.scene.removeItem(it)
            self.scene_items.remove(it)

    def add_object(self,  object_id,  object_type,  x,  y,  sel_cb):

        self.scene_items.append(ObjectItem(self.scene,  self.rpm, object_id,  object_type,  x,  y,  sel_cb))

    def remove_object(self,  object_id):

        obj = None

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == object_id:

                obj = it
                break

        if obj is not None:

            self.scene.removeItem(obj)
            self.scene_items.remove(obj)
            return True

        return False

    def select_object(self,  obj_id,  unselect_others=True):

        if obj_id == "": return

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == obj_id:

                it.set_selected(True)
                if not unselect_others: break

            elif unselect_others:

                it.set_selected(False)

    def select_object_type(self,  obj_type,  unselect_others=True):

        if obj_type == "": return

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_type == obj_type:
                it.set_selected(True)
            elif unselect_others: it.set_selected(False)

    def get_object(self,  obj_id):

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == obj_id: return it

        return None

    def add_place(self,  caption,  x,  y,  place_cb=None,  fixed=False):

        self.scene_items.append(PlaceItem(self.scene,  self.rpm,  caption,  x,  y,  place_pose_changed=place_cb,  fixed=fixed))

    def add_polygon(self,  caption,  obj_coords=[],  poly_points=[],  polygon_changed=None):

        self.scene_items.append(PolygonItem(self.scene,  self.rpm,  caption,  obj_coords,  poly_points, polygon_changed))

    def clear_places(self):

        self.remove_scene_items_by_type(PlaceItem)

    def clear_all(self):

        for it in self.get_scene_items_by_type(ObjectItem):

            it.set_selected(False)

        self.remove_scene_items_by_type(PlaceItem)
        self.remove_scene_items_by_type(PolygonItem)

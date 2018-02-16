#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from art_msgs.msg import Touch
from desc_item import DescItem
from button_item import ButtonItem
from label_item import LabelItem


class TouchPointItem(Item):

    def __init__(self, scene, x, y, parent, show=False):

        self.pointed_item = None
        self.offset = (0, 0)
        self.last_update = rospy.Time.now()
        self.show = show

        super(TouchPointItem, self).__init__(scene, x, y, parent)

        self.set_poss(x, y)

    def boundingRect(self):

        return QtCore.QRectF(-1, -1, 2, 2)  # TODO make this configurable

    def end_of_touch(self):

        if self.pointed_item is not None:

            rospy.logdebug("releasing pointed item: " + self.pointed_item.__class__.__name__)
            self.pointed_item.set_hover(False, self)
            self.pointed_item.cursor_release()
            self.pointed_item = None

    def set_poss(self, x, y):

        self.last_update = rospy.Time.now()
        self.set_pos(x, y)

        if self.pointed_item is None:

            for it in self.scene().items():

                if not isinstance(it, Item):  # TODO skip item not derived from Item
                    continue

                if not it.isVisible():
                    continue

                # TODO what types to skip?
                if isinstance(it, TouchTableItem) or isinstance(it, TouchPointItem) or isinstance(it, DescItem):
                    continue

                if self.collidesWithItem(it):

                    parent = it.parentItem()
                    if not isinstance(it, ButtonItem) and it.fixed and parent is not None and isinstance(parent, Item) and not parent.fixed:

                        it = it.parentItem()

                    rospy.logdebug("new pointed item: " + it.__class__.__name__)

                    it.set_hover(True, self)
                    self.pointed_item = it
                    self.pointed_item.cursor_press()

                    my_pos = self.get_pos()
                    it_pos = self.pointed_item.get_pos()

                    self.offset = (it_pos[0] - my_pos[0], it_pos[1] - my_pos[1])

                    break

        else:

            if self.pointed_item.fixed:

                if not self.collidesWithItem(self.pointed_item):

                    self.pointed_item.set_hover(False, self)
                    self.pointed_item = None

            else:

                self.pointed_item.set_pos(x + self.offset[0], y + self.offset[1])
                self.pointed_item.item_moved()

        self.update()

    def paint(self, painter, option, widget):

        if not self.scene() or not self.show:
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self.pointed_item is not None:
            painter.setBrush(QtCore.Qt.red)
        else:
            painter.setBrush(QtCore.Qt.green)

        painter.setOpacity(0.5)
        painter.drawEllipse(QtCore.QPoint(0, 0), 2, 2)  # TODO draw based on b.r. size


# TODO make a common helper class from this (also used in PoseStampedCursorItem)
class TouchTableItemHelper(QtCore.QObject):

    def __init__(self, topic, world_frame, cb):

        super(TouchTableItemHelper, self).__init__()

        self.ps_sub = rospy.Subscriber(topic, Touch, self.ps_cb)
        self.cb = cb

        QtCore.QObject.connect(self, QtCore.SIGNAL('ps_cb'), self.ps_cb_evt)

    def ps_cb(self, msg):

        self.emit(QtCore.SIGNAL('ps_cb'), msg)

    def ps_cb_evt(self, msg):

        self.cb(msg)


class TouchTableItem(Item):

    # TODO display corners of touchable area
    def __init__(self, scene, topic, items_to_disable_on_touch=[], world_frame="marker", show_touch_points=False):

        super(TouchTableItem, self).__init__(scene, 0.0, 0.0)
        self.touch_points = {}

        self.show_touch_points = show_touch_points
        self.items_to_disable_on_touch = items_to_disable_on_touch

        self.helper = TouchTableItemHelper(topic, world_frame, self.touch_cb)

        self.setZValue(400)

        self.prune_timer = QtCore.QTimer()

        QtCore.QObject.connect(self.prune_timer, QtCore.SIGNAL(
            'timeout()'), self.prune_timer_evt)
        self.prune_timer.start(0.5 * 1000)

    def prune_timer_evt(self):

        to_delete = []
        now = rospy.Time.now()
        for k, v in self.touch_points.iteritems():

            if now - v.last_update > rospy.Duration(self.prune_timer.interval() / 1000.0):
                to_delete.append(k)

        for k in to_delete:
            rospy.logdebug("deleting outdated touch, id: " + str(k))
            self.delete_id(k)

    def boundingRect(self):

        return self.scene().sceneRect()

    def paint(self, painter, option, widget):

        pass

    def delete_id(self, id):

        if id not in self.touch_points:
            return

        rospy.logdebug("deleting touch point, id: " + str(id))
        self.touch_points[id].end_of_touch()
        self.scene().removeItem(self.touch_points[id])
        del self.touch_points[id]

    def touch_cb(self, msg):

        # TODO delete all points on disable?
        if not self.isEnabled():
            return

        touches = len(self.touch_points)

        if msg.id not in self.touch_points and msg.touch:

            # TODO check frame_id
            rospy.logdebug("new touch point, id: " + str(msg.id))
            self.touch_points[msg.id] = TouchPointItem(self.scene(), msg.point.point.x, msg.point.point.y, self, show=self.show_touch_points)

        else:

            if not msg.touch:

                rospy.logdebug("end of touch, id: " + str(msg.id))
                self.delete_id(msg.id)

            else:

                rospy.logdebug("update of touch, id: " + str(msg.id))
                self.touch_points[msg.id].set_poss(msg.point.point.x, msg.point.point.y)

        # enable / disable given items
        if len(self.touch_points) > 0 and touches == 0:

            rospy.logdebug("disabling")

            for cur in self.items_to_disable_on_touch:
                cur.set_enabled(False, True)

        if len(self.touch_points) == 0 and touches > 0:

            rospy.logdebug("enabling")

            for cur in self.items_to_disable_on_touch:
                cur.set_enabled(True, True)

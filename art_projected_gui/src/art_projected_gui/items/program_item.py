#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt, LearningRequestGoal
from geometry_msgs.msg import Point32, Pose
from button_item import ButtonItem
from art_projected_gui.helpers import conversions
from list_item import ListItem
from art_projected_gui.helpers.items import group_enable, group_visible
from geometry_msgs.msg import PoseStamped

translate = QtCore.QCoreApplication.translate


class ProgramItem(Item):

    def __init__(self, scene, x, y, program_helper, done_cb=None, item_switched_cb=None, learning_request_cb=None, pause_cb=None, cancel_cb=None, stopped=False):

        self.w = 100
        self.h = 100

        self.done_cb = done_cb
        self.item_switched_cb = item_switched_cb
        self.learning_request_cb = learning_request_cb
        self.pause_cb = pause_cb
        self.cancel_cb = cancel_cb

        self.readonly = False
        self.stopped = stopped

        super(ProgramItem, self).__init__(scene, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)
        self.sp = self.m2pix(0.005)

        self.ph = program_helper

        self.block_id = None
        self.item_id = None

        self.block_learned = False
        self.program_learned = False

        # block "view"
        self.block_finished_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Done"), self, self.block_finished_btn_cb)
        self.block_edit_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Edit"), self, self.block_edit_btn_cb)
        self.block_on_failure_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "On fail"), self, self.block_on_failure_btn)

        bdata = []

        self.blocks_map = {}  # map from indexes (key) to block_id (value)
        self.blocks_map_rev = {}

        for i in range(len(self.ph.get_program().blocks)):

            bmsg = self.ph.get_program().blocks[i]

            bdata.append(translate("ProgramItem", "Block %1\n%2").arg(bmsg.id).arg(bmsg.name))
            idx = len(bdata) - 1
            self.blocks_map[idx] = bmsg.id
            self.blocks_map_rev[bmsg.id] = idx

        self.blocks_list = ListItem(self.scene(), 0, 0, 0.2 - 2 * 0.005, bdata, self.block_selected_cb, parent=self)

        for k, v in self.blocks_map.iteritems():

            self._update_block(v)

        y = 50
        self.blocks_list.setPos(self.sp, y)
        y += self.blocks_list._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [
                                         self.block_finished_btn, self.block_edit_btn, self.block_on_failure_btn])

        y += self.block_finished_btn._height() + self.sp

        group_enable((self.block_edit_btn, self.block_on_failure_btn), False)

        self.h = y

        # items "view"
        self.item_edit_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Edit"), self, self.item_edit_btn_cb)
        self.item_run_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Run"), self, self.item_run_btn_cb)
        self.item_on_success_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "On S"), self, self.item_on_success_btn_cb)
        self.item_on_failure_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "On F"), self, self.item_on_failure_btn_cb)

        self.item_finished_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Back to blocks"), self, self.item_finished_btn_cb)

        self.items_list = None

        group_visible((self.item_finished_btn, self.item_run_btn,
                       self.item_on_success_btn, self.item_on_failure_btn, self.item_edit_btn), False)

        # readonly (program running) "view"
        self.pr_pause_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Pause"), self, self.pr_pause_btn_cb)

        if self.stopped:
            self.pr_pause_btn.set_caption(translate("ProgramItem", "Resume"))

        self.pr_cancel_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Stop"), self, self.pr_cancel_btn_cb)

        group_visible((self.pr_pause_btn, self.pr_cancel_btn), False)

        self.fixed = False

        self.editing_item = False
        self.edit_request = False
        self.run_request = False

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.setZValue(100)

        self._update_learned()
        self.update()

        if self.item_switched_cb:
            self.item_switched_cb(None, None, blocks=True)

    def pr_pause_btn_cb(self, btn):

        if self.pause_cb is not None:
            ret = self.pause_cb()

            if ret:

                # set disabled and wait for state update
                self.set_enabled(False)

    def pr_cancel_btn_cb(self, btn):

        if self.cancel_cb is not None:
            ret = self.cancel_cb()

            if ret:

                # set disabled and wait for state update
                self.set_enabled(False)

    def _update_learned(self):

        if self.block_id is not None:
            self.block_learned = self.ph.block_learned(self.block_id)
        self.program_learned = self.ph.program_learned()

    def set_readonly(self, readonly):

        self.readonly = readonly

        if self.readonly:

            if self.items_list is not None:

                self.items_list.setVisible(True)
                self.items_list.setEnabled(False)

            self.blocks_list.set_enabled(False, True)

            group_visible((self.block_finished_btn,
                           self.block_edit_btn, self.block_on_failure_btn), False)
            group_enable((self.pr_pause_btn, self.pr_cancel_btn), True)

        else:

            # TODO
            pass

        self.update()

    def set_program_btns_enabled(self, state):

        group_enable((self.pr_pause_btn, self.pr_cancel_btn), state)

    def set_active(self, block_id, item_id):

        old_block_id = self.block_id

        self.block_id = block_id
        self.item_id = item_id

        if old_block_id != self.block_id:

            self._init_items_list()

        self.items_list.set_current_idx(
            self.items_map_rev[self.item_id], select=True)

        self._handle_item_btns()

        if self.item_id is not None:

            group_visible((self.block_finished_btn,
                           self.block_edit_btn, self.block_on_failure_btn), False)

    def get_text_for_item(self, block_id, item_id):

        item = self.ph.get_item_msg(block_id, item_id)

        text = str(item.id)
        text += " | "

        if item.type == ProgIt.GET_READY:

            text += translate("ProgramItem", "GET_READY").toUpper()

        elif item.type == ProgIt.WAIT_FOR_USER:

            text += translate("ProgramItem", "WAIT_FOR_USER").toUpper()

        elif item.type == ProgIt.WAIT_UNTIL_USER_FINISHES:

            text += translate("ProgramItem", "WAIT_UNTIL_USER_FINISHES").toUpper()

        elif item.type == ProgIt.PICK_FROM_POLYGON:

            text += translate("ProgramItem", "PICK_FROM_POLYGON").toUpper()

        elif item.type == ProgIt.PICK_FROM_FEEDER:

            text += translate("ProgramItem", "PICK_FROM_FEEDER").toUpper()

        elif item.type == ProgIt.PLACE_TO_POSE:

            text += translate("ProgramItem", "PLACE_TO_POSE").toUpper()

            text += ' ' + translate("ProgramItem", "object from step %1").toUpper().arg(item.ref_id[0])

        elif item.type == ProgIt.PLACE_TO_GRID:

            text += translate("ProgramItem", "PLACE_TO_GRID").toUpper()

        elif item.type == ProgIt.DRILL_POINTS:

            text += translate("ProgramItem", "DRILL POINTS").toUpper()

        if len(item.ref_id) > 0:

            if self.ph.item_has_nothing_to_set(block_id, item_id):

                text += translate("ProgramItem", " (copy of %1)").arg(item.ref_id[0])
            # else:
            #    text += translate("ProgramItem", " (refers to %1)").arg(', '.join(str(x) for x in item.ref_id))

        if item.type in self.ph.ITEMS_USING_OBJECT:

            (obj, ref_id) = self.ph.get_object(block_id, item_id)

            text += "\n"

            if self.ph.is_object_set(block_id, item_id):

                obj_txt = obj[0]

            else:

                obj_txt = "??"

            text += translate("ProgramItem", "     Object type: %1").arg(obj_txt)

            if ref_id != item_id:

                text += translate("ProgramItem", " (same as in %1)").arg(ref_id)

        if item.type == ProgIt.DRILL_POINTS:

            ps, ref_id = self.ph.get_pose(block_id, item_id)
            ps_learned = 0.0

            for i in range(len(ps)):

                if self.ph.is_pose_set(block_id, item_id, i):
                    ps_learned += 1

            text += translate("ProgramItem", " learned poses: %1/%2").arg(ps_learned).arg(len(ps))

        if item.type == ProgIt.PICK_FROM_FEEDER:

            text += "\n"

            if self.ph.is_pose_set(block_id, item_id):
                text += translate("ProgramItem", "     Pose stored.")
            else:
                text += translate("ProgramItem", "     Pose has to be set.")

        text += "\n"
        text += translate("ProgramItem", "     Success: %1, failure: %2").arg(item.on_success).arg(item.on_failure)

        return text

    def _init_items_list(self):

        idata = []
        self.items_map = {}  # map from indexes (key) to item_id (value)
        self.items_map_rev = {}

        bmsg = self.ph.get_block_msg(self.block_id)

        for i in range(len(bmsg.items)):

            item_id = bmsg.items[i].id

            idata.append(self.get_text_for_item(self.block_id, item_id))
            self.items_map[i] = item_id
            self.items_map_rev[item_id] = i

        self.items_list = ListItem(self.scene(
        ), 0, 0, 0.2 - 2 * 0.005, idata, self.item_selected_cb, parent=self)

        for k, v in self.items_map.iteritems():

            if self.ph.item_requires_learning(self.block_id, v):
                self._update_item(self.block_id, v)
            else:
                self.items_list.items[k].set_enabled(False)

        y = 50
        self.items_list.setPos(self.sp, y)
        y += self.items_list._height() + self.sp

        if not self.readonly:

            self. _place_childs_horizontally(
                y, self.sp, [self.item_edit_btn, self.item_run_btn, self.item_on_success_btn, self.item_on_failure_btn])

            y += self.item_finished_btn._height() + self.sp

            self. _place_childs_horizontally(
                y, self.sp, [self.item_finished_btn])

            y += self.item_finished_btn._height() + 3 * self.sp

            group_visible((self.item_finished_btn, self.item_run_btn,
                           self.item_on_success_btn, self.item_on_failure_btn, self.item_edit_btn), True)
            self.item_finished_btn.setEnabled(True)
            group_enable((self.item_run_btn, self.item_on_failure_btn,
                          self.item_on_success_btn, self.item_on_failure_btn), False)

            group_visible((self.pr_pause_btn, self.pr_cancel_btn), False)

        else:

            self.items_list.setEnabled(False)

            self. _place_childs_horizontally(
                y, self.sp, [self.pr_pause_btn, self.pr_cancel_btn])
            y += self.pr_pause_btn._height() + 3 * self.sp

            pr = (self.pr_pause_btn, self.pr_cancel_btn)
            group_enable(pr, True)

            group_visible((self.item_finished_btn, self.item_run_btn,
                           self.item_on_success_btn, self.item_on_failure_btn, self.item_edit_btn), False)

        self.h = y
        self.update()
        if self.item_switched_cb:
            self.item_switched_cb(self.block_id, None, blocks=False)

    def block_edit_btn_cb(self, btn):

        group_visible((self.block_finished_btn, self.block_edit_btn,
                       self.item_on_success_btn, self.block_on_failure_btn, self.blocks_list), False)

        self. _init_items_list()

    def block_selected_cb(self):

        if self.blocks_list.selected_item_idx is not None:

            self.block_id = self.blocks_map[self.blocks_list.selected_item_idx]

            if self.ph.get_block_on_failure(self.block_id) != 0:
                self.block_on_failure_btn.set_enabled(True)
            else:
                self.block_on_failure_btn.set_enabled(False)

            self.block_edit_btn.set_enabled(True)

            if self.item_switched_cb:

                self.item_switched_cb(self.block_id, None, blocks=True)

            self._update_learned()

        else:

            self.block_id = None
            self.item_id = None
            self.block_edit_btn.set_enabled(False)
            self.block_on_failure_btn.set_enabled(False)

        if self.item_switched_cb is not None:
            self.item_switched_cb(self.block_id, None, blocks=True)

    def _handle_item_btns(self):

        # print ("_handle_item_btns, self.editing_item: " + str(self.editing_item))

        if not self.editing_item:

            of = self.ph.get_id_on_failure(self.block_id, self.item_id)
            os = self.ph.get_id_on_success(self.block_id, self.item_id)

            self.item_on_failure_btn.set_enabled(of[0] != 0 and not (of[0] == self.block_id and of[1] == self.item_id))
            self.item_on_success_btn.set_enabled(os[0] != 0 and not (os[0] == self.block_id and os[1] == self.item_id))

            if self.ph.item_requires_learning(self.block_id, self.item_id) and self.ph.item_learned(self.block_id, self.item_id):
                self.item_run_btn.set_enabled(True)
            else:
                self.item_run_btn.set_enabled(False)

            # TODO place pose with object through ref_id - disable Edit when object is not set
            self.item_edit_btn.set_enabled(
                self.ph.item_requires_learning(self.block_id, self.item_id) and not self.ph.item_has_nothing_to_set(self.block_id, self.item_id))

        else:

            self.item_edit_btn.set_enabled(True)
            self.item_edit_btn.set_caption(translate("ProgramItem", "Done"))
            group_enable((self.item_finished_btn, self.items_list), False)
            self.item_run_btn.set_enabled(False)
            group_visible((self.pr_cancel_btn, self.pr_pause_btn), False)

    def item_selected_cb(self):

        # print ("self.items_list.selected_item_idx", self.items_list.selected_item_idx)

        if self.items_list.selected_item_idx is not None:

            self.item_id = self.items_map[self.items_list.selected_item_idx]

            self._handle_item_btns()

            self._update_learned()

        else:

            self.item_id = None
            group_enable(
                (self.item_run_btn, self.item_on_success_btn, self.item_on_failure_btn, self.item_edit_btn), False)

        if self.item_switched_cb is not None:
            self.item_switched_cb(self.block_id, self.item_id)

    def block_on_failure_btn(self, btn):

        self.set_active(*self.ph.get_id_on_failure(self.block_id, self.item_id))

    def block_finished_btn_cb(self, btn):

        if self.done_cb is not None:

            self.done_cb()

    def item_finished_btn_cb(self, btn):

        # go back to blocks view
        group_visible((self.block_finished_btn, self.block_edit_btn,
                       self.block_on_failure_btn, self.blocks_list), True)
        group_visible((self.item_finished_btn, self.item_run_btn,
                       self.item_on_success_btn, self.item_on_failure_btn, self.item_edit_btn), False)
        self.block_selected_cb()  # TODO extract method to set buttons to proper state
        self.blocks_list.setEnabled(True)
        self.block_finished_btn.setEnabled(True)

        self.scene().removeItem(self.items_list)
        self.items_list = None
        self.item_id = None

        if self.item_switched_cb is not None:

            self.item_switched_cb(self.block_id, self.item_id)

        self.update()

    def item_on_failure_btn_cb(self, btn):

        of = self.ph.get_id_on_failure(self.block_id, self.item_id)
        self.set_active(*of)
        if self.item_switched_cb is not None:
            self.item_switched_cb(*of)

    def item_on_success_btn_cb(self, btn):

        of = self.ph.get_id_on_success(self.block_id, self.item_id)
        self.set_active(*of)
        if self.item_switched_cb is not None:
            self.item_switched_cb(*of)

    def item_run_btn_cb(self, btn):

        self.run_request = True
        self.set_enabled(False)
        self.learning_request_cb(LearningRequestGoal.EXECUTE_ITEM)

    def item_edit_btn_cb(self, btn):

        self.edit_request = True

        # call action / disable all, wait for result (callback), enable editing
        if not self.editing_item:

            self.learning_request_cb(LearningRequestGoal.GET_READY)

        else:

            self.learning_request_cb(LearningRequestGoal.DONE)

        self.set_enabled(False)

    def learning_request_result(self, success):

        self.set_enabled(True)

        # TODO no success, no editing

        if self.edit_request:

            self.edit_request = False

            if not self.editing_item:

                self.editing_item = True
                self.item_edit_btn.set_caption(translate("ProgramItem", "Done"))
                group_enable((self.item_finished_btn, self.items_list, self.item_on_failure_btn, self.item_on_success_btn), False)

            else:

                self.editing_item = False
                self.item_edit_btn.set_caption(translate("ProgramItem", "Edit"))
                group_enable((self.item_finished_btn, self.items_list), True)
                self._update_learned()

            self._handle_item_btns()

        elif self.run_request:

            self.run_request = False

        if self.item_switched_cb is not None:
            self.item_switched_cb(
                self.block_id, self.item_id, not self.editing_item)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def set_place_pose(self, place):

        msg = self.get_current_item()

        assert len(msg.pose) > 0

        msg.pose[0].pose.position.x = place.position[0]
        msg.pose[0].pose.position.y = place.position[1]
        msg.pose[0].pose.position.z = place.position[2]
        msg.pose[0].pose.orientation = conversions.a2q(place.quaternion)

        self._update_item()

    '''
        Method which saves place poses of all objects (in the grid) into the ProgramItem message.
    '''

    def set_place_poses(self, poses):

        msg = self.get_current_item()

        poses_count = len(poses)
        msg_poses_count = len(msg.pose)

        # TODO nemel by byt pocet objektu v gridu spis fixni (dany strukturou programu)?

        if poses_count > msg_poses_count:
            for i in range(poses_count - msg_poses_count):
                ps = PoseStamped()
                msg.pose.append(ps)
        elif poses_count < msg_poses_count:
            for i in range(msg_poses_count - poses_count):
                msg.pose.pop()

        for i, pose in enumerate(poses):
            pos = pose.get_pos()
            msg.pose[i].pose.position.x = pos[0]
            msg.pose[i].pose.position.y = pos[1]
            msg.pose[i].pose.orientation = conversions.yaw2quaternion(pose.rotation())

        self._update_item()

    def set_pose(self, ps):

        msg = self.get_current_item()

        assert len(msg.pose) > 0

        msg.pose[0] = ps

        self._update_item()

    def update_pose(self, ps, idx):

        msg = self.get_current_item()
        msg.pose[idx] = ps
        self._update_item()

    def clear_poses(self):

        for ps in self.get_current_item().pose:

            ps.pose = Pose()

        self._update_item()

    def get_poses_count(self):
        msg = self.get_current_item()
        return len(msg.pose)

    def set_object(self, obj):

        msg = self.get_current_item()

        assert len(msg.object) > 0

        msg.object[0] = obj

        self._update_item()

    def set_polygon(self, pts):

        msg = self.get_current_item()

        assert len(msg.polygon) > 0

        del msg.polygon[0].polygon.points[:]

        for pt in pts:

            msg.polygon[0].polygon.points.append(Point32(pt[0], pt[1], 0))

        self._update_item()

    '''
        Method which saves 4 points forming a grid into the ProgramItem message.
    '''

    def set_place_grid(self, pts):

        msg = self.get_current_item()

        assert len(msg.polygon) > 0

        del msg.polygon[0].polygon.points[:]

        for pt in pts:
            msg.polygon[0].polygon.points.append(Point32(pt[0], pt[1], 0))

        self._update_item()

    def _update_block(self, block_id):

        idx = self.blocks_map_rev[block_id]

        if self.ph.block_learned(block_id):
            self.blocks_list.items[idx].set_background_color()
        else:
            self.blocks_list.items[idx].set_background_color(QtCore.Qt.red)

    def _update_item(self, block_id=None, item_id=None):

        if block_id is None:
            block_id = self.block_id

        # need to update all items in block as there might be various dependencies (ref_id)
        for idx, item_id in self.items_map.iteritems():

            if self.ph.item_learned(block_id, item_id):
                self.items_list.items[idx].set_background_color()
            else:
                self.items_list.items[idx].set_background_color(QtCore.Qt.red)

            self.items_list.items[idx].set_caption(
                self.get_text_for_item(block_id, item_id))

        self._update_block(block_id)

    def get_current_item(self):

        if self.block_id is not None and self.item_id is not None:

            return self.ph.get_item_msg(self.block_id, self.item_id)

        return None

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        # TODO measure text size / use label

        sp = self.m2pix(0.01)

        if self.items_list is not None:

            if not self.block_learned and not self.readonly:

                painter.setPen(QtCore.Qt.red)

            painter.drawText(sp, 2 * sp, translate("ProgramItem", "Program %1, block %2").arg(self.ph.get_program_id()).arg(self.block_id))
        else:

            if not self.program_learned and not self.readonly:

                painter.setPen(QtCore.Qt.red)

            painter.drawText(sp, 2 * sp, translate("ProgramItem", "Program %1").arg(self.ph.get_program_id()))

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)

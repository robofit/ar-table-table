#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem
from list_item import ListItem

translate = QtCore.QCoreApplication.translate


class ProgramListItem(Item):

    def __init__(self, scene, x, y, program_headers, learned_dict, selected_program_id=None, program_selected_cb=None, program_selection_changed_cb=None):

        self.w = 100
        self.h = 100

        self.program_headers = program_headers
        self.learned_dict = learned_dict
        self.program_selected_cb = program_selected_cb
        self.program_selection_changed_cb = program_selection_changed_cb

        super(ProgramListItem, self).__init__(scene, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)

        self.fixed = False
        self.setZValue(100)

        data = []
        self.map_from_idx_to_program_id = {}
        self.map_from_program_id_to_idx = {}

        self.program_headers.sort(key=lambda p: p.id)

        for ph in self.program_headers:

            data.append("Program " + str(ph.id) + "\n" + ph.name)
            idx = len(data) - 1
            self.map_from_idx_to_program_id[idx] = ph.id
            self.map_from_program_id_to_idx[ph.id] = idx

        self.list = ListItem(scene, 0, 0, 0.2 - 2 * 0.005, data, self.item_selected_cb, parent=self)

        for idx in range(0, len(data)):

            if not self.learned_dict[self.map_from_idx_to_program_id[idx]]:
                self.list.items[idx].set_background_color(QtCore.Qt.red)

        self.run_btn = ButtonItem(scene, 0, 0, translate("ProgramItem", "Run"), self, self.run_btn_cb)
        self.edit_btn = ButtonItem(scene, 0, 0, translate("ProgramItem", "Edit"), self, self.edit_btn_cb)
        self.template_btn = ButtonItem(scene, 0, 0, translate("ProgramItem", "Template"), self, self.template_btn_cb)

        self.run_btn.set_enabled(False)
        self.edit_btn.set_enabled(False)
        self.template_btn.set_enabled(False)

        if selected_program_id is not None:

            self.list.set_current_idx(self.map_from_program_id_to_idx[selected_program_id])

        sp = self.m2pix(0.005)
        h = 50
        self.list.setPos(sp, h)
        h += self.list._height()
        h += 2 * sp

        self. _place_childs_horizontally(h, sp, [self.run_btn, self.edit_btn, self.template_btn])

        h += self.run_btn._height()
        h += 3 * sp

        self.h = h

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.update()

    def item_selected_cb(self):

        if self.list.selected_item_idx is None:

            self.run_btn.set_enabled(False)
            self.edit_btn.set_enabled(False)
            self.template_btn.set_enabled(False)

            if self.program_selection_changed_cb:
                self.program_selection_changed_cb(None)

        else:

            pid = self.map_from_idx_to_program_id[self.list.selected_item_idx]
            self.run_btn.setEnabled(self.learned_dict[pid])

            for ph in self.program_headers:

                if ph.id == pid:

                    self.edit_btn.set_enabled(not ph.readonly)
                    break

            self.template_btn.set_enabled(True)

            if self.program_selection_changed_cb:
                self.program_selection_changed_cb(ph.id, ro=ph.readonly, learned=self.learned_dict[ph.id])

    def get_current_header(self):

        return self.program_headers[self.list.selected_item_idx]

    def run_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, run=True)

    def edit_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id)

    def template_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, template=True)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        sp = self.m2pix(0.01)
        painter.drawText(sp, 2 * sp, translate("ProgramListItem", "Program list"))

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)

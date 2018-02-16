#!/usr/bin/env python

from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.items import ObjectItem, PlaceItem, LabelItem, ProgramItem, PolygonItem, SquareItem, DialogItem
import rospy
from art_projected_gui.helpers import conversions
from art_msgs.srv import NotifyUserRequest
from std_srvs.srv import Empty, EmptyRequest
# import time
import unicodedata


class customGraphicsView(QtGui.QGraphicsView):

    def __init__(self, parent=None):
        QtGui.QGraphicsView.__init__(self, parent)

        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def resizeEvent(self, evt=None):

        self.fitInView(self.sceneRect(), QtCore.Qt.KeepAspectRatio)


class UICore(QtCore.QObject):
    """Class holds QGraphicsScene and its content (items).

    There are methods for manipulation (add, find, delete) with items.
    There should not be any ROS-related stuff, just basic things.
    All items to be displayed (objects, places etc.) are inserted into array with few exceptions (e.g. program visualization, notifications).

    Attributes:
        x (float): x coordinate of the scene's origin (in world coordinate system, meters).
        y (float): dtto.
        width (float): Width of the scene.
        height (float): dtto
        rpm (int): Resolution per meter (pixels per meter of width/height).
        scene (QGraphicsScene): Holds all Item(s), manages (re)painting etc.
        bottom_label (LabelItem): Label for displaying messages to user.
        program_vis (ProgramItem): Item to display robot's program.
        scene_items (list): Array to hold all displayed items.
        view (QGraphicsView): To show content of the scene in debug window.
    """

    def __init__(self, x, y, width, height, rpm, scene_server_port):
        """
        Args:
            x (float): x coordinate of the scene's origin (in world coordinate system, meters).
            y (float): dtto.
            width (float): Width of the scene.
            height (float): dtto
            rpm (int): Resolution per meter (pixels per meter of width/height).
        """

        super(UICore, self).__init__()

        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.port = scene_server_port

        w = self.width * rpm
        h = self.height / self.width * w

        self.scene = QtGui.QGraphicsScene(0, 0, int(w), int(h))
        self.scene.rpm = rpm
        self.scene.setBackgroundBrush(QtCore.Qt.black)
        # self.scene.setItemIndexMethod(QtGui.QGraphicsScene.NoIndex) # should
        # be good for dynamic scenes

        self.bottom_label = LabelItem(
            self.scene, 0.30, 0.07, self.width - 0.2, 0.03)

        self.selected_object_ids = []
        self.selected_object_types = []

        self.view = customGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet("QGraphicsView { border-style: none; }")

        self.tcpServer = QtNetwork.QTcpServer(self)
        if not self.tcpServer.listen(port=self.port):

            rospy.logerr(
                'Failed to start scene TCP server on port ' + str(self.port))

        self.tcpServer.newConnection.connect(self.new_connection)
        self.connections = []

        self.scene_timer = QtCore.QTimer()
        self.connect(
            self.scene_timer,
            QtCore.SIGNAL('timeout()'),
            self.send_to_clients_evt)
        self.scene_timer.start(1.0 / 15 * 1000)

        self.sound_info_srv = rospy.ServiceProxy("/art/interface/sound/info", Empty)
        self.sound_warning_srv = rospy.ServiceProxy("/art/interface/sound/warning", Empty)

        rospy.loginfo("Waiting for sound services...")
        self.sound_info_srv.wait_for_service()
        self.sound_warning_srv.wait_for_service()

    def new_connection(self):

        rospy.loginfo('Some projector node just connected.')
        self.connections.append(self.tcpServer.nextPendingConnection())
        self.connections[-1].setSocketOption(
            QtNetwork.QAbstractSocket.LowDelayOption, 1)

        # TODO deal with disconnected clients!
        # self.connections[-1].disconnected.connect(clientConnection.deleteLater)

    def send_to_clients_evt(self):

        if len(self.connections) == 0:
            return

        # start = time.time()

        pix = QtGui.QImage(
            self.scene.width(),
            self.scene.height(),
            QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        self.scene.render(painter)
        painter.end()
        pix = pix.mirrored()

        block = QtCore.QByteArray()
        out = QtCore.QDataStream(block, QtCore.QIODevice.WriteOnly)
        out.setVersion(QtCore.QDataStream.Qt_4_0)
        out.writeUInt32(0)

        img = QtCore.QByteArray()
        buffer = QtCore.QBuffer(img)
        buffer.open(QtCore.QIODevice.WriteOnly)
        pix.save(buffer, "JPG", 95)
        out << img

        out.device().seek(0)
        out.writeUInt32(block.size() - 4)

        # print block.size()

        for con in self.connections:

            con.write(block)

        # end = time.time()
        # rospy.logdebug("Image sent in: " + str(end-start))

    def snd_info(self):

        try:
            self.sound_info_srv.call()
        except rospy.ServiceException:
            rospy.logerr("Sound service call failed!")
            pass

    def snd_warn(self):

        try:
            self.sound_warning_srv.call()
        except rospy.ServiceException:
            rospy.logerr("Sound service call failed!")
            pass

    def notif(self, msg, min_duration=10.0, temp=False,
              message_type=NotifyUserRequest.INFO):
        """Display message (notification) to the user.

        Args:
            msg (str): Message to be displayed.
            min_duration (:obj:`float`, optional): Message should be displayed for min_duration seconds at least.
            temp (:obj:`bool`, optional): temporal message disappears after min_duration and last non-temporal message is displayed instead.
        """

        log_func = rospy.loginfo
        if message_type == NotifyUserRequest.WARN:
            log_func = rospy.logwarn
        elif message_type == NotifyUserRequest.ERROR:
            log_func = rospy.logerr
        elif message_type == NotifyUserRequest.INFO:
            log_func = rospy.loginfo

        msg_ascii = unicodedata.normalize('NFKD', unicode(msg)).encode('ascii', 'ignore')

        if temp:
            log_func("Notification (temp): " + msg_ascii)
        else:
            log_func("Notification: " + msg_ascii)

        self.bottom_label.add_msg(
            msg, message_type, rospy.Duration(min_duration), temp)

    def debug_view(self):
        """Show window with scene - for debugging purposes."""

        self.view.show()

    def get_scene_items_by_type(self, itype):
        """Generator to filter content of scene_items array."""

        for el in self.scene.items():
            if type(el) == itype:
                yield el

    def remove_scene_items_by_type(self, itype):
        """Removes items of the given type from scene (from scene_items and scene)."""

        its = []

        for it in self.scene.items():

            if type(it) != itype:
                continue
            its.append(it)

        for it in its:

            self.scene.removeItem(it)

    def add_object(self, object_id, object_type, x, y, z, quaternion, sel_cb=None):
        """Adds object to the scene.

        Args:
            object_id (str): unique object ID
            object_type (str): type (category) of the object
            x, y (float): world coordinates
            sel_cb (method): Callback which gets called one the object is selected.
        """

        obj = ObjectItem(self.scene, object_id, object_type, x, y, z, quaternion, sel_cb)

        if object_id in self.selected_object_ids or object_type.name in self.selected_object_types:

            obj.set_selected(True)

    def remove_object(self, object_id):
        """Removes ObjectItem with given object_id from the scene."""

        obj = None

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == object_id:

                obj = it
                break

        if obj is not None:

            self.scene.removeItem(obj)
            return True

        return False

    def select_object(self, obj_id, unselect_others=True):
        """Sets ObjectItem with given obj_id as selected. By default, all other items are unselected."""

        if unselect_others:
            self.selected_object_ids = []

        if obj_id not in self.selected_object_ids:
            self.selected_object_ids.append(obj_id)

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == obj_id:

                it.set_selected(True)
                if not unselect_others:
                    break

            elif unselect_others:

                it.set_selected(False)

    def select_object_type(self, obj_type_name, unselect_others=True):
        """Sets all ObjectItems with geiven object_type and selected. By default, all objects of other types are unselected."""

        if unselect_others:
            self.selected_object_types = []

        if obj_type_name not in self.selected_object_types:
            self.selected_object_types.append(obj_type_name)

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_type.name == obj_type_name:
                it.set_selected(True)
            elif unselect_others:
                it.set_selected(False)

    def get_object(self, obj_id):
        """Returns ObjectItem with given object_id or None if the ID is not found."""

        for it in self.get_scene_items_by_type(ObjectItem):
            if it.object_id == obj_id:
                return it

        return None

    def add_place(self, caption, pose_stamped, object_type,
                  object_id=None, place_cb=None, fixed=False, dashed=False):

        # TODO check frame_id in pose_stamped and transform if needed
        PlaceItem(
            self.scene,
            caption,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            conversions.q2a(pose_stamped.pose.orientation),
            object_type,
            object_id,
            place_pose_changed=place_cb,
            fixed=fixed,
            dashed=dashed
        )

    def add_polygon(self, caption, obj_coords=[], poly_points=[],
                    polygon_changed=None, fixed=False):

        PolygonItem(
            self.scene,
            caption,
            obj_coords,
            poly_points,
            polygon_changed,
            fixed)

    '''
        Method which creates instance of SquareItem class.
    '''

    def add_square(self, caption, min_x, min_y, square_width, square_height, object_type, poses, grid_points=[], square_changed=None, fixed=False):

        SquareItem(self.scene, caption, min_x, min_y, square_width, square_height, object_type, poses, grid_points, self.scene.items,
                   square_changed, fixed)

    def clear_places(self):

        self.remove_scene_items_by_type(PlaceItem)

    def clear_all(self):

        self.selected_object_ids = []
        self.selected_object_types = []

        for it in self.get_scene_items_by_type(ObjectItem):

            it.set_selected(False)

        self.remove_scene_items_by_type(PlaceItem)
        self.remove_scene_items_by_type(PolygonItem)
        self.remove_scene_items_by_type(SquareItem)

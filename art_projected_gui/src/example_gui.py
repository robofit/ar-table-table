#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore, QtNetwork
from art_projected_gui.helpers import ProjectorHelper
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse
from std_msgs.msg import Empty, Bool
from std_srvs.srv import Empty as EmptySrv, EmptyRequest


class customGraphicsView(QtGui.QGraphicsView):

    def __init__(self, parent=None):
        QtGui.QGraphicsView.__init__(self, parent)

        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def resizeEvent(self, evt=None):

        self.fitInView(self.sceneRect(), QtCore.Qt.KeepAspectRatio)


class ExampleGui(QtCore.QObject):

    def __init__(self, x, y, width, height, rpm, scene_server_port):

        super(ExampleGui, self).__init__()

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

        self.view = customGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet("QGraphicsView { border-style: none; }")

        self.touch_calib_srv = rospy.Service(
            '/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.touched_sub = None
        self.projectors_calibrated_pub = rospy.Publisher('/art/interface/projected_gui/app/projectors_calibrated', Bool, queue_size=10, latch=True)
        
        
        self.calibrating_touch = False
        self.touch_calibration_points = None
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'touch_calibration_points_evt'), self.touch_calibration_evt)
        self.point_item = None
        
        touch_calibrated = rospy.wait_for_message("/art/interface/touchtable/calibrated", Bool).data
        
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

        self.projectors = [ProjectorHelper("localhost")]

        rospy.loginfo("Waiting for projector nodes...")
        for proj in self.projectors:
            proj.wait_until_available()
            if not proj.is_calibrated():
                rospy.loginfo("Starting calibration of projector: " + proj.proj_id)
                b = Bool()
                b.data = False
                self.projectors_calibrated_pub.publish(b)
                proj.calibrate(self.calibrated_cb)                    
            else:
                rospy.loginfo("Projector " + proj.proj_id + " already calibrated.")
                
        
        self.text = QtGui.QGraphicsTextItem("Hello world!", None, self.scene)
        self.text.setFont(QtGui.QFont('Arial', 148))
        self.text.setDefaultTextColor(QtCore.Qt.white)
        if not touch_calibrated:
            rospy.wait_for_service('/art/interface/touchtable/calibrate')
            rospy.loginfo(
                'Get /art/interface/touchtable/calibrate service')
            self.calibrate_table_srv_client = rospy.ServiceProxy('/art/interface/touchtable/calibrate', EmptySrv)
            req = EmptyRequest()            
            self.calibrate_table_srv_client.call(req)
            
        
        
        rospy.loginfo("Ready")

    def touch_calibration_points_cb(self, req):
        for it in self.scene.items():

            it.setVisible(False) 
        
        self.touched_sub = rospy.Subscriber(
            '/art/interface/touchtable/touch_detected', Empty, self.touch_detected_cb, queue_size=10)
        
        self.touch_calibration_points = []
        for pt in req.points:

            self.touch_calibration_points.append((pt.point.x, pt.point.y))
        self.emit(QtCore.SIGNAL('touch_calibration_points_evt'))
        resp = TouchCalibrationPointsResponse()
        resp.success = True
        return resp


    def touch_calibration_evt(self):
        self.touch_calibrating = True
        try:
            p = self.touch_calibration_points.pop(0)
            self.point_item = QtGui.QGraphicsEllipseItem(0, 0, 0.01*self.scene.rpm, 0.01*self.scene.rpm, None, self.scene)
            self.point_item.setBrush(QtGui.QBrush(QtCore.Qt.white, style = QtCore.Qt.SolidPattern))
            self.point_item.setPos(p[0]*self.scene.rpm, p[1]*self.scene.rpm)
        except IndexError:
            for it in self.scene.items():

                # TODO fix this - in makes visible even items that are invisible by purpose
                it.setVisible(True)
                self.touched_sub.unregister()
                


    def touch_detected_cb(self, data):
        try:
            p = self.touch_calibration_points.pop(0)
            #self.scene.removeItem(self.point_item)
            #del self.point_item
            #self.point_item = QtGui.QGraphicsEllipseItem(p[0]*self.scene.rpm, p[1]*self.scene.rpm, 0.01*self.scene.rpm, 0.01*self.scene.rpm, None, self.scene)
            self.point_item.setPos(p[0]*self.scene.rpm, p[1]*self.scene.rpm)
            #self.point_item.setBrush(QtGui.QBrush(QtCore.Qt.white, style = QtCore.Qt.SolidPattern))
        except IndexError:
            self.scene.removeItem(self.point_item)
            del self.point_item
            for it in self.scene.items():
                
                # TODO fix this - in makes visible even items that are invisible by purpose
                it.setVisible(True)
                self.touched_sub.unregister()


    def calibrated_cb(self, proj):
        b = Bool()
        b.data = True
        self.projectors_calibrated_pub.publish(b)
        

        rospy.loginfo("Projector " + proj.proj_id + " calibrated: " + str(proj.is_calibrated()))

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

    def debug_view(self):
        """Show window with scene - for debugging purposes."""

        self.view.show()


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()


def main(args):

    rospy.init_node('projected_gui_example', anonymous=True, log_level=rospy.DEBUG)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)

    gui = ExampleGui(0, 0, 1.00, 0.60, 2000, 1234)
    gui.debug_view()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")

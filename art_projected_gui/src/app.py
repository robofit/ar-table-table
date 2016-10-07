#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
import rospkg

#from projector import Projector
from ui_core_ros import UICoreRos

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

def main(args):

    rospy.init_node('projected_gui', anonymous=True)

    #rospy.loginfo('Waiting for art_brain services...')
    #rospy.wait_for_service('/art/brain/program/start')
    #rospy.wait_for_service('/art/brain/program/stop')
    rospy.loginfo('Waiting for art_db services...')
    rospy.wait_for_service('/art/db/program/get')
    rospy.wait_for_service('/art/db/program/store')
    rospy.loginfo('Ready!')

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)

    rospack = rospkg.RosPack()

    translator = QtCore.QTranslator()
    translator.load(QtCore.QLocale.system().name() + '.qm',  rospack.get_path('art_projected_gui') + '/lang')
    app.installTranslator(translator)

    ui = UICoreRos()

    ui.debug_view()

    #ui.add_projector(Projector(0, None))

    ui. scene_changed(None)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")

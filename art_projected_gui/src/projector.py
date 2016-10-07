#!/usr/bin/env python

from PyQt4 import QtGui, QtCore

# warpovani obrazu pro kazdy z projektoru
# podle vysky v pointcloudu se vymaskuji mista kde je neco vyssiho - aby se promitalo jen na plochu stolu ????
class Projector(QtGui.QWidget):
    
    def __init__(self, screen,  calibration):
        
        super(Projector, self).__init__()
        
        desktop = QtGui.QDesktopWidget()
        geometry = desktop.screenGeometry(screen)
        self.move(geometry.left(), geometry.top())
        self.resize(geometry.width(), geometry.height())
        
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        #self.setPalette(p)
        
        self.pix_label = QtGui.QLabel(self)
        self.pix_label.resize(self.size())
        
        self.showFullScreen()
        
    def on_resize(self, event):
        
        self.pix_label.resize(self.size())
    
    def set_img(self,  img):
        
        # TODO warp image according to calibration
        self.pix_label.setPixmap(img)

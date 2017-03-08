#!/usr/bin/env python

import sys
import numpy as np
from PyQt4 import QtGui, QtCore, uic
import pyqtgraph
import rospy
from audio_common_msgs.msg import AudioData
from rospkg import RosPack

"""Plot audio data from audio_common_msgs/AudioData
messages.

Based on https://github.com/swharden/Python-GUI-examples
and https://github.com/awesomebytes/python_qt_tutorial

Requirements:
sudo pip install pyqtgraph
sudo apt-get install python-qt4
sudo apt-get install python-numpy
sudo apt-get install ros-indigo-audio-common-msgs

Author: Sammy Pfeiffer"""


class AudioTopicPlotter(QtGui.QMainWindow):
    def __init__(self, parent=None):
        pyqtgraph.setConfigOption('background', 'w')  # before loading widget
        super(AudioTopicPlotter, self).__init__(parent)
        # We load dynamically the ui file
        rp = RosPack()
        pkg_path = rp.get_path('audio_stream_arecord_hack')
        uic.loadUi(pkg_path + '/scripts/ui_pcm_vol.ui', self)
        self.grPCM.plotItem.showGrid(True, True, 0.7)
        self.maxPCM = 0
        self.blocksize = None
        self.last_data = None
        self.new_data = False
        self.sub = rospy.Subscriber('/audio_stream/audio_data',
                                    AudioData, self.audio_cb, queue_size=10)
        rospy.loginfo('Subscribed to topic: ' + str(self.sub.resolved_name))

    def audio_cb(self, data):
        self.new_data = True
        self.last_data = np.fromstring(data.data, dtype=np.int16)
        if self.blocksize is None:
            self.blocksize = len(self.last_data)
            # The X axis is just gonna be the blocksize
            self.datax = np.arange(self.blocksize)

    def update(self):
        if self.last_data is not None and self.new_data:
            pcmMax = np.max(np.abs(self.last_data))
            # The range of the volume will be set dynamically
            if pcmMax > self.maxPCM:
                self.maxPCM = pcmMax
                self.grPCM.plotItem.setRange(yRange=[-pcmMax, pcmMax])

            self.pbLevel.setValue(1000 * pcmMax / self.maxPCM)
            pen = pyqtgraph.mkPen(color='b')
            self.grPCM.plot(self.datax, self.last_data, pen=pen, clear=True)
            self.new_data = False

        # Keep calling from the main thread
        QtCore.QTimer.singleShot(1, self.update)


if __name__ == "__main__":
    rospy.init_node('audio_plot')
    argv = rospy.myargv(sys.argv)
    app = QtGui.QApplication(argv)
    plot_window = AudioTopicPlotter()
    plot_window.show()
    # Start the update calls from the main thread
    plot_window.update()
    app.exec_()

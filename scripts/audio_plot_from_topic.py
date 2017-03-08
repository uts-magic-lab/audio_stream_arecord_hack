#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

"""Real time plotting topic audio stream"""

# Author: Sammy Pfeiffer


class AudioPlotSub(QtGui.QMainWindow):
    def __init__(self, plot_ms_update=30, buffer_blocks=16):
        rospy.loginfo("Initializing AudioPlotSub...")
        self.plot_ms_update = plot_ms_update
        self.buffer_blocks = buffer_blocks
        self.block_size = None
        self.last_data = None
        self.new_data = False
        self.configured = False
        self.sub = rospy.Subscriber('/audio_stream/audio_data',
                                    AudioData,
                                    self.audio_cb,
                                    queue_size=10)
        rospy.loginfo("Subscribing to topic: " + str(self.sub.resolved_name))

        # Timer for the plotter
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.plot_vals)
        while not self.configured and not rospy.is_shutdown():
            rospy.sleep(0.1)
        # QtGui.QApplication.instance().exec_()

    def configure_plot(self, blocksize):
        self.block_size = blocksize
        # Plotter config
        self.x = range(self.block_size * self.buffer_blocks)
        # Start with 0s in the buffer
        self.buffer = [0] * self.block_size * self.buffer_blocks
        self.plot = pg.plot()
        self.plot.setWindowTitle('Audioplot')
        self.plot.setRange(
            xRange=[0, (self.block_size) * self.buffer_blocks],
            yRange=[-32767, 32767])  # From signed int 16b
        self.timer.start(self.plot_ms_update)
        self.configured = True

        rospy.loginfo("Configured plot.")
        rospy.loginfo("Blocksize is: " + str(self.block_size))
        rospy.loginfo("Plot update rate in ms is: " + str(self.plot_ms_update))
        rospy.loginfo("Buffer block size is: " + str(self.buffer_blocks))

    def audio_cb(self, data):
        if self.last_data is None:
            self.configure_plot(len(data.data))
        self.last_data = data.data
        self.new_data = True

    def plot_vals(self):
        if self.new_data:
            y = self.buffer[self.block_size / 2:]
            last_data = np.fromstring(self.last_data, dtype=np.int16)
            y.extend(last_data)
            self.plot.plot(self.x, y, clear=True)
            self.buffer = y
            pg.QtGui.QApplication.processEvents()
            self.new_data = False


if __name__ == '__main__':
    rospy.init_node('audio_plotter')
    app = QtGui.QApplication(["plot audio"])
    as_ = AudioPlotSub()
    as_.show()
    # rospy.spin()
    # rospy.loginfo(
    #     "Waiting for plot to be configured with the first message received...")
    # while not as_.configured:
    #     rospy.sleep(0.1)
    # rospy.loginfo("Plotting!")
    # try:
    #     QtGui.QApplication.instance().exec_()
    # except KeyboardInterrupt:
    #     print "Control+C requested..."

    app.exec_()
    as_.close()

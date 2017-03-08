#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
import subprocess
import numpy as np

"""
Supper hacky script to plot audio topic data on the terminal using
gnuplot.

You need:
sudo apt-get install gnuplot

Author: Sammy Pfeiffer
"""

if __name__ == "__main__":
    rospy.init_node('audio_plot')

    def cb(data):
        y = np.fromstring(data.data, dtype=np.int16)
        x = range(len(data.data))
        gnuplot = subprocess.Popen(["/usr/bin/gnuplot"],
                                   stdin=subprocess.PIPE)
        gnuplot.stdin.write("set term dumb 200 60\n")
        gnuplot.stdin.write(
            "plot '-' using 1:2 title 'Line1' with linespoints \n")
        for i, j in zip(x, y):
            gnuplot.stdin.write("%d %d\n" % (i, j))
        gnuplot.stdin.write("e\n")
        gnuplot.stdin.flush()

    sub = rospy.Subscriber('/audio_stream/audio_data',
                           AudioData,
                           cb,
                           queue_size=10)
    rospy.spin()

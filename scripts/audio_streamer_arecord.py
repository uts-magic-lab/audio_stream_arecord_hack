#!/usr/bin/env python

import rospy
import sys
import threading
import subprocess
import numpy as np
from audio_common_msgs.msg import AudioData

"""Publish audio stream data
by using arecord to get the audio into std_out.

Note that the trick is to do '-t raw'.
"""

# Author: Sammy Pfeiffer


class AudioStreamerPub(object):
    def __init__(self, block_size=2048,
                 arecord_command=["arecord",
                                  "-f", "S16_LE",
                                  "-r", "16000",
                                  "-t", "raw"]):
        rospy.loginfo("Initializing AudioStreamerPub...")
        rospy.loginfo("Block size is: " + str(block_size))
        rospy.loginfo("arecord command is: " + str(arecord_command))
        self.arecord_command = arecord_command
        self.pub = rospy.Publisher('~audio_data', AudioData, queue_size=10)
        self.last_data = None
        rospy.loginfo("Publishing on topic: " + self.pub.resolved_name)
        self.block_size = block_size
        self.streamer = threading.Thread(target=self.stream_audio)
        self.streamer.start()

    def stream_audio(self):
        reccmd = self.arecord_command
        self.p = subprocess.Popen(reccmd, stdout=subprocess.PIPE)

        while not rospy.is_shutdown():
            self.last_data = self.p.stdout.read(self.block_size)
            if self.last_data is not None:
                ad = AudioData()
                ad.data = tuple(np.fromstring(self.last_data, dtype=np.uint8))
                self.pub.publish(ad)

        self.close()

    def close(self):
        self.p.kill()
        self.streamer.join()


if __name__ == '__main__':
    rospy.init_node('audio_stream')
    argv = rospy.myargv(sys.argv)
    block_size = 2048
    arecord_command = ["arecord",
                       "-f", "S16_LE",
                       "-r", "16000",
                       "-t", "raw"]
    if len(argv) > 1:
        block_size = argv[1]
        if len(argv) > 2:
            arecord_command = argv[2:]

    as_ = AudioStreamerPub(block_size=block_size,
                           arecord_command=arecord_command)
    as_.stream_audio()

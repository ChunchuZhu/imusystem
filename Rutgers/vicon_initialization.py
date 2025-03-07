#!/usr/bin/env python

import sys
sys.path.append('/usr/local/lib/python3.8/dist-packages')

from pyvicon import *

def vicon_init(IP_ADDRESS):
    vicon_stream = PyVicon()
    print("SDK version : {}".format(vicon_stream.__version__))
    # vicon_stream.connect("192.168.10.203")
    vicon_stream.connect(IP_ADDRESS)
    print("Connection status : {}".format(vicon_stream.is_connected()))

    vicon_stream.set_stream_mode(StreamMode.ClientPull)

    vicon_stream.enable_marker_data()
    vicon_stream.enable_segment_data()

    while True:
        vicon_stream.get_frame()
        if vicon_stream.get_subject_count():
            subject_name = vicon_stream.get_subject_name(0)
            marker_count = vicon_stream.get_marker_count(subject_name)
            # print("Subject name is ", subject_name)
            # print("The number of markers is ", marker_count)
            break

    return vicon_stream#, subject_name,  marker_count


if __name__ == "__main__":
    vicon_stream, subject_name,  marker_count = vicon_init("192.168.10.200")
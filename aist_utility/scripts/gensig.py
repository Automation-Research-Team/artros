#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, random
import numpy as np
from math             import pi, sin
from aist_utility.msg import Float32Stamped
# from scipy import signal
# from scipy import fftpack
# from matplotlib import pyplot as plt

######################################################################
#  class GenSig                                                      #
######################################################################
class GenSig(object):
    def __init__(self):
        super(GenSig, self).__init__()
        self._pub = rospy.Publisher('~out', Float32Stamped, queue_size=10)
        self._sub = rospy.Subscriber('/in', Float32Stamped, self._flt_cb)

    def run(self):
        f           = rospy.get_param('~signal_frequency', 2)
        noize_level = rospy.get_param('~noize_level', 0.2)
        rate        = rospy.Rate(rospy.get_param('~rate', 1000.0))

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dx  = random.uniform(-noize_level, noize_level)
            flt = Float32Stamped()
            flt.header.stamp = now
            flt.data = sin(2*pi*f*now.to_sec()) + dx
            self._pub.publish(flt)
            rate.sleep()

    def _flt_cb(self, flt):
        pass

if __name__ == '__main__':
    rospy.init_node('gensig', anonymous=True)
    gensig = GenSig()
    gensig.run()

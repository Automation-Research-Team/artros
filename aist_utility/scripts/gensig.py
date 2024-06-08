#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, random
from rclpy.node    import Node
from math          import pi, sin
from aist_msgs.msg import Float32Stamped

######################################################################
#  class GenSig                                                      #
######################################################################
class GenSig(Node):
    def __init__(self):
        super().__init__('gensig')

        self.declare_parameter('signal_frequency', 2.0)
        self.declare_parameter('rate', 1000.0)
        self.declare_parameter('noize_level', 0.2)

        rate = self.get_parameter('rate').get_parameter_value().double_value

        self._pub   = self.create_publisher(Float32Stamped,
                                            self.get_name() + '/out', 10)
        self._timer = self.create_timer(1.0/rate, self._timer_cb)

    def _timer_cb(self):
        frequency   = self.get_parameter('signal_frequency')\
                          .get_parameter_value().double_value
        noize_level = self.get_parameter('noize_level')\
                          .get_parameter_value().double_value

        now = self.get_clock().now()
        dx  = random.uniform(-noize_level, noize_level)
        flt = Float32Stamped()
        flt.header.stamp = now.to_msg()
        flt.data = sin(2*pi*frequency*now.nanoseconds*1e-9) + dx
        self._pub.publish(flt)

def main(args=None):
    rclpy.init(args=args)
    gensig = GenSig()

    rclpy.spin(gensig)

    gensig.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

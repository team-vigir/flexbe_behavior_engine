#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_core')
import tf


class ProxyTransformListener(object):
    listener = None
    
    def start(self):
        ProxyTransformListener.listener = tf.TransformListener()

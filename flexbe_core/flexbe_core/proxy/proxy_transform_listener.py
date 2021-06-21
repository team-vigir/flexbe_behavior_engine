#!/usr/bin/env python
import tf


#TODO implement tf client
class ProxyTransformListener(object):
    _listener = None

    def __init__(self):
        if ProxyTransformListener._listener is None:
            ProxyTransformListener._listener = tf.TransformListener()

    def listener(self):
        return ProxyTransformListener._listener

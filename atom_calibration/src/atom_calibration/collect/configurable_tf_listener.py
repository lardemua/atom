#!/usr/bin/env python3

import rospy
import threading
from tf2_msgs.msg import TFMessage
import tf2_ros


class ConfigurableTransformListener(tf2_ros.TransformListener):
    # https://github.com/ros/geometry2/issues/69

    def __init__(self, buffer, queue_size=None, buff_size=65536, tcp_nodelay=False, tf_topic='tf', tf_static_topic='tf_static'):

        # Copy past the code from the tf2_ros.TransformListener constructor method, changing the subscribed topics.
        self.buffer = buffer
        self.last_update = rospy.Time.now()
        self.last_update_lock = threading.Lock()
        self.tf_sub = rospy.Subscriber(tf_topic, TFMessage, self.callback,
                                       queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)
        self.tf_static_sub = rospy.Subscriber(tf_static_topic, TFMessage, self.static_callback,
                                              queue_size=queue_size, buff_size=buff_size, tcp_nodelay=tcp_nodelay)

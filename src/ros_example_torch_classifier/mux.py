#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from ros_example_torch_classifier.msg import StringStamped

from ros_example_torch_classifier.utils import check_remap
## TODO: make generic ???
## we want to read title and abstract and output titleabstract

def append_pair(title, abstract):
    rospy.loginfo("dddddd")
    mys = StringStamped()
    mys.header = title.header
    mys.data = " ".join([title.data, abstract.data])
    mypub.publish(mys)

rospy.init_node("mux", log_level=rospy.DEBUG)


check_remap(["title", "abstract", "titleabstract"], level="error")
title    = message_filters.Subscriber("title", StringStamped)
abstract = message_filters.Subscriber("abstract", StringStamped)

mypub = rospy.Publisher("titleabstract", StringStamped, queue_size=10 )

muxin = [title, abstract]

ts = message_filters.TimeSynchronizer(muxin, 10)
ts.registerCallback(append_pair)

rospy.spin()

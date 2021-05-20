#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from ros_example_torch_classifier.msg import StringStamped

import ros_example_torch_classifier.utils as u

def append_pair(in1, in2):
    u.nlogdebug(in1.index)
    mys = StringStamped()
    if not in1.index == in2.index:
        u.nlogerr("Index mismatch between equally timestepped frames!\nIndex of input1 is {} and input2 is {}".format(in1.index,in2.index))
    else:
        mys.header = in1.header
        mys.data = " ".join([in1.data, in2.data])
        mypub.publish(mys)

try:
    rospy.init_node("mux", log_level=rospy.INFO)
    #    rospy.init_node("mux", log_level=rospy.DEBUG)

    input1 = rospy.resolve_name("in1") # im desperate
    input2 = rospy.resolve_name("in2") #
    output = rospy.resolve_name("output")
    u.check_remap([input1, input2, output], level="info")

    myrate = rospy.Rate(2)

    #rospy.wait_for_message(input1, StringStamped)
    #rospy.wait_for_message(input2, StringStamped)

    ##this is waiting for the topic without showing that it reads the topic
    u.wait_for_topics([input1, input2])
    u.nloginfo("MUX: OK")
 
    in1 = message_filters.Subscriber(input1, StringStamped)
    in2 = message_filters.Subscriber(input2, StringStamped)

    mypub = rospy.Publisher(output, StringStamped, queue_size=10 )

    muxin = [in1, in2]

    ts = message_filters.TimeSynchronizer(muxin, 10)
    ts.registerCallback(append_pair)

    rospy.spin()
except rospy.ROSInterruptException:
    pass

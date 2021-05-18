#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from ros_example_torch_classifier.msg import StringStamped

from ros_example_torch_classifier.utils import check_remap
## TODO: make generic ???
## we want to read title and abstract and output titleabstract

def append_pair(in1, in2):
    rospy.loginfo(in1.index)
    mys = StringStamped()
    if not in1.index == in2.index:
        rospy.logerr("Index mismatch between equally timestepped frames!\nIndex of input1 is {} and input2 is {}".format(in1.index,in2.index))
    else:
        mys.header = in1.header
        mys.data = " ".join([in1.data, in2.data])
        mypub.publish(mys)

try:
    rospy.init_node("mux", log_level=rospy.DEBUG)

    input1 = rospy.resolve_name("in1") # im desperate
    input2 = rospy.resolve_name("in2") #
    output = rospy.resolve_name("output")
    check_remap([input1, input2, output], level="error")

    myrate = rospy.Rate(2)

    #rospy.wait_for_message(input1, StringStamped)
    #rospy.wait_for_message(input2, StringStamped)

    while(True):
        for tup in rospy.get_published_topics():
            rospy.logwarn(tup)
            if input1 in tup or input2 in tup:            
                rospy.logerr("Im` OK")
                break
        else: ##witchcraft
            rospy.logwarn("Trust me: Im not OK")
            myrate.sleep()
            continue
        break

    in1 = message_filters.Subscriber(input1, StringStamped)
    in2 = message_filters.Subscriber(input2, StringStamped)

    mypub = rospy.Publisher(output, StringStamped, queue_size=10 )

    muxin = [in1, in2]

    ts = message_filters.TimeSynchronizer(muxin, 10)
    ts.registerCallback(append_pair)

    rospy.spin()
except rospy.ROSInterruptException:
    pass

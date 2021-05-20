#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import numpy
from ros_example_torch_classifier.msg import StringStamped 
import ros_example_torch_classifier.utils as u 


def callback(ss):
    print(ss)
    mys = StringStamped()
    mys.index = ss.index
    mys.header = ss.header
    if "Included" in ss.data or "Maybe" in ss.data:
        mys.data = "1"
    elif "Excluded" in ss.data:
        mys.data = "0"
    else:
        mys.data = ""
    u.nlogdebug(mys)

    mypub.publish(mys)

try:
    rospy.init_node("labeller", log_level=rospy.DEBUG)
    mypub = rospy.Publisher("out", StringStamped, queue_size=10)
    rospy.Subscriber("in", StringStamped, callback)
    rospy.spin()

except rospy.ROSInterruptException:
    pass


### we copy paste what we did for the paper here, but need to read from a string topic and publish something

def old():
    articles["titleabstract"] = articles["title"] + " " + articles["abstract"].fillna('')

    articles["labelled"] = [not pd.isnull(s)
                and ("Included" in s or "Excluded" in s or "Maybe" in s) for s in articles['notes']]

    print(articles.shape)

    lab_df = articles.loc[articles['labelled']].copy()
    print("Labelled data: {}".format(len(lab_df)))

    # included list
    includedlist = [("Included" in art) for art in articles["notes"].fillna('') ]

    # we maybe do not want the maybes in the future?
    maybelist = [("Maybe" in art) for art in articles["notes"].fillna('') ]

    articles["class"] =  np.logical_or(includedlist, maybelist) ## python has a strange behaviour for or, but numpy saves the day
